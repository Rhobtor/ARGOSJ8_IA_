// terrain_slope_node.cpp
// ROS 2 node that listens to an OctoMap, extracts the ground surface and estimates local slope.
// Publishes a MarkerArray where each sphere is coloured by the slope angle (green ≈ flat, red ≈ steep).
//
// Author: ChatGPT 2025‑05‑17

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <unordered_map>
#include <memory>
#include <limits>
#include <cmath>

/** Grid key utilities -----------------------------------------------------*/
struct CellKey
{
  int x;
  int y;
  bool operator==(const CellKey &other) const noexcept { return x == other.x && y == other.y; }
};

struct CellKeyHasher
{
  std::size_t operator()(const CellKey &k) const noexcept
  {
    // Pair‑hash taken from boost::hash_combine idea
    return static_cast<std::size_t>(k.x) * 73856093u ^ static_cast<std::size_t>(k.y) * 19349663u;
  }
};

/** Main node --------------------------------------------------------------*/
class TerrainSlopeNode : public rclcpp::Node
{
public:
  TerrainSlopeNode() : Node("terrain_slope_node"), tf_buffer_(this->get_clock())
  {
    /* ---- Parámetros ---- */
    this->declare_parameter("cell_size", 0.5);                       // [m]
    this->declare_parameter("max_ground_height_variation", 0.25);    // [m] para filtrar ruido / obstáculos
    this->declare_parameter("min_slope_deg", 2.0);                   // descartar pendientes menores a X deg
    this->declare_parameter("roi_max_distance", 40.0);               // radio máximo en m

    cell_size_                 = this->get_parameter("cell_size").as_double();
    max_ground_variation_      = this->get_parameter("max_ground_height_variation").as_double();
    min_slope_deg_             = this->get_parameter("min_slope_deg").as_double();
    roi_max_distance_          = this->get_parameter("roi_max_distance").as_double();

    /* ---- Comms ---- */
    slope_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("terrain_slope", 10);

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_full", 10,
      std::bind(&TerrainSlopeNode::octomapCallback, this, std::placeholders::_1));

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this);

    RCLCPP_INFO(get_logger(), "Terrain slope node ready. Cell %.2fm", cell_size_);
  }

private:
  /* ------------------------------ CALLBACK ------------------------------ */
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    // Pose del robot para ROI
    geometry_msgs::msg::TransformStamped tf_map_base;
    try {
      tf_map_base = tf_buffer_.lookupTransform("map", "base_link", rclcpp::Time(0));
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN(get_logger(), "TF error: %s", e.what());
      return;
    }
    const auto base_x = tf_map_base.transform.translation.x;
    const auto base_y = tf_map_base.transform.translation.y;
    const auto base_z = tf_map_base.transform.translation.z;

    // Convertir mensaje a OcTree
    std::unique_ptr<octomap::OcTree> tree(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg)));
    if (!tree) {
      RCLCPP_ERROR(get_logger(), "Octomap conversion failed");
      return;
    }

    /* ---- 1. Construir grid con altura mínima (supuesto suelo) ---- */
    std::unordered_map<CellKey, double, CellKeyHasher> ground_height; // z mínima

    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
      if (tree->isNodeOccupied(*it)) {
        continue; // Sólo nodos libres
      }
      const auto &p   = it.getCoordinate();
      const double dx = p.x() - base_x;
      const double dy = p.y() - base_y;
      const double dist_sq = dx*dx + dy*dy;
      if (dist_sq > roi_max_distance_ * roi_max_distance_) {
        continue; // fuera ROI
      }

      CellKey key { static_cast<int>(std::floor(p.x() / cell_size_)),
                    static_cast<int>(std::floor(p.y() / cell_size_)) };

      auto it_height = ground_height.find(key);
      if (it_height == ground_height.end()) {
        ground_height.emplace(key, p.z());
      } else {
        it_height->second = std::min(it_height->second, static_cast<double>(p.z()));
      }
    }

    /* ---- 2. Calcular pendiente local ---- */
    visualization_msgs::msg::MarkerArray markers;
    int id = 0;

    for (const auto &kv : ground_height) {
      const CellKey &key = kv.first;
      const double   z0  = kv.second;

      // Buscar vecinos 4‑direcciones
      double dz_dx = 0.0; // derivada parcial en x
      double dz_dy = 0.0; // derivada parcial en y
      bool   have_dx = false, have_dy = false;

      const auto neighbor = [&](int nx, int ny, double &out_dz, bool &flag) {
        const auto it_n = ground_height.find({nx, ny});
        if (it_n != ground_height.end()) {
          const double dz = it_n->second - z0;
          if (std::fabs(dz) < max_ground_variation_) {
            out_dz = dz;
            flag   = true;
          }
        }
      };

      neighbor(key.x + 1, key.y, dz_dx, have_dx);
      neighbor(key.x, key.y + 1, dz_dy, have_dy);

      if (!have_dx && !have_dy) {
        continue; // no vecinos válidos
      }

      const double grad_mag = std::sqrt((have_dx ? dz_dx : 0.0)*(have_dx ? dz_dx : 0.0) +
                                        (have_dy ? dz_dy : 0.0)*(have_dy ? dz_dy : 0.0)) / cell_size_;
      const double slope_rad = std::atan(grad_mag);
      const double slope_deg = slope_rad * 180.0 / M_PI;
      if (slope_deg < min_slope_deg_) {
        continue; // prácticamente plano
      }

      // Marker sphere at cell centre
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp    = this->now();
      m.ns   = "terrain_slope";
      m.id   = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = (key.x + 0.5) * cell_size_;
      m.pose.position.y = (key.y + 0.5) * cell_size_;
      m.pose.position.z = z0 + 0.05; // ligeramente sobre el suelo
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.2;

      // Colorear: verde‑amarillo‑rojo según pendiente 0‑45º
      const double t = std::clamp(slope_deg / 45.0, 0.0, 1.0);
      m.color.r = t;
      m.color.g = 1.0 - t;
      m.color.b = 0.0;
      m.color.a = 0.9;
      m.lifetime = rclcpp::Duration::from_seconds(0.0);
      markers.markers.push_back(m);
    }

    slope_markers_pub_->publish(markers);
    RCLCPP_DEBUG(get_logger(), "Publicados %zu markers de pendiente", markers.markers.size());
  }

  /* ------------------------------ Vars ------------------------------ */
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slope_markers_pub_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parámetros
  double cell_size_;
  double max_ground_variation_;
  double min_slope_deg_;
  double roi_max_distance_;
};

/* -------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainSlopeNode>());
  rclcpp::shutdown();
  return 0;
}
