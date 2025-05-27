// terrain_slope_node.cpp
// ROS 2 node that listens to an OctoMap, extracts the ground surface and estimates the local slope.
// Publishes:
//   • visualization_msgs/MarkerArray “terrain_slope”  – spheres colour-coded by slope angle
//   • geometry_msgs/PoseArray       “terrain_slope_pose” – centres of the cells with slope ≥ min_slope_deg
//
// Autor: ChatGPT – 27 may 2025

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <unordered_map>
#include <algorithm>   // std::clamp
#include <cmath>
#include <memory>

/* ------------------------------------------------------------------------- */
/* ----------------------------- Grid utilities ---------------------------- */
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
    // pair-hash idea (x and y are small ints)
    return static_cast<std::size_t>(k.x) * 73856093u ^ static_cast<std::size_t>(k.y) * 19349663u;
  }
};

/* ------------------------------------------------------------------------- */
/* --------------------------------  Node  -------------------------------- */
class TerrainSlopeNode : public rclcpp::Node
{
public:
  TerrainSlopeNode()
  : Node("terrain_slope_node"),
    tf_buffer_(this->get_clock())                // clock needed by tf buffer
  {
    /* -------- Parámetros -------- */
    this->declare_parameter("cell_size",                1.0);   // [m]
    this->declare_parameter("max_ground_height_variation", 0.25); // [m]
    this->declare_parameter("min_slope_deg",            2.0);   // [deg]
    this->declare_parameter("roi_max_distance",        20.0);   // [m]

    cell_size_            = this->get_parameter("cell_size").as_double();
    max_ground_variation_ = this->get_parameter("max_ground_height_variation").as_double();
    min_slope_deg_        = this->get_parameter("min_slope_deg").as_double();
    roi_max_distance_     = this->get_parameter("roi_max_distance").as_double();

    /* -------- Comms -------- */
    slope_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("terrain_slope", 10);
    slope_pose_array_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>("terrain_slope_pose", 10);

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_full", 10,
      std::bind(&TerrainSlopeNode::octomapCallback, this, std::placeholders::_1));

    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    RCLCPP_INFO(get_logger(), "Terrain-slope node listo. cell_size = %.2f m", cell_size_);
  }

private:
  /* ------------------------------ Callback ------------------------------ */
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    /* --- 0. Pose de referencia para limitar el ROI --- */
    geometry_msgs::msg::TransformStamped tf_map_base;
    try {
      tf_map_base = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN(get_logger(), "TF lookup failed: %s", e.what());
      return;
    }
    const double base_x = tf_map_base.transform.translation.x;
    const double base_y = tf_map_base.transform.translation.y;

    /* --- 1. Convertir el mensaje a OcTree --- */
    std::unique_ptr<octomap::OcTree> tree(
      dynamic_cast<octomap::OcTree *>(octomap_msgs::fullMsgToMap(*msg)));
    if (!tree) {
      RCLCPP_ERROR(get_logger(), "No pude convertir el Octomap a OcTree");
      return;
    }

    /* --- 2. Construir grid con la altura mínima Z (suelo) --- */
    std::unordered_map<CellKey, double, CellKeyHasher> ground_height;   // z mínima por celda

    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
      if (tree->isNodeOccupied(*it)) {          // solo consideramos nodos libres
        continue;
      }
      const auto &p = it.getCoordinate();
      const double dx = p.x() - base_x;
      const double dy = p.y() - base_y;
      if (dx*dx + dy*dy > roi_max_distance_ * roi_max_distance_) {
        continue;                               // fuera del ROI
      }
      CellKey key {
        static_cast<int>(std::floor(p.x() / cell_size_)),
        static_cast<int>(std::floor(p.y() / cell_size_))
      };
      auto it_h = ground_height.find(key);
      if (it_h == ground_height.end()) {
        ground_height.emplace(key, p.z());
      } else {
        it_h->second = std::min(it_h->second, static_cast<double>(p.z()));
      }
    }

    /* --- 3. Calcular pendiente local, preparar mensajes --- */
    visualization_msgs::msg::MarkerArray markers;
    geometry_msgs::msg::PoseArray        pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp    = this->now();

    int marker_id = 0;
    for (const auto &kv : ground_height) {
      const CellKey key = kv.first;
      const double  z0  = kv.second;

      /* vecinos en +x y +y (4-conectados) */
      const auto dz_from_neigh = [&](int nx, int ny, bool &ok) -> double {
        auto it_n = ground_height.find({nx, ny});
        if (it_n == ground_height.end()) { ok = false; return 0.0; }
        double dz = it_n->second - z0;
        ok = std::fabs(dz) < max_ground_variation_;
        return ok ? dz : 0.0;
      };

      bool have_dx = false, have_dy = false;
      double dz_dx = dz_from_neigh(key.x + 1, key.y    , have_dx);
      double dz_dy = dz_from_neigh(key.x    , key.y + 1, have_dy);
      if (!have_dx && !have_dy) { continue; }           // sin datos fiables

      double grad_mag = std::sqrt((have_dx?dz_dx:0.0)*(have_dx?dz_dx:0.0) +
                                  (have_dy?dz_dy:0.0)*(have_dy?dz_dy:0.0)) / cell_size_;
      double slope_deg = std::atan(grad_mag) * 180.0 / M_PI;
      if (slope_deg < min_slope_deg_) { continue; }     // prácticamente plano

      /* ---- Marker (esfera coloreada) ---- */
      visualization_msgs::msg::Marker m;
      m.header = pose_array.header;
      m.ns     = "terrain_slope";
      m.id     = marker_id++;
      m.type   = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = (key.x + 0.5) * cell_size_;
      m.pose.position.y = (key.y + 0.5) * cell_size_;
      m.pose.position.z = z0 + 0.05;          // ligera elevación visual
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.2;

      double t = std::clamp(slope_deg / 45.0, 0.0, 1.0);   // 0°→verde  45°→rojo
      m.color.r = t;
      m.color.g = 1.0 - t;
      m.color.b = 0.0;
      m.color.a = 0.9;
      m.lifetime = rclcpp::Duration(0, 0);   // 0 = infinito
      markers.markers.push_back(m);

      /* ---- Pose para el PoseArray ---- */
      geometry_msgs::msg::Pose p;
      p.position = m.pose.position;
      p.orientation.w = 1.0;
      pose_array.poses.push_back(p);
    }

    /* --- 4. Publicar --- */
    slope_markers_pub_->publish(markers);
    slope_pose_array_pub_->publish(pose_array);
    RCLCPP_DEBUG(get_logger(), "Publicados %zu markers con pendiente ≥ %.1f °",
                 markers.markers.size(), min_slope_deg_);
  }

  /* --------------------------- Miembros privados ------------------------- */
  /* publishers / subscribers */
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slope_markers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr        slope_pose_array_pub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr        octomap_sub_;

  /* TF */
  tf2_ros::Buffer                                   tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>       tf_listener_;

  /* parámetros – valores ya extraídos */
  double cell_size_;
  double max_ground_variation_;
  double min_slope_deg_;
  double roi_max_distance_;
};

/* ------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerrainSlopeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
