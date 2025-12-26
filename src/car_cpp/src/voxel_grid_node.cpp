#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <unordered_map>
#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <cstring>   // memcpy
#include <limits>

using std::placeholders::_1;

class VoxelGridNode : public rclcpp::Node
{
public:
  VoxelGridNode()
  : Node("voxel_grid_cpp")
  {
    // ---- Parámetros de IO ----
    input_topic_  = this->declare_parameter<std::string>(
          "input", "/camera/points");
    output_topic_ = this->declare_parameter<std::string>(
          "output", "/merged_points_filtered");
    leaf_size_    = this->declare_parameter<double>("leaf_size", 0.15);
    frame_id_     = this->declare_parameter<std::string>("frame_id", "base_link");

    // ---- Filtros de rango y altura ----
    min_range_ = this->declare_parameter<double>("min_range", 0.3);
    max_range_ = this->declare_parameter<double>("max_range", 20.0);
    min_z_     = this->declare_parameter<double>("min_z",   -5.0);
    max_z_     = this->declare_parameter<double>("max_z",    1.5);

    // ---- Filtro por densidad de voxel ----
    min_points_per_voxel_ =
        this->declare_parameter<int>("min_points_per_voxel", 3);

    // ---- Caja de exclusión opcional ----
    exclude_enabled_ = this->declare_parameter<bool>("exclude_box_enabled", true);
    exclude_frame_   = this->declare_parameter<std::string>("exclude_box_frame", "base_link");
    ex_cx_ = this->declare_parameter<double>("exclude_box_center_x", 0.35);
    ex_cy_ = this->declare_parameter<double>("exclude_box_center_y", 0.0);
    ex_cz_ = this->declare_parameter<double>("exclude_box_center_z", 0.35);
    ex_sx_ = this->declare_parameter<double>("exclude_box_size_x",   3.1);
    ex_sy_ = this->declare_parameter<double>("exclude_box_size_y",   1.8);
    ex_sz_ = this->declare_parameter<double>("exclude_box_size_z",   1.4);

    // ---- Pub/Sub ----
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 5);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 5, std::bind(&VoxelGridNode::cloudCallback, this, _1));

    // ---- TF para la caja de exclusión ----
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(
      this->get_logger(),
      "VoxelGridCpp: %s -> %s (leaf=%.3f m, min_range=%.2f, max_range=%.2f, "
      "min_z=%.2f, max_z=%.2f, min_points_per_voxel=%d). Caja: %s frame=%s",
      input_topic_.c_str(), output_topic_.c_str(), leaf_size_,
      min_range_, max_range_, min_z_, max_z_, min_points_per_voxel_,
      exclude_enabled_ ? "ON" : "OFF", exclude_frame_.c_str()
    );
  }

private:
  // ---- Tipos auxiliares para el voxel grid ----
  struct KeyHasher {
    std::size_t operator()(const std::array<int,3>& k) const noexcept {
      // hash típico para 3 enteros
      return static_cast<std::size_t>(k[0]) * 73856093u ^
             static_cast<std::size_t>(k[1]) * 19349663u ^
             static_cast<std::size_t>(k[2]) * 83492791u;
    }
  };

  struct KeyEqual {
    bool operator()(const std::array<int,3>& a,
                    const std::array<int,3>& b) const noexcept {
      return a[0]==b[0] && a[1]==b[1] && a[2]==b[2];
    }
  };

  struct Accum {
    float sx, sy, sz;
    int count;
    Accum() : sx(0.f), sy(0.f), sz(0.f), count(0) {}
  };

  using VoxelMap = std::unordered_map<std::array<int,3>, Accum, KeyHasher, KeyEqual>;

  // ---- Callback principal ----
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // nº de puntos
    const std::size_t npts = static_cast<std::size_t>(msg->width) *
                             static_cast<std::size_t>(msg->height);
    if (npts == 0) {
      return;
    }

    // Sólo soportamos little-endian (ZED lo es)
    if (msg->is_bigendian) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Nube big-endian no soportada.");
      return;
    }

    // Buscar campos x,y,z
    const sensor_msgs::msg::PointField *fx=nullptr, *fy=nullptr, *fz=nullptr;
    for (const auto & f : msg->fields) {
      if (f.name == "x") fx = &f;
      else if (f.name == "y") fy = &f;
      else if (f.name == "z") fz = &f;
    }
    if (!fx || !fy || !fz) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "La nube no tiene campos x,y,z.");
      return;
    }
    if (fx->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        fy->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        fz->datatype != sensor_msgs::msg::PointField::FLOAT32) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Los campos x,y,z deben ser FLOAT32.");
      return;
    }

    const uint8_t * data = msg->data.data();
    const std::size_t step = static_cast<std::size_t>(msg->point_step);

    // ---- Preparar TF de la caja (si se usa) ----
    tf2::Transform box_tf;    // transform: cloud_frame -> exclude_frame
    bool use_box_tf = false;
    double half_x = ex_sx_ * 0.5;
    double half_y = ex_sy_ * 0.5;
    double half_z = ex_sz_ * 0.5;

    if (exclude_enabled_ && ex_sx_ > 0.0 && ex_sy_ > 0.0 && ex_sz_ > 0.0) {
      if (exclude_frame_ == msg->header.frame_id) {
        // misma frame: transform identidad
        box_tf.setIdentity();
        use_box_tf = true;
      } else {
        try {
          auto ts = tf_buffer_->lookupTransform(
            exclude_frame_, msg->header.frame_id, tf2::TimePointZero);
          tf2::fromMsg(ts.transform, box_tf);
          use_box_tf = true;
        } catch (tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "No TF %s <- %s. Caja de exclusión omitida. (%s)",
            exclude_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
          use_box_tf = false;
        }
      }
    }

    // ---- Voxel grid con filtro de densidad ----
    VoxelMap voxels;
    voxels.reserve(npts / 4);   // heurístico

    for (std::size_t i = 0; i < npts; ++i) {
      const uint8_t * base = data + i * step;

      float x, y, z;
      std::memcpy(&x, base + fx->offset, sizeof(float));
      std::memcpy(&y, base + fy->offset, sizeof(float));
      std::memcpy(&z, base + fz->offset, sizeof(float));

      // NaN/Inf
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      // rango y altura
      float dist = std::sqrt(x*x + y*y + z*z);
      if (dist < static_cast<float>(min_range_) ||
          dist > static_cast<float>(max_range_)) {
        continue;
      }
      if (z < static_cast<float>(min_z_) ||
          z > static_cast<float>(max_z_)) {
        continue;
      }

      // caja de exclusión (si está activa)
      if (use_box_tf) {
        tf2::Vector3 p_src(x, y, z);
        tf2::Vector3 p_box = box_tf * p_src;

        double rx = p_box.x() - ex_cx_;
        double ry = p_box.y() - ex_cy_;
        double rz = p_box.z() - ex_cz_;

        if (std::fabs(rx) <= half_x &&
            std::fabs(ry) <= half_y &&
            std::fabs(rz) <= half_z) {
          // dentro de la caja -> descartar
          continue;
        }
      }

      // cuantización a voxel
      int ix = static_cast<int>(std::floor(x / static_cast<float>(leaf_size_)));
      int iy = static_cast<int>(std::floor(y / static_cast<float>(leaf_size_)));
      int iz = static_cast<int>(std::floor(z / static_cast<float>(leaf_size_)));

      std::array<int,3> key{{ix, iy, iz}};
      auto & acc = voxels[key];
      acc.sx += x;
      acc.sy += y;
      acc.sz += z;
      acc.count += 1;
    }

    if (voxels.empty()) {
      return;
    }

    // ---- Construir vector final de puntos (centroide por voxel) ----
    std::vector<float> out_xyz;
    out_xyz.reserve(voxels.size() * 3);

    for (const auto & kv : voxels) {
      const Accum & acc = kv.second;
      if (acc.count < min_points_per_voxel_) {
        continue;  // muy pocos puntos -> ruido
      }
      float inv = 1.0f / static_cast<float>(acc.count);
      out_xyz.push_back(acc.sx * inv);
      out_xyz.push_back(acc.sy * inv);
      out_xyz.push_back(acc.sz * inv);
    }

    if (out_xyz.empty()) {
      return;
    }

    // ---- Empaquetar PointCloud2 x,y,z compacta ----
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = frame_id_.empty() ? msg->header.frame_id : frame_id_;

    out.height = 1;
    out.width  = static_cast<uint32_t>(out_xyz.size() / 3);
    out.is_bigendian = false;
    out.is_dense = true;

    out.point_step = 12; // 3 * float32
    out.row_step   = out.point_step * out.width;

    out.fields.resize(3);
    out.fields[0].name = "x";
    out.fields[0].offset = 0;
    out.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[0].count = 1;

    out.fields[1].name = "y";
    out.fields[1].offset = 4;
    out.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[1].count = 1;

    out.fields[2].name = "z";
    out.fields[2].offset = 8;
    out.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[2].count = 1;

    out.data.resize(out.row_step * out.height);
    std::memcpy(out.data.data(),
                out_xyz.data(),
                out_xyz.size() * sizeof(float));

    pub_->publish(out);
  }

  // ---- miembros ----
  std::string input_topic_;
  std::string output_topic_;
  double leaf_size_;
  std::string frame_id_;

  double min_range_, max_range_, min_z_, max_z_;
  int    min_points_per_voxel_;

  bool   exclude_enabled_;
  std::string exclude_frame_;
  double ex_cx_, ex_cy_, ex_cz_;
  double ex_sx_, ex_sy_, ex_sz_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelGridNode>());
  rclcpp::shutdown();
  return 0;
}
