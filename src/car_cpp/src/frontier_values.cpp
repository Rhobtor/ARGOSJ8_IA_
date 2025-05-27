// // #include <rclcpp/rclcpp.hpp>
// // #include <nav_msgs/msg/occupancy_grid.hpp>
// // #include <visualization_msgs/msg/marker.hpp>
// // #include <geometry_msgs/msg/point.hpp>
// // #include <geometry_msgs/msg/pose_array.hpp>
// // #include <geometry_msgs/msg/pose.hpp>
// // #include <std_msgs/msg/float64.hpp>
// // #include <std_msgs/msg/float64_multi_array.hpp>
// // #include <vector>
// // #include <cmath>
// // #include <algorithm>
// // #include <utility>

// // using std::placeholders::_1;

// // // Parámetros ajustables
// // constexpr double SAFE_DISTANCE_THRESHOLD = 3;      // Distancia mínima permitida entre centroide y obstáculo (m)
// // constexpr double CLUSTER_DISTANCE_THRESHOLD = 4;     // Distancia máxima para agrupar puntos en un mismo cluster

// // // Función para calcular la distancia euclidiana entre dos puntos
// // double euclideanDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
// //   return std::hypot(p2.x - p1.x, p2.y - p1.y);
// // }

// // class FrontierBoundaryNode : public rclcpp::Node {
// // public:
// //   FrontierBoundaryNode() : Node("frontier_boundary_node")
// //   {
// //     // Suscribirse al mapa proyectado (OccupancyGrid)
// //     occupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
// //       "/occupancy_grid", 10,
// //       std::bind(&FrontierBoundaryNode::occupancyCallback, this, _1)
// //     );
// //     // Suscribirse a los nodos ocupados (obstáculos)
// //     occupied_nodes_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
// //       "/occupied_rejected_nodes", 10,
// //       std::bind(&FrontierBoundaryNode::occupiedNodesCallback, this, _1)
// //     );
// //     // Publicadores originales
// //     marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier_marker", 10);
// //     frontier_points_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("frontier_points", 10);
// //     frontier_entropy_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("frontier_entropies", 10);
// //     total_entropy_pub_ = this->create_publisher<std_msgs::msg::Float64>("total_entropy", 10);
// //     // Publicadores para safe frontier
// //     safe_frontier_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("safe_frontier_points", 10);
// //     safe_frontier_entropy_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("safe_frontier_entropy", 10);
// //     safe_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("safe_frontier_marker", 10);
// //     total_safe_entropy_pub_ = this->create_publisher<std_msgs::msg::Float64>("total_safe_entropy", 10);

// //     RCLCPP_INFO(this->get_logger(), "Nodo de frontera basado en projected_map iniciado.");
// //   }

// // private:
// //   // Suscriptores
// //   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
// //   rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr occupied_nodes_sub_;
// //   // Publicadores originales
// //   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
// //   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr frontier_points_pub_;
// //   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr frontier_entropy_pub_;
// //   rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_entropy_pub_;
// //   // Publicadores para safe frontier
// //   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr safe_frontier_pub_;
// //   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr safe_frontier_entropy_pub_;
// //   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr safe_marker_pub_;
// //   rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_safe_entropy_pub_;

// //   // Variables para almacenar datos
// //   nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;
// //   geometry_msgs::msg::PoseArray::SharedPtr occupied_nodes_;

// //   // Función para calcular la entropía de una celda
// //   double computeCellEntropy(int8_t cell_value) {
// //     double p;
// //     if (cell_value == -1) {
// //       p = 0.5;
// //     } else {
// //       p = static_cast<double>(cell_value) / 100.0;
// //     }
// //     if (p <= 0.0 || p >= 1.0)
// //       return 0.0;
// //     return -(p * std::log(p) + (1 - p) * std::log(1 - p));
// //   }

// //   void occupancyCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
// //     occupancy_grid_ = msg;
// //     processOccupancyGrid(msg);
// //   }

// //   void occupiedNodesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
// //     occupied_nodes_ = msg;
// //   }

// //   void processOccupancyGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
// //     int width = msg->info.width;
// //     int height = msg->info.height;
// //     double resolution = msg->info.resolution;
// //     double origin_x = msg->info.origin.position.x;
// //     double origin_y = msg->info.origin.position.y;

// //     std::vector<geometry_msgs::msg::Point> frontier_points;
// //     std::vector<double> frontier_local_entropies;

// //     // Calcular la entropía total del mapa
// //     double total_entropy_sum = 0.0;
// //     for (size_t i = 0; i < msg->data.size(); ++i) {
// //       total_entropy_sum += computeCellEntropy(msg->data[i]);
// //     }
// //     double total_entropy = total_entropy_sum / msg->data.size();

// //     // Recorrer la grilla para detectar celdas frontera
// //     for (int y = 0; y < height; ++y) {
// //       for (int x = 0; x < width; ++x) {
// //         int index = y * width + x;
// //         int8_t cell_value = msg->data[index];
// //         if (cell_value == 0) {  // celda libre
// //           bool is_frontier = false;
// //           for (int dy = -1; dy <= 1 && !is_frontier; ++dy) {
// //             for (int dx = -1; dx <= 1 && !is_frontier; ++dx) {
// //               if (dx == 0 && dy == 0)
// //                 continue;
// //               int nx = x + dx;
// //               int ny = y + dy;
// //               if (nx < 0 || nx >= width || ny < 0 || ny >= height)
// //                 continue;
// //               int n_index = ny * width + nx;
// //               if (msg->data[n_index] == -1) {
// //                 is_frontier = true;
// //               }
// //             }
// //           }
// //           if (is_frontier) {
// //             geometry_msgs::msg::Point pt;
// //             pt.x = origin_x + (x + 0.5) * resolution;
// //             pt.y = origin_y + (y + 0.5) * resolution;
// //             pt.z = 0.0;
// //             frontier_points.push_back(pt);

// //             double local_entropy_sum = 0.0;
// //             int count = 0;
// //             for (int dy = -1; dy <= 1; ++dy) {
// //               for (int dx = -1; dx <= 1; ++dx) {
// //                 int nx = x + dx;
// //                 int ny = y + dy;
// //                 if (nx < 0 || nx >= width || ny < 0 || ny >= height)
// //                   continue;
// //                 int n_index = ny * width + nx;
// //                 local_entropy_sum += computeCellEntropy(msg->data[n_index]);
// //                 ++count;
// //               }
// //             }
// //             double local_entropy = (count > 0) ? local_entropy_sum / count : 0.0;
// //             frontier_local_entropies.push_back(local_entropy);
// //           }
// //         }
// //       }
// //     }

// //     // Publicar los puntos frontera originales (PoseArray)
// //     geometry_msgs::msg::PoseArray frontier_poses;
// //     frontier_poses.header = msg->header;
// //     for (const auto &pt : frontier_points) {
// //       geometry_msgs::msg::Pose pose;
// //       pose.position = pt;
// //       pose.orientation.w = 1.0;
// //       frontier_poses.poses.push_back(pose);
// //     }
// //     frontier_points_pub_->publish(frontier_poses);

// //     // Publicar marcador para visualizar los puntos frontera originales
// //     visualization_msgs::msg::Marker marker;
// //     marker.header.frame_id = msg->header.frame_id;
// //     marker.header.stamp = this->now();
// //     marker.ns = "frontier_boundary";
// //     marker.id = 0;
// //     marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
// //     marker.action = visualization_msgs::msg::Marker::ADD;
// //     marker.scale.x = resolution;
// //     marker.scale.y = resolution;
// //     marker.scale.z = resolution;
// //     marker.color.r = 1.0;
// //     marker.color.g = 0.0;
// //     marker.color.b = 0.0;
// //     marker.color.a = 1.0;
// //     marker.lifetime = rclcpp::Duration(0, 0);
// //     marker.points = frontier_points;
// //     marker_pub_->publish(marker);

// //     // Publicar entropías locales y total
// //     std_msgs::msg::Float64MultiArray entropy_array_msg;
// //     entropy_array_msg.data = frontier_local_entropies;
// //     frontier_entropy_pub_->publish(entropy_array_msg);

// //     std_msgs::msg::Float64 total_entropy_msg;
// //     total_entropy_msg.data = total_entropy;
// //     total_entropy_pub_->publish(total_entropy_msg);
// //     RCLCPP_INFO(this->get_logger(), "Entropía total del mapa: %.3f", total_entropy);

// //     // Combinar posición y entropía en un solo vector
// //     std::vector<std::pair<geometry_msgs::msg::Point, double>> frontier_points_with_entropy;
// //     if (frontier_points.size() == frontier_local_entropies.size()) {
// //       for (size_t i = 0; i < frontier_points.size(); ++i) {
// //         frontier_points_with_entropy.push_back(std::make_pair(frontier_points[i], frontier_local_entropies[i]));
// //       }
// //     }
// //     // Clustering y publicación de safe frontier points y sus entropías
// //     clusterAndPublishFrontiers(frontier_points_with_entropy);
// //   }

// //   // Función de clustering que trabaja con pares (punto, entropía)
// //   void clusterAndPublishFrontiers(const std::vector<std::pair<geometry_msgs::msg::Point, double>> &frontier_points_with_entropy) {
// //     // Agrupar en clusters simples
// //     std::vector<std::vector<std::pair<geometry_msgs::msg::Point, double>>> clusters;
// //     for (const auto &pt_pair : frontier_points_with_entropy) {
// //       const auto &pt = pt_pair.first;
// //       bool added = false;
// //       for (auto &cluster : clusters) {
// //         geometry_msgs::msg::Point centroid = computeCentroidFromPairs(cluster);
// //         if (euclideanDistance(pt, centroid) < CLUSTER_DISTANCE_THRESHOLD) {
// //           cluster.push_back(pt_pair);
// //           added = true;
// //           break;
// //         }
// //       }
// //       if (!added) {
// //         clusters.push_back({pt_pair});
// //       }
// //     }

// //     // Calcular centroide y entropía promedio de cada cluster
// //     std::vector<geometry_msgs::msg::Point> safe_centroids;
// //     std::vector<double> safe_entropies;
// //     for (const auto &cluster : clusters) {
// //       geometry_msgs::msg::Point centroid = computeCentroidFromPairs(cluster);
// //       double sum_entropy = 0.0;
// //       for (const auto &pair : cluster) {
// //         sum_entropy += pair.second;
// //       }
// //       double avg_entropy = sum_entropy / cluster.size();
// //       // Filtrar el centroide si está muy cerca de algún obstáculo
// //       bool safe = true;
// //       if (occupied_nodes_ != nullptr) {
// //         for (const auto &obs_pose : occupied_nodes_->poses) {
// //           if (euclideanDistance(centroid, obs_pose.position) < SAFE_DISTANCE_THRESHOLD) {
// //             safe = false;
// //             break;
// //           }
// //         }
// //       }
// //       if (safe) {
// //         safe_centroids.push_back(centroid);
// //         safe_entropies.push_back(avg_entropy);
// //       }
// //     }

// //     // Publicar safe frontier points como PoseArray
// //     geometry_msgs::msg::PoseArray safe_frontier_poses;
// //     safe_frontier_poses.header.stamp = this->now();
// //     safe_frontier_poses.header.frame_id = "map";
// //     for (const auto &pt : safe_centroids) {
// //       geometry_msgs::msg::Pose pose;
// //       pose.position = pt;
// //       pose.orientation.w = 1.0;
// //       safe_frontier_poses.poses.push_back(pose);
// //     }
// //     safe_frontier_pub_->publish(safe_frontier_poses);

// //     // Publicar marcador para safe frontier points (usando un publicador separado)
// //     visualization_msgs::msg::Marker safe_marker;
// //     safe_marker.header.frame_id = "map";
// //     safe_marker.header.stamp = this->now();
// //     safe_marker.ns = "safe_frontier";
// //     safe_marker.id = 1;
// //     safe_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
// //     safe_marker.action = visualization_msgs::msg::Marker::ADD;
// //     safe_marker.scale.x = 0.2;
// //     safe_marker.scale.y = 0.2;
// //     safe_marker.scale.z = 0.2;
// //     safe_marker.color.r = 0.0;
// //     safe_marker.color.g = 1.0;
// //     safe_marker.color.b = 0.0;
// //     safe_marker.color.a = 1.0;
// //     for (const auto &pt : safe_centroids) {
// //       safe_marker.points.push_back(pt);
// //     }
// //     safe_marker_pub_->publish(safe_marker);

// //     // Publicar las entropías de los safe frontier points
// //     std_msgs::msg::Float64MultiArray safe_entropy_array;
// //     safe_entropy_array.data = safe_entropies;
// //     safe_frontier_entropy_pub_->publish(safe_entropy_array);

// //     // Calcular y publicar la entropía total de los safe frontier points (promedio)
// //     double total_safe_entropy = 0.0;
// //     if (!safe_entropies.empty()) {
// //       for (double e : safe_entropies) {
// //         total_safe_entropy += e;
// //       }
// //       total_safe_entropy /= safe_entropies.size();
// //     }
// //     std_msgs::msg::Float64 total_safe_entropy_msg;
// //     total_safe_entropy_msg.data = total_safe_entropy;
// //     total_safe_entropy_pub_->publish(total_safe_entropy_msg);

// //     RCLCPP_INFO(this->get_logger(), "Se publicaron %zu centroides seguros de %zu puntos frontera.",
// //                 safe_centroids.size(), frontier_points_with_entropy.size());
// //   }

// //   // Función para calcular el centroide de un vector de pares (punto, entropía)
// //   geometry_msgs::msg::Point computeCentroidFromPairs(const std::vector<std::pair<geometry_msgs::msg::Point, double>> &pairs) {
// //     geometry_msgs::msg::Point centroid;
// //     double sum_x = 0.0, sum_y = 0.0;
// //     for (const auto &pair : pairs) {
// //       sum_x += pair.first.x;
// //       sum_y += pair.first.y;
// //     }
// //     centroid.x = sum_x / pairs.size();
// //     centroid.y = sum_y / pairs.size();
// //     centroid.z = 0.0;
// //     return centroid;
// //   }

// //   // Función para calcular el centroide de un vector de puntos
// //   geometry_msgs::msg::Point computeCentroid(const std::vector<geometry_msgs::msg::Point> &points) {
// //     geometry_msgs::msg::Point centroid;
// //     double sum_x = 0.0, sum_y = 0.0;
// //     for (const auto &pt : points) {
// //       sum_x += pt.x;
// //       sum_y += pt.y;
// //     }
// //     centroid.x = sum_x / points.size();
// //     centroid.y = sum_y / points.size();
// //     centroid.z = 0.0;
// //     return centroid;
// //   }
// // };

// // int main(int argc, char ** argv)
// // {
// //   rclcpp::init(argc, argv);
// //   auto node = std::make_shared<FrontierBoundaryNode>();
// //   rclcpp::spin(node);
// //   rclcpp::shutdown();
// //   return 0;
// // }



// // enhanced_frontier_node.cpp
// //
// // Genera puntos de exploración (frontier + lattice + Voronoi) a partir
// // de un OccupancyGrid.  Está pensado para usarse junto a tu planner,
// // que seleccionará la sub-meta más adecuada.
// //
// // Requiere:  OpenCV 4 (modules core, imgproc, ximgproc)  +  tf2_ros
// // Compilar con C++17.
// //
// // Autor: ChatGPT · 27-may-2025
// // ----------------------------------------------------------------------

// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <geometry_msgs/msg/pose_array.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <visualization_msgs/msg/marker.hpp>

// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <opencv2/core.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/ximgproc.hpp>

// #include <unordered_set>
// #include <cmath>
// #include <memory>
// #include <string>
// #include <vector>

// using std::placeholders::_1;

// namespace {

// double euclidean(const geometry_msgs::msg::Point &a,
//                  const geometry_msgs::msg::Point &b)
// {
//   return std::hypot(a.x - b.x, a.y - b.y);
// }

// /* Para hash de celda en std::unordered_set */
// struct Key
// {
//   int x, y;
//   bool operator==(const Key &o) const noexcept { return x == o.x && y == o.y; }
// };
// struct KeyHash
// {
//   std::size_t operator()(const Key &k) const noexcept
//   { return static_cast<std::size_t>(k.x) * 73856093u ^ static_cast<std::size_t>(k.y) * 19349663u; }
// };

// } // anon

// /* ===================================================================== */
// class EnhancedFrontierNode : public rclcpp::Node
// {
// public:
//   EnhancedFrontierNode()
//   : Node("enhanced_frontier_node"),
//     tf_buffer_(get_clock()),
//     tf_listener_(tf_buffer_)
//   {
//     /* -------- parámetros ----------------------------- */
//     declare_parameter("cluster_distance",         2.0);   // m
//     declare_parameter("dilate_iterations",        1);     // int
//     declare_parameter("min_frontiers_for_lattice",10);    // int
//     declare_parameter("lattice_ring_step",        5.0);   // m
//     declare_parameter("lattice_angle_step_deg",  30.0);   // deg
//     declare_parameter("sensor_range",            40.0);   // m
//     declare_parameter("voronoi_enabled",         true);   // bool
//     declare_parameter("voronoi_factor",           0.33);  // dist_thresh = factor*maxDT
//     declare_parameter("robot_base_frame",        "base_link");

//     cluster_distance_     = get_parameter("cluster_distance").as_double();
//     dilate_iters_         = get_parameter("dilate_iterations").as_int();
//     min_frontiers_        = get_parameter("min_frontiers_for_lattice").as_int();
//     ring_step_            = get_parameter("lattice_ring_step").as_double();
//     angle_step_deg_       = get_parameter("lattice_angle_step_deg").as_double();
//     sensor_range_         = get_parameter("sensor_range").as_double();
//     voronoi_enabled_      = get_parameter("voronoi_enabled").as_bool();
//     voronoi_factor_       = get_parameter("voronoi_factor").as_double();
//     base_frame_           = get_parameter("robot_base_frame").as_string();

//     /* -------- topics --------------------------------- */
//     sub_grid_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
//         "/occupancy_grid", 10,
//         std::bind(&EnhancedFrontierNode::gridCb, this, _1));

//     pub_points_ = create_publisher<geometry_msgs::msg::PoseArray>(
//         "/exploration_points", 10);

//     // markers (opcionales; pon queue 1 para no saturar)
//     pub_marker_frontier_ = create_publisher<visualization_msgs::msg::Marker>(
//         "/exploration/frontier_marker",  1);
//     pub_marker_lattice_  = create_publisher<visualization_msgs::msg::Marker>(
//         "/exploration/lattice_marker",   1);
//     pub_marker_voronoi_  = create_publisher<visualization_msgs::msg::Marker>(
//         "/exploration/voronoi_marker",   1);

//     RCLCPP_INFO(get_logger(), "Nodo enhanced_frontier_node listo.");
//   }

// private:
//   /* ------------------------------------------------------------------ */
//   void gridCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
//   {
//     const int W = msg->info.width;
//     const int H = msg->info.height;
//     const double res = msg->info.resolution;
//     const double ox  = msg->info.origin.position.x;
//     const double oy  = msg->info.origin.position.y;

//     // ---- 0) generar máscaras OpenCV ---------------------------------
//     cv::Mat mask_free  (H, W, CV_8UC1, cv::Scalar(0));
//     cv::Mat mask_unknown(H, W, CV_8UC1, cv::Scalar(0));

//     for (int y = 0; y < H; ++y) {
//       const int8_t *row = &msg->data[y*W];
//       uchar *f = mask_free.ptr<uchar>(y);
//       uchar *u = mask_unknown.ptr<uchar>(y);
//       for (int x = 0; x < W; ++x) {
//         if      (row[x] == 0)  f[x] = 255;
//         else if (row[x] == -1) u[x] = 255;
//       }
//     }

//     // ---- 1) frontera UNKNOWN-touch-FREE -----------------------------
//     cv::Mat unknown_erode;
//     cv::erode(mask_unknown, unknown_erode, cv::Mat());     // vecinos de Unknown
//     cv::Mat frontier_mask;
//     cv::bitwise_and(unknown_erode, mask_free, frontier_mask);

//     if (dilate_iters_ > 0)
//       cv::dilate(frontier_mask, frontier_mask,
//                  cv::Mat(), {-1,-1}, dilate_iters_);

//     // ---------- vector de puntos frontera ---------------------------
//     std::vector<geometry_msgs::msg::Point> frontier_pts;
//     frontier_pts.reserve(1000);
//     for (int y=0; y<H; ++y) {
//       const uchar *row = frontier_mask.ptr<uchar>(y);
//       for (int x=0; x<W; ++x) if (row[x]) {
//         geometry_msgs::msg::Point p;
//         p.x = ox + (x + 0.5)*res;
//         p.y = oy + (y + 0.5)*res;
//         p.z = 0.0;
//         frontier_pts.push_back(p);
//       }
//     }

//     // ---- 2) si < N fronteras  → generar lattice ---------------------
//     std::vector<geometry_msgs::msg::Point> lattice_pts;
//     if (static_cast<int>(frontier_pts.size()) < min_frontiers_) {
//       if (getRobotPose(last_robot_x_, last_robot_y_)) {
//         generateLatticePts(msg, last_robot_x_, last_robot_y_, lattice_pts);
//       }
//     }

//     // ---- 3) Voronoi -------------------------------------------------
//     std::vector<geometry_msgs::msg::Point> voro_pts;
//     if (voronoi_enabled_) {
//       generateVoronoiPts(mask_free, res, ox, oy, voro_pts);
//     }

//     // ---- 4) unifica listas -----------------------------------------
//     frontier_pts.insert(frontier_pts.end(),
//                         lattice_pts.begin(), lattice_pts.end());
//     frontier_pts.insert(frontier_pts.end(),
//                         voro_pts.begin(),    voro_pts.end());

//     if (frontier_pts.empty()) {
//       RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
//                            "Sin puntos de exploración en este ciclo");
//       return;
//     }

//     // ---- 5) clustering ---------------------------------------------
//     std::vector<geometry_msgs::msg::Point> centroids;
//     clusterPoints(frontier_pts, centroids);

//     // ---- 6) publicar PoseArray -------------------------------------
//     geometry_msgs::msg::PoseArray pa;
//     pa.header = msg->header;
//     pa.header.stamp = now();
//     for (const auto &pt : centroids) {
//       geometry_msgs::msg::Pose pose;
//       pose.position = pt;
//       pose.orientation.w = 1.0;
//       pa.poses.push_back(pose);
//     }
//     pub_points_->publish(pa);

//     // ---- 7) markers (solo si hay suscriptores) ---------------------
//     if (pub_marker_frontier_->get_subscription_count())
//       publishMarker(frontier_pts, msg->header.frame_id,
//                     0, 1.0,0.0,0.0, pub_marker_frontier_);   // rojo

//     if (!lattice_pts.empty() &&
//         pub_marker_lattice_->get_subscription_count())
//       publishMarker(lattice_pts, msg->header.frame_id,
//                     1, 0.0,1.0,0.0, pub_marker_lattice_);    // verde

//     if (!voro_pts.empty() &&
//         pub_marker_voronoi_->get_subscription_count())
//       publishMarker(voro_pts, msg->header.frame_id,
//                     2, 0.0,0.0,1.0, pub_marker_voronoi_);    // azul
//   }

//   /* ================================================================ */
//   /* --------------------------- helpers ---------------------------- */

//   bool getRobotPose(double &x, double &y)
//   {
//     try {
//       geometry_msgs::msg::TransformStamped tf =
//         tf_buffer_.lookupTransform("map", base_frame_,
//                                    tf2::TimePointZero);
//       x = tf.transform.translation.x;
//       y = tf.transform.translation.y;
//       return true;
//     } catch (const tf2::TransformException &e) {
//       RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
//                            "TF lookup failed: %s", e.what());
//       return false;
//     }
//   }

//   /* ---------- lattice -------------------------------------------- */
//   void generateLatticePts(const nav_msgs::msg::OccupancyGrid::SharedPtr &msg,
//                           double rx, double ry,
//                           std::vector<geometry_msgs::msg::Point> &out)
//   {
//     const int W = msg->info.width;
//     const int H = msg->info.height;
//     const double res = msg->info.resolution;
//     const double ox  = msg->info.origin.position.x;
//     const double oy  = msg->info.origin.position.y;

//     auto worldToMap = [&](double wx, double wy, int &mx, int &my)->bool {
//       mx = static_cast<int>((wx - ox)/res);
//       my = static_cast<int>((wy - oy)/res);
//       return mx>=0 && mx<W && my>=0 && my<H;
//     };

//     for (double r = ring_step_; r <= sensor_range_; r += ring_step_) {
//       for (double ang = 0; ang < 360.0; ang += angle_step_deg_) {
//         const double ca = std::cos(ang*M_PI/180.0);
//         const double sa = std::sin(ang*M_PI/180.0);
//         double wx = rx;
//         double wy = ry;
//         geometry_msgs::msg::Point last_free;
//         bool found_unknown = false;

//         for (double d = 0.0; d <= r; d += res) {
//           wx = rx + d*ca;
//           wy = ry + d*sa;
//           int mx,my;
//           if (!worldToMap(wx,wy,mx,my)) break;
//           int idx = my*W + mx;
//           int8_t v = msg->data[idx];
//           if (v == 100) break;                // obstáculo → aborta rayo
//           if (v == -1) {                      // unknown → último FREE = sub-goal
//             found_unknown = true;
//             break;
//           }
//           // v == 0
//           last_free.x = wx; last_free.y = wy; last_free.z = 0.0;
//         }
//         if (found_unknown) out.push_back(last_free);
//       }
//     }
//   }

//   /* ---------- Voronoi -------------------------------------------- */
//   void generateVoronoiPts(const cv::Mat &freeMask,
//                           double res, double ox, double oy,
//                           std::vector<geometry_msgs::msg::Point> &out)
//   {
//     cv::Mat dist;
//     cv::distanceTransform(freeMask, dist, cv::DIST_L2, 5);
//     double maxv;
//     cv::minMaxLoc(dist, nullptr, &maxv);
//     double thresh = voronoi_factor_ * maxv;

//     // se extraen "crestas": píxeles con DT >= thresh y locales máximos
//     for (int y = 1; y < freeMask.rows-1; ++y) {
//       const float *row = dist.ptr<float>(y);
//       for (int x = 1; x < freeMask.cols-1; ++x) {
//         float v = row[x];
//         if (v < thresh) continue;
//         bool is_max = true;
//         for (int dy=-1; dy<=1 && is_max; ++dy)
//           for (int dx=-1; dx<=1 && is_max; ++dx)
//             if (dx||dy) is_max &= v >= dist.at<float>(y+dy,x+dx);
//         if (!is_max) continue;

//         geometry_msgs::msg::Point p;
//         p.x = ox + (x+0.5)*res;
//         p.y = oy + (y+0.5)*res;
//         p.z = 0.0;
//         out.push_back(p);
//       }
//     }
//   }

//   /* ---------- clustering ----------------------------------------- */
//   void clusterPoints(const std::vector<geometry_msgs::msg::Point> &in,
//                      std::vector<geometry_msgs::msg::Point> &centroids)
//   {
//     const double thresh2 = cluster_distance_ * cluster_distance_;
//     std::vector<bool> used(in.size(), false);

//     for (size_t i = 0; i < in.size(); ++i) if (!used[i]) {
//       double sumx = 0, sumy = 0; int cnt = 0;
//       for (size_t j = i; j < in.size(); ++j) if (!used[j]) {
//         double dx = in[j].x - in[i].x;
//         double dy = in[j].y - in[i].y;
//         if (dx*dx + dy*dy <= thresh2) {
//           used[j] = true;
//           sumx += in[j].x;
//           sumy += in[j].y;
//           ++cnt;
//         }
//       }
//       geometry_msgs::msg::Point c;
//       c.x = sumx / cnt;
//       c.y = sumy / cnt;
//       c.z = 0.0;
//       centroids.push_back(c);
//     }
//   }

//   /* ---------- marker --------------------------------------------- */
//   void publishMarker(const std::vector<geometry_msgs::msg::Point> &pts,
//                      const std::string &frame, int id,
//                      double r,double g,double b,
//                      const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub)
//   {
//     visualization_msgs::msg::Marker m;
//     m.header.frame_id = frame;
//     m.header.stamp    = now();
//     m.ns   = "exploration_pts";
//     m.id   = id;
//     m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
//     m.action = visualization_msgs::msg::Marker::ADD;
//     m.scale.x = m.scale.y = m.scale.z = 0.3;
//     m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0;
//     m.points  = pts;
//     pub->publish(m);
//   }

//   /* ---------------------------------------------------------------- */
//   /* variables miembro */
//   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_grid_;
//   rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr   pub_points_;
//   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_frontier_;
//   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_lattice_;
//   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_voronoi_;

//   /* tf */
//   tf2_ros::Buffer tf_buffer_;
//   tf2_ros::TransformListener tf_listener_;

//   /* parámetros */
//   double cluster_distance_;
//   int    dilate_iters_;
//   int    min_frontiers_;
//   double ring_step_;
//   double angle_step_deg_;
//   double sensor_range_;
//   bool   voronoi_enabled_;
//   double voronoi_factor_;
//   std::string base_frame_;

//   /* estado */
//   double last_robot_x_{0.0}, last_robot_y_{0.0};
// };

// /* ===================================================================== */
// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<EnhancedFrontierNode>());
//   rclcpp::shutdown();
//   return 0;
// }


// enhanced_frontier_node.cpp
//
// Genera puntos de exploración combinando:
//
//   1.  Fronteras FREE ↔ UNKNOWN  (rojo)
//   2.  Lattice polar             (verde)   – si hay pocas fronteras
//   3.  Vértices Voronoi          (azul)    – opcional
//
// Publica todos los candidatos, ya clusterizados, en /exploration_points.
//
// Autor: ChatGPT · 27-may-2025
// --------------------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <unordered_set>

using std::placeholders::_1;

namespace {          /* utilidades internas */

double euclidean(const geometry_msgs::msg::Point &a,
                 const geometry_msgs::msg::Point &b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

} // anon namespace

/* ======================================================================= */
class EnhancedFrontierNode : public rclcpp::Node
{
public:
  EnhancedFrontierNode()
  : Node("enhanced_frontier_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    /* -------------------- parámetros -------------------- */
    declare_parameter("cluster_distance",          2.0);   // m
    declare_parameter("dilate_iterations",         1);     // celdas extras a la frontera
    declare_parameter("min_frontiers_for_lattice",10);     // umbral
    declare_parameter("lattice_ring_step",         5.0);   // m
    declare_parameter("lattice_angle_step_deg",   30.0);   // deg
    declare_parameter("sensor_range",             40.0);   // m
    declare_parameter("voronoi_enabled",          true);
    declare_parameter("voronoi_factor",            0.33);  // crest threshold
    declare_parameter("robot_base_frame",         "base_link");
    declare_parameter("unknown_threshold",         10);    // ≤ este valor ⇒ unknown

    cluster_distance_  = get_parameter("cluster_distance").as_double();
    dilate_iters_      = get_parameter("dilate_iterations").as_int();
    min_frontiers_     = get_parameter("min_frontiers_for_lattice").as_int();
    ring_step_         = get_parameter("lattice_ring_step").as_double();
    angle_step_deg_    = get_parameter("lattice_angle_step_deg").as_double();
    sensor_range_      = get_parameter("sensor_range").as_double();
    voronoi_enabled_   = get_parameter("voronoi_enabled").as_bool();
    voronoi_factor_    = get_parameter("voronoi_factor").as_double();
    base_frame_        = get_parameter("robot_base_frame").as_string();
    unknown_thr_       = get_parameter("unknown_threshold").as_int();

    /* ------------------- topics ------------------------ */
    sub_grid_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_grid", 10,
        std::bind(&EnhancedFrontierNode::gridCb, this, _1));

    pub_points_ = create_publisher<geometry_msgs::msg::PoseArray>(
        "/exploration_points", 10);

    pub_marker_frontier_ = create_publisher<visualization_msgs::msg::Marker>(
        "/exploration/frontier_marker",  1);
    pub_marker_lattice_  = create_publisher<visualization_msgs::msg::Marker>(
        "/exploration/lattice_marker",   1);
    pub_marker_voronoi_  = create_publisher<visualization_msgs::msg::Marker>(
        "/exploration/voronoi_marker",   1);

    RCLCPP_INFO(get_logger(), "enhanced_frontier_node iniciado.");
  }

private:
  /* -------------------------------------------------------------------- */
  void gridCb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const int    W   = msg->info.width;
    const int    H   = msg->info.height;
    const double res = msg->info.resolution;
    const double ox  = msg->info.origin.position.x;
    const double oy  = msg->info.origin.position.y;

    /* 0) Construir máscaras libre / unknown */
    cv::Mat m_free(H, W, CV_8UC1, cv::Scalar(0));
    cv::Mat m_unk (H, W, CV_8UC1, cv::Scalar(0));

    for (int y = 0; y < H; ++y) {
      const int8_t *row = &msg->data[y*W];
      uchar *pf = m_free.ptr<uchar>(y);
      uchar *pu = m_unk .ptr<uchar>(y);
      for (int x = 0; x < W; ++x) {
        if (row[x] == 0)               pf[x] = 255;
        else if (row[x] == -1 ||
                 row[x] <= unknown_thr_) pu[x] = 255;
      }
    }

    /* 1) Frontera = FREE ∧ dilate(UNKNOWN) */
    cv::Mat unk_dilate;
    cv::dilate(m_unk, unk_dilate, cv::Mat(), {-1,-1}, 1);  // 1 celda
    cv::Mat m_frontier;
    cv::bitwise_and(unk_dilate, m_free, m_frontier);

    if (dilate_iters_ > 0)
      cv::dilate(m_frontier, m_frontier,
                 cv::Mat(), {-1,-1}, dilate_iters_);

    /* 2) Pasar máscara a vector de puntos */
    std::vector<geometry_msgs::msg::Point> frontier_pts;
    for (int y=0; y<H; ++y) {
      const uchar *row = m_frontier.ptr<uchar>(y);
      for (int x=0; x<W; ++x) if (row[x]) {
        geometry_msgs::msg::Point p;
        p.x = ox + (x + 0.5)*res;
        p.y = oy + (y + 0.5)*res;
        p.z = 0.0;
        frontier_pts.push_back(p);
      }
    }

    /* 3) Lattice si hay pocas fronteras */
    std::vector<geometry_msgs::msg::Point> lattice_pts;
    if (static_cast<int>(frontier_pts.size()) < min_frontiers_) {
      if (getRobotPose(last_robot_x_, last_robot_y_))
        generateLatticePts(msg, last_robot_x_, last_robot_y_, lattice_pts);
    }

    /* 4) Voronoi */
    std::vector<geometry_msgs::msg::Point> voro_pts;
    if (voronoi_enabled_)
      generateVoronoiPts(m_free, res, ox, oy, voro_pts);

    /* 5) Unir listas */
    frontier_pts.insert(frontier_pts.end(),
                        lattice_pts.begin(), lattice_pts.end());
    frontier_pts.insert(frontier_pts.end(),
                        voro_pts.begin(),    voro_pts.end());

    if (frontier_pts.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Sin puntos de exploración este ciclo");
      return;
    }

    /* 6) Clustering */
    std::vector<geometry_msgs::msg::Point> centroids;
    clusterPoints(frontier_pts, centroids);

    /* 7) Publicar PoseArray final */
    geometry_msgs::msg::PoseArray pa;
    pa.header = msg->header;
    pa.header.stamp = now();
    for (const auto &pt : centroids) {
      geometry_msgs::msg::Pose pose;
      pose.position = pt;
      pose.orientation.w = 1.0;
      pa.poses.push_back(pose);
    }
    pub_points_->publish(pa);

    /* 8) Markers para RViz (opcionales) */
    if (pub_marker_frontier_->get_subscription_count())
      publishMarker(frontier_pts, msg->header.frame_id,
                    0, 1.0,0.0,0.0, pub_marker_frontier_);
    if (!lattice_pts.empty() &&
        pub_marker_lattice_->get_subscription_count())
      publishMarker(lattice_pts, msg->header.frame_id,
                    1, 0.0,1.0,0.0, pub_marker_lattice_);
    if (!voro_pts.empty() &&
        pub_marker_voronoi_->get_subscription_count())
      publishMarker(voro_pts, msg->header.frame_id,
                    2, 0.0,0.0,1.0, pub_marker_voronoi_);
  }

  /* ========================= helpers ============================== */

  /* ------ obtener pose del robot en frame map --------------------- */
  bool getRobotPose(double &x, double &y)
  {
    try {
      geometry_msgs::msg::TransformStamped tf =
        tf_buffer_.lookupTransform("map", base_frame_,
                                   tf2::TimePointZero);
      x = tf.transform.translation.x;
      y = tf.transform.translation.y;
      return true;
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "TF lookup failed: %s", e.what());
      return false;
    }
  }

  /* ------ lattice polar ------------------------------------------ */
  void generateLatticePts(const nav_msgs::msg::OccupancyGrid::SharedPtr &msg,
                          double rx, double ry,
                          std::vector<geometry_msgs::msg::Point> &out)
  {
    const int    W   = msg->info.width;
    const int    H   = msg->info.height;
    const double res = msg->info.resolution;
    const double ox  = msg->info.origin.position.x;
    const double oy  = msg->info.origin.position.y;

    auto w2m = [&](double wx,double wy,int &mx,int &my)->bool{
      mx = static_cast<int>((wx-ox)/res);
      my = static_cast<int>((wy-oy)/res);
      return mx>=0 && mx<W && my>=0 && my<H;
    };

    for (double r=ring_step_; r<=sensor_range_; r+=ring_step_) {
      for (double ang=0; ang<360.0; ang+=angle_step_deg_) {
        const double ca = std::cos(ang*M_PI/180.0);
        const double sa = std::sin(ang*M_PI/180.0);
        double wx=rx, wy=ry;
        geometry_msgs::msg::Point last_free;
        bool found_unknown=false;

        for (double d=0; d<=r; d+=res) {
          wx = rx + d*ca;
          wy = ry + d*sa;
          int mx,my;
          if (!w2m(wx,wy,mx,my)) break;
          int8_t v = msg->data[my*W+mx];
          if (v == 100) break;          // obstáculo
          if (v == -1 || v<=unknown_thr_) { found_unknown=true; break; }
          last_free.x=wx; last_free.y=wy; last_free.z=0;
        }
        if (found_unknown) out.push_back(last_free);
      }
    }
  }

  /* ------ Voronoi crestas ---------------------------------------- */
  void generateVoronoiPts(const cv::Mat &freeMask,
                          double res,double ox,double oy,
                          std::vector<geometry_msgs::msg::Point> &out)
  {
    cv::Mat dist;
    cv::distanceTransform(freeMask, dist, cv::DIST_L2, 5);
    double maxv; cv::minMaxLoc(dist,nullptr,&maxv);
    double thresh = voronoi_factor_ * maxv;

    for (int y=1; y<freeMask.rows-1; ++y){
      const float *row = dist.ptr<float>(y);
      for (int x=1; x<freeMask.cols-1; ++x){
        float v=row[x]; if (v<thresh) continue;
        bool is_max=true;
        for(int dy=-1;dy<=1 && is_max;++dy)
          for(int dx=-1;dx<=1 && is_max;++dx)
            if(dx||dy) is_max&=v>=dist.at<float>(y+dy,x+dx);
        if(!is_max) continue;

        geometry_msgs::msg::Point p;
        p.x = ox + (x+0.5)*res;
        p.y = oy + (y+0.5)*res;
        p.z = 0;
        out.push_back(p);
      }
    }
  }

  /* ------ clustering simple -------------------------------------- */
  void clusterPoints(const std::vector<geometry_msgs::msg::Point> &in,
                     std::vector<geometry_msgs::msg::Point> &out)
  {
    const double th2 = cluster_distance_*cluster_distance_;
    std::vector<char> used(in.size(),0);

    for (size_t i=0;i<in.size();++i) if(!used[i]){
      double sx=0,sy=0; int n=0;
      for(size_t j=i;j<in.size();++j) if(!used[j]){
        double dx=in[j].x-in[i].x, dy=in[j].y-in[i].y;
        if(dx*dx+dy*dy<=th2){
          used[j]=1; sx+=in[j].x; sy+=in[j].y; ++n;
        }
      }
      geometry_msgs::msg::Point c;
      c.x=sx/n; c.y=sy/n; c.z=0;
      out.push_back(c);
    }
  }

  /* ------ marker gen --------------------------------------------- */
  void publishMarker(const std::vector<geometry_msgs::msg::Point> &pts,
                     const std::string &frame,int id,
                     double r,double g,double b,
                     const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id=frame;
    m.header.stamp   =now();
    m.ns  ="exploration_pts";
    m.id  =id;
    m.type=visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action=visualization_msgs::msg::Marker::ADD;
    m.scale.x=m.scale.y=m.scale.z=0.3;
    m.color.r=r; m.color.g=g; m.color.b=b; m.color.a=1;
    m.points=pts;
    pub->publish(m);
  }

  /* -------------------- miembros ---------------------------------- */
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_grid_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr   pub_points_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_frontier_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_lattice_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_voronoi_;

  tf2_ros::Buffer             tf_buffer_;
  tf2_ros::TransformListener  tf_listener_;

  /* parámetros */
  double cluster_distance_;
  int    dilate_iters_;
  int    min_frontiers_;
  double ring_step_;
  double angle_step_deg_;
  double sensor_range_;
  bool   voronoi_enabled_;
  double voronoi_factor_;
  std::string base_frame_;
  int    unknown_thr_;

  /* estado */
  double last_robot_x_{0.0}, last_robot_y_{0.0};
};

/* ======================================================================= */
int main(int argc,char **argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<EnhancedFrontierNode>());
  rclcpp::shutdown();
  return 0;
}
