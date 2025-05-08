#ifndef PATH_MANAGER_HPP
#define PATH_MANAGER_HPP

#include <chrono>
#include <memory>
#include <string>
#include <fstream>
#include <iomanip> 
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <iostream>
#include <filesystem>
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "path_manager_interfaces/srv/read_path_from_file.hpp"
#include "path_manager_interfaces/srv/write_path_to_file.hpp"
#include "path_manager_interfaces/srv/return_robot_path.hpp"
#include "path_manager_interfaces/srv/get_ll_path.hpp"
#include "path_manager_interfaces/srv/get_fix_frame_path.hpp"
#include "path_manager_interfaces/srv/plan_path.hpp"
#include "path_manager_interfaces/srv/robot_path.hpp"
#include "path_manager_interfaces/srv/assist_emergency.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
using namespace std::chrono_literals;
namespace DEMAIAS
{

std::pair <nav_msgs::msg::Path, nav_msgs::msg::Path> planner(char* DEM, char* mapping, char* LTL);

}
class PathManager : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit PathManager(const std::string & node_name, bool intra_process_comms = false);

  void publish();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

  void ll_path_callback(const std::shared_ptr<nav_msgs::msg::Path> msg);

  void readPath(const std::shared_ptr<path_manager_interfaces::srv::ReadPathFromFile::Request> req,
                const std::shared_ptr<path_manager_interfaces::srv::ReadPathFromFile::Response> res);

  std::string getPathFolder();

  void planPath(const std::shared_ptr<path_manager_interfaces::srv::PlanPath::Request> req,
                           std::shared_ptr<path_manager_interfaces::srv::PlanPath::Response> res);

  void writePath(const std::shared_ptr<path_manager_interfaces::srv::WritePathToFile::Request> req,
                 const std::shared_ptr<path_manager_interfaces::srv::WritePathToFile::Response> res);

  void getLatLonPath(const std::shared_ptr<path_manager_interfaces::srv::GetLLPath::Request> req,
                     const std::shared_ptr<path_manager_interfaces::srv::GetLLPath::Response> res);

  void getFFPath(const std::shared_ptr<path_manager_interfaces::srv::GetFixFramePath::Request> req,
                 const std::shared_ptr<path_manager_interfaces::srv::GetFixFramePath::Response> res);

  void returnPath(const std::shared_ptr<path_manager_interfaces::srv::ReturnRobotPath::Request> req,
                  const std::shared_ptr<path_manager_interfaces::srv::ReturnRobotPath::Response> res);

  void latLonToUTM(double latitude, double longitude, double& utmNorthing, double& utmEasting, int& zone, bool& isNorth);

  void receivePath(const std::shared_ptr<path_manager_interfaces::srv::RobotPath::Request> req,
                           std::shared_ptr<path_manager_interfaces::srv::RobotPath::Response> res);

  void assistEmergency(const std::shared_ptr<path_manager_interfaces::srv::AssistEmergency::Request> req,
                           std::shared_ptr<path_manager_interfaces::srv::AssistEmergency::Response> res);

  geometry_msgs::msg::PoseStamped ConvertToECEF(double latitude, double longitude, double altitude);

  void ConvertUTMToLatLon(double utmNorthing, double utmEasting, int zone, bool north, double& lat, double& lon);


private:
  std::shared_ptr<rclcpp::TimerBase> timer_;
  rclcpp::Service<path_manager_interfaces::srv::WritePathToFile>::SharedPtr writePathServ;
  rclcpp::Service<path_manager_interfaces::srv::ReadPathFromFile>::SharedPtr readPathServ;
  rclcpp::Service<path_manager_interfaces::srv::ReturnRobotPath>::SharedPtr getRobotPath;
  rclcpp::Service<path_manager_interfaces::srv::GetLLPath>::SharedPtr getLLPath;
  rclcpp::Service<path_manager_interfaces::srv::GetFixFramePath>::SharedPtr getFixFramePath;
  rclcpp::Service<path_manager_interfaces::srv::PlanPath>::SharedPtr plannerPathServ;
  rclcpp::Service<path_manager_interfaces::srv::RobotPath>::SharedPtr receivePathserv;
    rclcpp::Service<path_manager_interfaces::srv::AssistEmergency>::SharedPtr assistEmersrv;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Path>> sub_ll_path;
  nav_msgs::msg::Path robot_path;
  nav_msgs::msg::Path ll_path;
  std::string fixed_frame, localization_method, global_param;
  // WGS-84 Earth model constants
  static constexpr double a = 6378137.0;  // Semi-major axis
  static constexpr double e_squared = 0.00669437999014;  // Eccentricity squared
};

#endif  // PATH_MANAGER_HPP
