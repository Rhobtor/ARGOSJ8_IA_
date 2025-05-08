#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <lifecycle_msgs/msg/transition.hpp>
#include "rclcpp/rclcpp.hpp"
#include <lifecycle_msgs/msg/state.hpp>
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "rcutils/logging_macros.h"

#include "argj801_sensors/SensorBuilder.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "argj801_sensors_msgs/msg/motor_telemetry.hpp"
#include "argj801_sensors_msgs/msg/odometer.hpp"
#include "argj801_sensors/Sensor.hpp"

#include "visitor/SensorDataVisitor.hpp"

#include "diagnostic_updater/update_functions.hpp"

#include "argj801_sensors/argj801_sensors_interface.hpp"

using namespace std::chrono_literals;

class Argj801SensorsNode : public rclcpp_lifecycle::LifecycleNode
{
private:

  std::unique_ptr<diagnostic_updater::Updater> diagnostic_; //! Object that allows the IMU diagnostic
  double lidar_desired_freq_;
  std::unique_ptr<diagnostic_updater::FrequencyStatus> fast_freq_diag_; 
  double drive_line_desired_freq_;
  std::unique_ptr<diagnostic_updater::FrequencyStatus> slow_freq_diag_; 
  double camera_desired_freq_;

  std::shared_ptr<Builder::SensorBuilder> builder;
  std::shared_ptr<Sensor> sensor;

  std::shared_ptr<Visitor::Visitor> visitor;

  std::string lcm_config_file;
  std::string robot_frame;
  std::string velodyne_frame;
  std::string sick_frame;
  bool velodyne;
  bool sick;
  bool left_motor;
  bool right_motor;
  bool odometer;
  bool twist;
  bool camera;
  std::string camera_url;
  std::string camera_frame;
  std::string camera_topic;
  bool resize_image;
  int compression_ratio;


  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> velodyne_publisher;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> sick_publisher;
  std::shared_ptr<rclcpp::Publisher<argj801_sensors_msgs::msg::MotorTelemetry>> left_motor_publisher;
  std::shared_ptr<rclcpp::Publisher<argj801_sensors_msgs::msg::MotorTelemetry>> right_motor_publisher;
  std::shared_ptr<rclcpp::Publisher<argj801_sensors_msgs::msg::Odometer>> odometer_publisher;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>> twist_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr camera_publisher;

  rclcpp::TimerBase::SharedPtr lidarTimer; 
  rclcpp::TimerBase::SharedPtr driveLineTimer; //! Timer to capture calibration data
  rclcpp::TimerBase::SharedPtr cameraTimer; 

  rclcpp::CallbackGroup::SharedPtr group1;

  void getFastData();
  void getSlowData();
  
public:
  Argj801SensorsNode(std::shared_ptr<Builder::SensorBuilder> builder);
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &state);

};

