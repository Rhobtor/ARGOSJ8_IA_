// // Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
// // All rights reserved.
// //
// // Software License Agreement (BSD License 2.0)
// //
// // Redistribution and use in source and binary forms, with or without
// // modification, are permitted provided that the following conditions
// // are met:
// //
// //  * Redistributions of source code must retain the above copyright
// //    notice, this list of conditions and the following disclaimer.
// //  * Redistributions in binary form must reproduce the above
// //    copyright notice, this list of conditions and the following
// //    disclaimer in the documentation and/or other materials provided
// //    with the distribution.
// //  * Neither the name of the company nor the names of its
// //    contributors may be used to endorse or promote products derived
// //    from this software without specific prior written permission.
// //
// // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// // "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// // LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// // FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// // COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// // INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// // BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// // LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// // CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// // LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// // ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// // POSSIBILITY OF SUCH DAMAGE.

// /*
//  * \file  gazebo_ros_diff_drive.cpp
//  *
//  * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
//  * developed for the erratic robot (see copyright notice above). The original
//  * plugin can be found in the ROS package gazebo_erratic_plugins.
//  *
//  * \author  Piyush Khandelwal (piyushk@gmail.com)
//  *
//  * $ Id: 06/21/2013 11:23:40 AM piyushk $
//  */

// /*
//  *
//  * The support of acceleration limit was added by
//  * \author   George Todoran <todorangrg@gmail.com>
//  * \author   Markus Bader <markus.bader@tuwien.ac.at>
//  * \date 22th of May 2014
//  */

// #include <gazebo/common/Time.hh>
// #include <gazebo/physics/Joint.hh>
// #include <gazebo/physics/Link.hh>
// #include <gazebo/physics/Model.hh>
// #include <gazebo/physics/World.hh>
// #include <j8_xacro_model/j8_motors_control.hpp>
// #include <gazebo_ros/conversions/builtin_interfaces.hpp>
// #include <gazebo_ros/conversions/geometry_msgs.hpp>
// #include <gazebo_ros/node.hpp>
// #include <geometry_msgs/msg/pose2_d.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <sdf/sdf.hh>
// #include "argj801_ctl_platform_interfaces/msg/cmd_throttle_msg.hpp"




// #include <chrono>

// #ifdef NO_ERROR
// // NO_ERROR is a macro defined in Windows that's used as an enum in tf2
// #undef NO_ERROR
// #endif

// #ifdef IGN_PROFILER_ENABLE
// #include <ignition/common/Profiler.hh>
// #endif

// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>

// #include <memory>
// #include <string>
// #include <vector>

// namespace gazebo_plugins
// {
// class GazeboRosDiffDrivePrivate
// {
// public:
//   /// Indicates where the odometry info is coming from
//   enum OdomSource
//   {
//     /// Use an ancoder
//     ENCODER = 0,

//     /// Use ground truth from simulation world
//     WORLD = 1,
//   };

//   /// Indicates which wheel
//   enum
//   {
//     /// Right wheel
//     RIGHT = 0,

//     /// Left wheel
//     LEFT = 1,
//   };

//   /// Callback to be called at every simulation iteration.
//   /// \param[in] _info Updated simulation info.
//   void OnUpdate(const gazebo::common::UpdateInfo & _info);

//   /// Callback when a velocity command is received.
//   /// \param[in] _msg Twist command message.
//   void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

//   /// Update wheel velocities according to latest target velocities.
//   void UpdateWheelVelocities();

//   /// Update odometry according to encoder.
//   /// \param[in] _current_time Current simulation time
//   void UpdateOdometryEncoder(const gazebo::common::Time & _current_time);

//   /// Update odometry according to world
//   void UpdateOdometryWorld();

//   /// Publish odometry transforms
//   /// \param[in] _current_time Current simulation time
//   void PublishOdometryTf(const gazebo::common::Time & _current_time);

//   /// Publish trasforms for the wheels
//   /// \param[in] _current_time Current simulation time
//   void PublishWheelsTf(const gazebo::common::Time & _current_time);

//   /// Publish odometry messages
//   /// \param[in] _current_time Current simulation time
//   void PublishOdometryMsg(const gazebo::common::Time & _current_time);
//   //lcm_loop
//   void loop();

 

//   /// Callback when a descrite device request is received.
//   void OnThrottleMsg(const argj801_ctl_platform_interfaces::msg::CmdThrottleMsg::SharedPtr  _msg);

//   /// Callback when a vel msg is received.
//    void OnVelMsg(const geometry_msgs::msg::Twist::SharedPtr _msg);



//   // Double saturation fnc.
//   double doubleSaturation(double a, double L);

//   /// A pointer to the GazeboROS node.
//   gazebo_ros::Node::SharedPtr ros_node_;

//   /// Subscriber to command velocities
//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;



//   /// Subscriber to throttle velocities
//   rclcpp::Subscription<argj801_ctl_platform_interfaces::msg::CmdThrottleMsg>::SharedPtr throttle_speed_sub_;

//   /// Odometry publisher
//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_msgs_sub_;
//   ///Change mode service


//   //current lcm mode: 0:control, 1:serial, 2:radio, 3:follow_me
//   int lcm_mode = 0;
  
//   /// Connection to event called at every world iteration.
//   gazebo::event::ConnectionPtr update_connection_;

//   /// Distance between the wheels, in meters.
//   std::vector<double> wheel_separation_;

//   /// Diameter of wheels, in meters.
//   std::vector<double> wheel_diameter_;

//   /// Maximum wheel torque, in Nm.
//   double max_wheel_torque_;

//   /// Maximum wheel acceleration
//   double max_wheel_accel_;

//   /// Desired wheel speed.
//   std::vector<double> desired_wheel_speed_;

//   /// Speed sent to wheel.
//   std::vector<double> wheel_speed_instr_;

//   /// Pointers to wheel joints.
//   std::vector<gazebo::physics::JointPtr> joints_;

//   /// Pointer to model.
//   gazebo::physics::ModelPtr model_;

//   /// To broadcast TFs
//   std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

//   /// Protect variables accessed on callbacks.
//   std::mutex lock_;

//   /// Linear velocity in X received on command (m/s).
//   double target_x_{0.0};

//   /// Angular velocity in Z received on command (rad/s).
//   double target_rot_{0.0};

//   /// Update period in seconds.
//   double update_period_;

//   /// Last update time.
//   gazebo::common::Time last_update_time_;

//   /// Keep encoder data.
//   geometry_msgs::msg::Pose2D pose_encoder_;

//   /// Odometry frame ID
//   std::string odometry_frame_;

//   /// Last time the encoder was updated
//   gazebo::common::Time last_encoder_update_;

//   /// Either ENCODER or WORLD
//   OdomSource odom_source_;

//   /// Keep latest odometry message
//   nav_msgs::msg::Odometry odom_;

//   /// Robot base frame ID
//   std::string robot_base_frame_;

//   /// True to publish odometry messages.
//   bool publish_odom_;

//   /// True to publish wheel-to-base transforms.
//   bool publish_wheel_tf_;

//   /// True to publish odom-to-world transforms.
//   bool publish_odom_tf_;

//   /// Store number of wheel pairs
//   unsigned int num_wheel_pairs_;

//   /// Covariance in odometry
//   double covariance_[3];

//   /// is controlled mode.
//   bool is_controlled = false;

//   float right_wheels_speed = 0, left_wheels_speed = 0;
//   // throtle and steer sp
//   float throttle_sp_rads = 0, steer_sp_rads = 0;
//   //% to rad/s constants
// float throttle_to_rads = 0.254608412, steer_to_rads = 0.05787372; 
//  // max angular acceleration
//   float a_throttle_max = 1.86, a_steer_max=1.23;
//   // current wheels
//   float current_left_speed, current_right_speed;
  
//   // double dead_zone_time = 0.15;

//   // bool got_new_throttle_sp = false;
//   // bool got_new_steer_sp= false;


//   // gazebo::common::Time last_throttle_sp_time_;
//   // gazebo::common::Time last_steer_sp_time_;
// };

// GazeboRosDiffDrive::GazeboRosDiffDrive()
// : impl_(std::make_unique<GazeboRosDiffDrivePrivate>())
// {
// }

// GazeboRosDiffDrive::~GazeboRosDiffDrive()
// {
// }

// void GazeboRosDiffDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
// {
//   impl_->model_ = _model;

//   // Initialize ROS node
//   impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

//   // Get QoS profiles
//   const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

//   // Get number of wheel pairs in the model
//   impl_->num_wheel_pairs_ = static_cast<unsigned int>(_sdf->Get<int>("num_wheel_pairs", 1).first);

//   if (impl_->num_wheel_pairs_ < 1) {
//     impl_->num_wheel_pairs_ = 1;
//     RCLCPP_WARN(
//       impl_->ros_node_->get_logger(),
//       "Drive requires at least one pair of wheels. Setting [num_wheel_pairs] to 1");
//   }

//   // Dynamic properties
//   impl_->max_wheel_accel_ = _sdf->Get<double>("max_wheel_acceleration", 0.0).first;
//   impl_->max_wheel_torque_ = _sdf->Get<double>("max_wheel_torque", 5.0).first;

//   // Get max motors acceleration
//   impl_->a_throttle_max = _sdf->Get<double>("max_throttle_acceleration", 1.86).first;
//   impl_->a_steer_max = _sdf->Get<double>("max_steer_acceleration", 1.23).first;


//   // Get joints and Kinematic properties
//   std::vector<gazebo::physics::JointPtr> left_joints, right_joints;

//   for (auto left_joint_elem = _sdf->GetElement("left_joint"); left_joint_elem != nullptr;
//     left_joint_elem = left_joint_elem->GetNextElement("left_joint"))
//   {
//     auto left_joint_name = left_joint_elem->Get<std::string>();
//     auto left_joint = _model->GetJoint(left_joint_name);
//     if (!left_joint) {
//       RCLCPP_ERROR(
//         impl_->ros_node_->get_logger(),
//         "Joint [%s] not found, plugin will not work.", left_joint_name.c_str());
//       impl_->ros_node_.reset();
//       return;
//     }
//     left_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
//     left_joints.push_back(left_joint);
//   }

//   for (auto right_joint_elem = _sdf->GetElement("right_joint"); right_joint_elem != nullptr;
//     right_joint_elem = right_joint_elem->GetNextElement("right_joint"))
//   {
//     auto right_joint_name = right_joint_elem->Get<std::string>();
//     auto right_joint = _model->GetJoint(right_joint_name);
//     if (!right_joint) {
//       RCLCPP_ERROR(
//         impl_->ros_node_->get_logger(),
//         "Joint [%s] not found, plugin will not work.", right_joint_name.c_str());
//       impl_->ros_node_.reset();
//       return;
//     }
//     right_joint->SetParam("fmax", 0, impl_->max_wheel_torque_);
//     right_joints.push_back(right_joint);
//   }

//   if (left_joints.size() != right_joints.size() || left_joints.size() != impl_->num_wheel_pairs_) {
//     RCLCPP_ERROR(
//       impl_->ros_node_->get_logger(),
//       "Inconsistent number of joints specified. Plugin will not work.");
//     impl_->ros_node_.reset();
//     return;
//   }

//   unsigned int index;
//   for (index = 0; index < impl_->num_wheel_pairs_; ++index) {
//     impl_->joints_.push_back(right_joints[index]);
//     impl_->joints_.push_back(left_joints[index]);
//   }

//   index = 0;
//   impl_->wheel_separation_.assign(impl_->num_wheel_pairs_, 0.34);
//   for (auto wheel_separation = _sdf->GetElement("wheel_separation"); wheel_separation != nullptr;
//     wheel_separation = wheel_separation->GetNextElement("wheel_separation"))
//   {
//     if (index >= impl_->num_wheel_pairs_) {
//       RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheel_separation>");
//       break;
//     }
//     impl_->wheel_separation_[index] = wheel_separation->Get<double>();
//     RCLCPP_INFO(
//       impl_->ros_node_->get_logger(),
//       "Wheel pair %i separation set to [%fm]", index + 1, impl_->wheel_separation_[index]);
//     index++;
//   }

//   index = 0;
//   impl_->wheel_diameter_.assign(impl_->num_wheel_pairs_, 0.15);
//   for (auto wheel_diameter = _sdf->GetElement("wheel_diameter"); wheel_diameter != nullptr;
//     wheel_diameter = wheel_diameter->GetNextElement("wheel_diameter"))
//   {
//     if (index >= impl_->num_wheel_pairs_) {
//       RCLCPP_WARN(impl_->ros_node_->get_logger(), "Ignoring rest of specified <wheel_diameter>");
//       break;
//     }
//     impl_->wheel_diameter_[index] = wheel_diameter->Get<double>();
//     RCLCPP_INFO(
//       impl_->ros_node_->get_logger(),
//       "Wheel pair %i diameter set to [%fm]", index + 1, impl_->wheel_diameter_[index]);
//     index++;
//   }

//   impl_->wheel_speed_instr_.assign(2 * impl_->num_wheel_pairs_, 0);
//   impl_->desired_wheel_speed_.assign(2 * impl_->num_wheel_pairs_, 0);

//   // Update rate
//   auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
//   if (update_rate > 0.0) {
//     impl_->update_period_ = 1.0 / update_rate;
//   } else {
//     impl_->update_period_ = 0.0;
//   }
//   impl_->last_update_time_ = _model->GetWorld()->SimTime();

//   impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
//     "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
//     std::bind(&GazeboRosDiffDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));



//   impl_->throttle_speed_sub_ = impl_->ros_node_->create_subscription<argj801_ctl_platform_interfaces::msg::CmdThrottleMsg>(
//     "/ARGJ801/cmd_throttle_msg", qos.get_subscription_qos("/lcm_to_ros/cmd_throttle_msg", rclcpp::QoS(1)),
//     std::bind(&GazeboRosDiffDrivePrivate::OnThrottleMsg, impl_.get(), std::placeholders::_1));

//   /*impl_->vel_msgs_sub_  = impl_->ros_node_->create_subscription<lcm_to_ros_interfaces::msg::CmdVelocityMsg>(
//     "/lcm_to_ros/cmd_velocity_msg", qos.get_subscription_qos("/lcm_to_ros/cmd_velocity_msg", rclcpp::QoS(1)),
//     std::bind(&GazeboRosDiffDrivePrivate::OnVelMsg, impl_.get(), std::placeholders::_1));*/

//       impl_->vel_msgs_sub_  = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
//     "/ARGJ801/cmd_vel", qos.get_subscription_qos("/ARGJ801/cmd_vel", rclcpp::QoS(1)),
//     std::bind(&GazeboRosDiffDrivePrivate::OnVelMsg, impl_.get(), std::placeholders::_1));



//   RCLCPP_INFO(
//     impl_->ros_node_->get_logger(), "Subscribed to [%s]",
//     impl_->cmd_vel_sub_->get_topic_name());

//   // Odometry
//   impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
//   impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
//   impl_->odom_source_ = static_cast<GazeboRosDiffDrivePrivate::OdomSource>(
//     _sdf->Get<int>("odometry_source", 1).first);

//   // Advertise odometry topic
//   impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", false).first;
//   if (impl_->publish_odom_) {
//     impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
//       "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

//     RCLCPP_INFO(
//       impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
//       impl_->odometry_pub_->get_topic_name());
//   }

//   // Create TF broadcaster if needed
//   impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;
//   impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;
//   if (impl_->publish_wheel_tf_ || impl_->publish_odom_tf_) {
//     impl_->transform_broadcaster_ =
//       std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

//     if (impl_->publish_odom_tf_) {
//       RCLCPP_INFO(
//         impl_->ros_node_->get_logger(),
//         "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
//         impl_->robot_base_frame_.c_str());
//     }

//     for (index = 0; index < impl_->num_wheel_pairs_; ++index) {
//       if (impl_->publish_wheel_tf_) {
//         RCLCPP_INFO(
//           impl_->ros_node_->get_logger(),
//           "Publishing wheel transforms between [%s], [%s] and [%s]",
//           impl_->robot_base_frame_.c_str(),
//           impl_->joints_[2 * index + GazeboRosDiffDrivePrivate::LEFT]->GetName().c_str(),
//           impl_->joints_[2 * index + GazeboRosDiffDrivePrivate::RIGHT]->GetName().c_str());
//       }
//     }
//   }

//   impl_->covariance_[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
//   impl_->covariance_[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
//   impl_->covariance_[2] = _sdf->Get<double>("covariance_yaw", 0.001).first;

//   // Listen to the update event (broadcast every simulation iteration)
//   impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
//     std::bind(&GazeboRosDiffDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));

// }
// void GazeboRosDiffDrive::Reset()
// {
//   impl_->last_update_time_ =
//     impl_->joints_[GazeboRosDiffDrivePrivate::LEFT]->GetWorld()->SimTime();
//   for (unsigned int i = 0; i < impl_->num_wheel_pairs_; ++i) {
//     if (impl_->joints_[2 * i + GazeboRosDiffDrivePrivate::LEFT] &&
//       impl_->joints_[2 * i + GazeboRosDiffDrivePrivate::RIGHT])
//     {
//       impl_->joints_[2 * i + GazeboRosDiffDrivePrivate::LEFT]->SetParam(
//         "fmax", 0, impl_->max_wheel_torque_);
//       impl_->joints_[2 * i + GazeboRosDiffDrivePrivate::RIGHT]->SetParam(
//         "fmax", 0, impl_->max_wheel_torque_);
//     }
//   }
//   impl_->pose_encoder_.x = 0;
//   impl_->pose_encoder_.y = 0;
//   impl_->pose_encoder_.theta = 0;
//   impl_->target_x_ = 0;
//   impl_->target_rot_ = 0;
// }

// void GazeboRosDiffDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
// {
// #ifdef IGN_PROFILER_ENABLE
//   IGN_PROFILE("GazeboRosDiffDrivePrivate::OnUpdate");
// #endif
//   // Update encoder even if we're going to skip this update
//   if (odom_source_ == ENCODER) {
//     UpdateOdometryEncoder(_info.simTime);
//   }

//   double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

//   if (seconds_since_last_update < update_period_) {
//     return;
//   }

// #ifdef IGN_PROFILER_ENABLE
//   IGN_PROFILE_BEGIN("UpdateOdometryWorld");
// #endif
//   // Update odom message if using ground truth
//   if (odom_source_ == WORLD) {
//     UpdateOdometryWorld();
//   }
// #ifdef IGN_PROFILER_ENABLE
//   IGN_PROFILE_END();
//   IGN_PROFILE_BEGIN("PublishOdometryMsg");
// #endif
//   if (publish_odom_) {
//     PublishOdometryMsg(_info.simTime);
//   }
// #ifdef IGN_PROFILER_ENABLE
//   IGN_PROFILE_END();
//   IGN_PROFILE_BEGIN("PublishWheelsTf");
// #endif
//   if (publish_wheel_tf_) {
//     PublishWheelsTf(_info.simTime);
//   }
// #ifdef IGN_PROFILER_ENABLE
//   IGN_PROFILE_END();
//   IGN_PROFILE_BEGIN("PublishOdometryTf");
// #endif
//   if (publish_odom_tf_) {
//     PublishOdometryTf(_info.simTime);
//   }
// #ifdef IGN_PROFILER_ENABLE
//   IGN_PROFILE_END();
//   IGN_PROFILE_BEGIN("UpdateWheelVelocities");
// #endif
//   // Update robot in case new velocities have been requested
//   UpdateWheelVelocities();
// #ifdef IGN_PROFILER_ENABLE
//   IGN_PROFILE_END();
// #endif
//   // Current speed
//   //  if (got_new_throttle_sp){
//   //   last_throttle_sp_time_ = _info.simTime;
//   //   got_new_throttle_sp = false;
//   // }
//   // if (got_new_steer_sp){
//   //   last_steer_sp_time_ = _info.simTime;
//   //   got_new_steer_sp = false;
//   // }
//   std::vector<double> current_speed(2 * num_wheel_pairs_);
//   for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
//     current_left_speed = joints_[2 * i + LEFT]->GetVelocity(0);
//     current_speed[2 * i + LEFT] =
//       joints_[2 * i + LEFT]->GetVelocity(0);
//     current_speed[2 * i + RIGHT] =
//       joints_[2 * i + RIGHT]->GetVelocity(0);
//   }
//   current_right_speed = joints_[RIGHT]->GetVelocity(0);
//   current_left_speed = joints_[LEFT]->GetVelocity(0);

//   double current_throttle = 0.5*(current_left_speed+current_right_speed);
//   double current_steer  = 0.5*(-current_left_speed+current_right_speed);
  
//   double next_right_wheels_speed, next_left_wheels_speed; 
//   double next_throttle_speed, next_steer_speed; 

//   double simulation_step = _info.simTime.Double()-last_update_time_.Double();
//   double throttle_acc_sp =  (throttle_sp_rads - current_throttle)/simulation_step;
//   double sat_throttle_acc = doubleSaturation(throttle_acc_sp, a_throttle_max);
//   double steer_acc_sp =  (steer_sp_rads - current_steer)/simulation_step;
//   double sat_steer_acc = doubleSaturation(steer_acc_sp, a_steer_max);
//   next_throttle_speed = current_throttle;
//   next_steer_speed = current_steer;
  
//   // if(_info.simTime.Double() > last_throttle_sp_time_.Double() + dead_zone_time)

//     next_throttle_speed += simulation_step*sat_throttle_acc;
  
//   // if(_info.simTime.Double() > last_steer_sp_time_.Double() + dead_zone_time)
//     next_steer_speed += simulation_step*sat_steer_acc;
  
  
//       std::cout << " ------------------------------------" <<std::endl;
//       std::cout << "simulation_step "<< simulation_step <<std::endl;
//       std::cout << "throttle_acc_sp "<< throttle_acc_sp <<std::endl;
//       std::cout << "current_right_speed "<< current_right_speed <<std::endl;
//       std::cout << "current_left_speed "<< current_left_speed <<std::endl;

//       std::cout << "sat_throttle_acc "<< sat_throttle_acc <<std::endl;
//       std::cout << "max_throttle_acc "<< a_throttle_max <<std::endl;
//       std::cout << "current_throttle "<< floor(100.0*current_throttle) <<std::endl;

//       std::cout << "steer_acc_sp "<< steer_acc_sp <<std::endl;
//       std::cout << "sat_steer_acc "<< sat_steer_acc <<std::endl;
//       std::cout << "next_throttle_speed "<< next_throttle_speed <<std::endl;
//       std::cout << "current right wheel speed "<< current_right_speed <<std::endl;
//       std::cout << "current left wheel speed "<< current_left_speed <<std::endl;      
//       std::cout << "inc "<< simulation_step*sat_throttle_acc <<std::endl;
//       // std::cout << "last_throttle_sp_time_ "<< last_throttle_sp_time_.Double() <<std::endl;
//       // std::cout << "last_steer_sp_time_ "<< last_steer_sp_time_.Double() <<std::endl;
//       std::cout << "_info "<< _info.simTime.Double() <<std::endl;
//       // std::cout << "got_new_throttle_sp "<< got_new_throttle_sp <<std::endl;
//       // std::cout << "got_new_steer_sp "<< got_new_steer_sp <<std::endl;

//   // if (throttle_sp_rads==0.0 || throttle_sp_rads*floor(100.0*current_throttle) < .0)
//   //   next_throttle_speed = 0.0;
//   // else if ((fabs(throttle_sp_rads) <= fabs(current_throttle)) && (throttle_sp_rads*current_throttle > .0)) 
//   //   next_throttle_speed = throttle_sp_rads;
  
//   // if (steer_sp_rads==0.0 || steer_sp_rads*floor(100.0*current_steer) < .0)
//   //   next_steer_speed = 0.0;
//   // else if ((fabs(steer_sp_rads) <= fabs(current_steer)) && (steer_sp_rads*current_steer > .0)) 
//   //   next_steer_speed = steer_sp_rads;
  
//   if (   fabs(next_throttle_speed) > fabs(throttle_sp_rads))
//     next_throttle_speed = throttle_sp_rads;
//   if (   fabs(next_steer_speed) > fabs(steer_sp_rads))
//     next_steer_speed    = steer_sp_rads;



//   // if (throttle_sp_rads == 0.0) next_throttle_speed = 0.0;
//   // if (steer_sp_rads == 0.0) next_steer_speed = 0.0;

//   next_left_wheels_speed = next_throttle_speed-next_steer_speed;
//   next_right_wheels_speed = next_throttle_speed+next_steer_speed;
//   left_wheels_speed  = next_left_wheels_speed;
//   right_wheels_speed = next_right_wheels_speed;
//         std::cout << "next_left_wheels_speed"<< next_left_wheels_speed <<std::endl;
//       std::cout << "next_right_wheels_speed "<< next_right_wheels_speed <<std::endl;  
//             std::cout << " ------------------------------------" <<std::endl;

//   // If max_accel == 0, or target speed is reached
//   for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
//     //desired_wheel_speed_[2 * i + LEFT] = left_wheels_speed;
//     //desired_wheel_speed_[2 * i + RIGHT] = right_wheels_speed;
//     joints_[2 * i + LEFT]->SetParam("vel", 0,  next_left_wheels_speed);
//     joints_[2 * i + RIGHT]->SetParam("vel", 0, next_right_wheels_speed);


//   }

//   last_update_time_ = _info.simTime;
 
// }

// double GazeboRosDiffDrivePrivate::doubleSaturation(double a, double L) {
//     if (a > L) {
//         return L;
//     } else if (a < -L) {
//         return -L;
//     } else {
//         return a;
//     }
// }


// void GazeboRosDiffDrivePrivate::UpdateWheelVelocities()
// {
//   std::lock_guard<std::mutex> scoped_lock(lock_);

//   double vr = target_x_;
//   double va =  target_rot_;
 
//   for (unsigned int i = 0; i < num_wheel_pairs_; ++i) {
//     // desired_wheel_speed_[2 * i + LEFT] = left_wheels_speed;
//     // desired_wheel_speed_[2 * i + RIGHT] = right_wheels_speed;
//     joints_[2*i + LEFT ]->SetVelocity(0, left_wheels_speed);
//     joints_[2*i + RIGHT]->SetVelocity(0, right_wheels_speed);
//   }
//       std::cout << "left_wheels_speed "<< left_wheels_speed <<std::endl;
//       std::cout << "right_wheels_speed "<< right_wheels_speed <<std::endl;


// }



// void GazeboRosDiffDrivePrivate::OnThrottleMsg(const argj801_ctl_platform_interfaces::msg::CmdThrottleMsg::SharedPtr _msg){
  
//   // if (fabs(_msg->throttle - throttle_sp_rads/throttle_to_rads)>=0.1){
//   //   throttle_sp_rads = _msg->throttle * throttle_to_rads;
//   //   got_new_throttle_sp = true;
//   //   }
//   // if (fabs(_msg->steering-steer_sp_rads/steer_to_rads)>=0.1){
//   //   steer_sp_rads = _msg->steering * steer_to_rads;
//   //   got_new_steer_sp = true;
//   // }
//     double limited_steer, limited_throttle;
//     limited_steer = _msg->steering ;
//     limited_throttle= _msg->throttle ;

//     if (limited_steer > 100.0) limited_steer = 100.0;
//     else if (limited_steer < -100.0) limited_steer = -100.0;
//     if (limited_throttle > 100.0) limited_throttle = 100.0;
//     else if (limited_throttle < -100.0) limited_throttle = -100.0;
//     steer_sp_rads = limited_steer* steer_to_rads;
//     throttle_sp_rads = limited_throttle * throttle_to_rads;

// }
 
 



// void GazeboRosDiffDrivePrivate::OnVelMsg(const geometry_msgs::msg::Twist::SharedPtr _msg){

//   std::lock_guard<std::mutex> scoped_lock(lock_);
//   target_x_ = _msg->linear.x;
//   target_rot_ = _msg->angular.z;
// }

// /*void GazeboRosDiffDrivePrivate::OnVelMsg(const lcm_to_ros_interfaces::msg::CmdVelocityMsg::SharedPtr _msg){

//   std::lock_guard<std::mutex> scoped_lock(lock_);
//   target_x_ = _msg->forward_velocity;
//   target_rot_ = _msg->angular_velocity;
// }*/


// void GazeboRosDiffDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
// {
//   //std::lock_guard<std::mutex> scoped_lock(lock_);
//   //target_x_ = _msg->linear.x;
//   //target_rot_ = _msg->angular.z;
// }
//  void GazeboRosDiffDrivePrivate::loop(){}

// void GazeboRosDiffDrivePrivate::UpdateOdometryEncoder(const gazebo::common::Time & _current_time)
// {
//   double vl = joints_[LEFT]->GetVelocity(0);
//   double vr = joints_[RIGHT]->GetVelocity(0);

//   double seconds_since_last_update = (_current_time - last_encoder_update_).Double();
//   last_encoder_update_ = _current_time;

//   double b = wheel_separation_[0];

//   // Book: Sigwart 2011 Autonompus Mobile Robots page:337
//   double sl = vl * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
//   double sr = vr * (wheel_diameter_[0] / 2.0) * seconds_since_last_update;
//   double ssum = sl + sr;

//   double sdiff = sr - sl;

//   double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0 * b));
//   double dy = (ssum) / 2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0 * b));
//   double dtheta = (sdiff) / b;

//   pose_encoder_.x += dx;
//   pose_encoder_.y += dy;
//   pose_encoder_.theta += dtheta;

//   double w = dtheta / seconds_since_last_update;
//   double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

//   tf2::Quaternion qt;
//   tf2::Vector3 vt;
//   qt.setRPY(0, 0, pose_encoder_.theta);
//   vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y, 0);

//   odom_.pose.pose.position.x = vt.x();
//   odom_.pose.pose.position.y = vt.y();
//   odom_.pose.pose.position.z = vt.z();

//   odom_.pose.pose.orientation.x = qt.x();
//   odom_.pose.pose.orientation.y = qt.y();
//   odom_.pose.pose.orientation.z = qt.z();
//   odom_.pose.pose.orientation.w = qt.w();

//   odom_.twist.twist.angular.z = w;
//   odom_.twist.twist.linear.x = v;
//   odom_.twist.twist.linear.y = 0;
// }

// void GazeboRosDiffDrivePrivate::UpdateOdometryWorld()
// {
//   auto pose = model_->WorldPose();
//   odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
//   odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

//   // Get velocity in odom frame
//   auto linear = model_->WorldLinearVel();
//   odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

//   // Convert velocity to child_frame_id(aka base_footprint)
//   float yaw = pose.Rot().Yaw();
//   odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
//   odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
// }

// void GazeboRosDiffDrivePrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
// {
//   geometry_msgs::msg::TransformStamped msg;
//   msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
//   msg.header.frame_id = odometry_frame_;
//   msg.child_frame_id = robot_base_frame_;
//   msg.transform.translation =
//     gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
//   msg.transform.rotation = odom_.pose.pose.orientation;

//   transform_broadcaster_->sendTransform(msg);
// }

// void GazeboRosDiffDrivePrivate::PublishWheelsTf(const gazebo::common::Time & _current_time)
// {
//   for (unsigned int i = 0; i < 2 * num_wheel_pairs_; ++i) {
//     auto pose_wheel = joints_[i]->GetChild()->RelativePose();

//     geometry_msgs::msg::TransformStamped msg;
//     msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
//     msg.header.frame_id = joints_[i]->GetParent()->GetName();
//     msg.child_frame_id = joints_[i]->GetChild()->GetName();
//     msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose_wheel.Pos());
//     msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose_wheel.Rot());

//     transform_broadcaster_->sendTransform(msg);
//   }
// }

// void GazeboRosDiffDrivePrivate::PublishOdometryMsg(const gazebo::common::Time & _current_time)
// {
//   // Set covariance
//   odom_.pose.covariance[0] = covariance_[0];
//   odom_.pose.covariance[7] = covariance_[1];
//   odom_.pose.covariance[14] = 0.01;
//   odom_.pose.covariance[21] = 0.01;
//   odom_.pose.covariance[28] = 0.01;
//   odom_.pose.covariance[35] = covariance_[2];

//   odom_.twist.covariance[0] = covariance_[0];
//   odom_.twist.covariance[7] = covariance_[1];
//   odom_.twist.covariance[14] = 0.01;
//   odom_.twist.covariance[21] = 0.01;
//   odom_.twist.covariance[28] = 0.01;
//   odom_.twist.covariance[35] = covariance_[2];

//   // Set header
//   odom_.header.frame_id = odometry_frame_;
//   odom_.child_frame_id = robot_base_frame_;
//   odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

//   // Publish
//   odometry_pub_->publish(odom_);
// }
// GZ_REGISTER_MODEL_PLUGIN(GazeboRosDiffDrive)
// }  // namespace gazebo_plugins

// gazebo_ros_j8_wheels_plugin.cpp
// Plugin Gazebo-ROS2 para controlar vehículo con CmdThrottleMsg (throttle %, steering %)
// Compatible Gazebo 11 + ROS2 Humble/Foxy
// gazebo_ros_j8_wheels_plugin.cpp
// Plugin Gazebo-ROS2 para controlar vehículo con CmdThrottleMsg (throttle %, steering %)
// Compatible Gazebo 11 + ROS2 Humble/Foxy

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "argj801_ctl_platform_interfaces/msg/cmd_throttle_msg.hpp"

#include <thread>
#include <mutex>
#include <vector>
#include <algorithm>
#include <string>

namespace gazebo {

class GazeboRosJ8WheelsPlugin : public ModelPlugin {
public:
  GazeboRosJ8WheelsPlugin() : ModelPlugin(), alive_(true) {}
  ~GazeboRosJ8WheelsPlugin() override {
    alive_ = false;
    if (spin_thread_.joinable()) spin_thread_.join();
  }

  // Métodos de plugin
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override;
  void UpdateChild();
  void FiniChild() { alive_ = false; if (spin_thread_.joinable()) spin_thread_.join(); }

private:
  // Callbacks ROS
  void cmdThrottleCb(const argj801_ctl_platform_interfaces::msg::CmdThrottleMsg::SharedPtr msg);
  void spinLoop();

  // Funciones internas
  void stepControl(double dt);
  void setJointSpeeds(double wl, double wr);
  void publishOdometry();

  // ROS
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<argj801_ctl_platform_interfaces::msg::CmdThrottleMsg>::SharedPtr sub_thr_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::thread spin_thread_;
  bool alive_;
  std::mutex mutex_;

  // Parámetros
  std::string ns_;
  std::string thr_topic_{"cmd_throttle_msg"};
  std::string odom_topic_{"odom_demo"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  double update_rate_{200.0};
  double update_period_{0.005};
  bool pub_odom_{true};
  bool pub_odom_tf_{true};

  double wheel_separation_{0.34};
  double wheel_diameter_{0.15};
  double max_wheel_torque_{5.0};
  double max_throttle_acc_{1.86};
  double max_steering_acc_{1.23};
  const double thr_to_rad_{0.254608412};
  const double str_to_rad_{0.05787372};
  // const double thr_to_rad_ = 0.101;   // 100 % → 10 rad/s
  // const double str_to_rad_ = 0.0423;  // 100 % → 4.23 rad/s
  // double max_throttle_acc_ = 0.6;     // rad/s²  (≈0.18 m/s²)
  // double max_steering_acc_ = 0.4;     // rad/s²




  // Estado
  double thr_cmd_{0.0}, str_cmd_{0.0};
  double thr_{0.0}, str_{0.0};
  double wl_{0.0}, wr_{0.0};
  nav_msgs::msg::Odometry odom_;

  // Joints
  enum { RIGHT = 0, LEFT = 1 };
  std::vector<std::string> joint_names_[2];
  std::vector<physics::JointPtr> joints_[2];

  // Gazebo
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  common::Time last_update_;
  event::ConnectionPtr update_connection_;
};

void GazeboRosJ8WheelsPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
  model_ = parent;
  world_ = parent->GetWorld();

  // Leer parámetros genéricos
  auto getStr = [&](const std::string &tag, const std::string &def) {
    return sdf->HasElement(tag) ? sdf->Get<std::string>(tag) : def;
  };
  auto getNum = [&](const std::string &tag, double def) {
    return sdf->HasElement(tag) ? sdf->Get<double>(tag) : def;
  };

  update_rate_   = getNum("update_rate", update_rate_);
  update_period_ = update_rate_ > 0.0 ? 1.0/update_rate_ : 0.0;
  max_throttle_acc_ = getNum("max_throttle_acceleration", max_throttle_acc_);
  max_steering_acc_ = getNum("max_steer_acceleration", max_steering_acc_);
  wheel_separation_ = getNum("wheel_separation", wheel_separation_);
  wheel_diameter_   = getNum("wheel_diameter", wheel_diameter_);
  max_wheel_torque_ = getNum("max_wheel_torque", max_wheel_torque_);

  thr_topic_ = getStr("commandThrottleTopic", thr_topic_);
  odom_topic_ = getStr("ros/remapping", odom_topic_);
  pub_odom_    = sdf->Get<bool>("publish_odom", pub_odom_).first;
  pub_odom_tf_ = sdf->Get<bool>("publish_odom_tf", pub_odom_tf_).first;

  // Namespace ROS en <ros><namespace>
  if (sdf->HasElement("ros")) {
    auto rosElm = sdf->GetElement("ros");
    if (rosElm->HasElement("namespace")) {
      ns_ = rosElm->Get<std::string>("namespace");
      if (!ns_.empty() && ns_.front()=='/') ns_.erase(0,1);
    }
  }

  // Limpiar '/' inicial en tópicos relativos
  auto stripSlash = [&](std::string &s){ if(!s.empty() && s.front()=='/') s.erase(0,1); };
  stripSlash(thr_topic_);
  stripSlash(odom_topic_);

  // Leer los joints
  for (auto e = sdf->GetElement("left_joint"); e; e = e->GetNextElement("left_joint"))
    joint_names_[LEFT].push_back(e->Get<std::string>());
  for (auto e = sdf->GetElement("right_joint"); e; e = e->GetNextElement("right_joint"))
    joint_names_[RIGHT].push_back(e->Get<std::string>());
  if (joint_names_[LEFT].size() != joint_names_[RIGHT].size() || joint_names_[LEFT].empty())
    gzthrow("Mismatch in left/right joint count");

  for (int side=0; side<2; ++side) {
    for (auto &jn : joint_names_[side]) {
      auto j = model_->GetJoint(jn);
      if (!j) gzthrow("Joint '" + jn + "' not found");
      j->SetEffortLimit(0, max_wheel_torque_);
      joints_[side].push_back(j);
    }
  }

  // Inicializar ROS
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args"});
  node_ = rclcpp::Node::make_shared("gazebo_ros_j8_wheels_plugin", ns_, opts);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

  sub_thr_  = node_->create_subscription<argj801_ctl_platform_interfaces::msg::CmdThrottleMsg>(
    thr_topic_, 10, std::bind(&GazeboRosJ8WheelsPlugin::cmdThrottleCb, this, std::placeholders::_1));
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

  spin_thread_ = std::thread(&GazeboRosJ8WheelsPlugin::spinLoop, this);

  last_update_ = world_->SimTime();
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosJ8WheelsPlugin::UpdateChild, this));
}

void GazeboRosJ8WheelsPlugin::cmdThrottleCb(const argj801_ctl_platform_interfaces::msg::CmdThrottleMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lk(mutex_);
  double thr = std::clamp<double>(static_cast<double>(msg->throttle), -100.0, 100.0);
  double str = std::clamp<double>(static_cast<double>(msg->steering), -100.0, 100.0);
  thr_cmd_ = thr * thr_to_rad_;  // 100% throttle → thr_to_rad_ * 100 = ~25.46 rad/s
  str_cmd_ = str * str_to_rad_;  // 100% steering → str_to_rad_ * 100 = ~5.79 rad/s
}

void GazeboRosJ8WheelsPlugin::spinLoop() {
  rclcpp::Rate rate(100);
  while (alive_ && rclcpp::ok()) {
    rclcpp::spin_some(node_);
    rate.sleep();
  }
}

void GazeboRosJ8WheelsPlugin::stepControl(double dt) {
  std::lock_guard<std::mutex> lk(mutex_);
  double d_thr = thr_cmd_ - thr_;
  double max_dthr = max_throttle_acc_ * dt;
  thr_ += std::clamp(d_thr, -max_dthr, max_dthr);
  double d_str = str_cmd_ - str_;
  double max_dstr = max_steering_acc_ * dt;
  str_ += std::clamp(d_str, -max_dstr, max_dstr);
  wl_ = thr_ - str_;
  wr_ = thr_ + str_;
}

void GazeboRosJ8WheelsPlugin::setJointSpeeds(double wl, double wr) {
  for (size_t i = 0; i < joints_[LEFT].size(); ++i) {
    joints_[LEFT][i]->SetVelocity(0, wl);
    joints_[RIGHT][i]->SetVelocity(0, wr);
  }
}

void GazeboRosJ8WheelsPlugin::publishOdometry() {
  if (!pub_odom_ && !pub_odom_tf_) return;
  auto pose = model_->WorldPose();
  auto sim = world_->SimTime();
  builtin_interfaces::msg::Time ts;
  ts.sec = sim.sec;
  ts.nanosec = sim.nsec;

  geometry_msgs::msg::TransformStamped tfm;
  tfm.header.stamp = ts;
  tfm.header.frame_id = odom_frame_;
  tfm.child_frame_id = base_frame_;
  tfm.transform.translation.x = pose.Pos().X();
  tfm.transform.translation.y = pose.Pos().Y();
  tfm.transform.translation.z = pose.Pos().Z();
  tfm.transform.rotation.x = pose.Rot().X();
  tfm.transform.rotation.y = pose.Rot().Y();
  tfm.transform.rotation.z = pose.Rot().Z();
  tfm.transform.rotation.w = pose.Rot().W();

  if (pub_odom_tf_) tf_broadcaster_->sendTransform(tfm);

  if (pub_odom_) {
    odom_.header.stamp = ts;
    odom_.header.frame_id = odom_frame_;
    odom_.child_frame_id = base_frame_;
    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();
    odom_.pose.pose.position.z = pose.Pos().Z();
    odom_.pose.pose.orientation.x = pose.Rot().X();
    odom_.pose.pose.orientation.y = pose.Rot().Y();
    odom_.pose.pose.orientation.z = pose.Rot().Z();
    odom_.pose.pose.orientation.w = pose.Rot().W();
    odom_pub_->publish(odom_);
  }
}

void GazeboRosJ8WheelsPlugin::UpdateChild() {
  auto now = world_->SimTime();
  double dt = (now - last_update_).Double();
  if (dt < update_period_) return;
  stepControl(dt);
  setJointSpeeds(wl_, wr_);
  publishOdometry();
  last_update_ = now;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosJ8WheelsPlugin)

} // namespace gazebo
