#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to fixposition_driver_ros2__msg__VRTK
///    Copyright (c) 2023
///    Fixposition AG
///
/// Fixposition VRTK Message

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VRTK {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,

    /// frame of the pose values [pose, quaternion]
    pub pose_frame: std::string::String,

    /// frame of the kinematic values [linear/angular velocity, acceleration]
    pub kin_frame: std::string::String,

    /// position, orientation
    pub pose: geometry_msgs::msg::PoseWithCovariance,

    /// linear, angular
    pub velocity: geometry_msgs::msg::TwistWithCovariance,

    /// linear acceleration
    pub acceleration: geometry_msgs::msg::Vector3,

    /// field for the fusion status
    pub fusion_status: i16,

    /// field for the IMU bias status
    pub imu_bias_status: i16,

    /// field for the gnss1 status
    pub gnss1_status: i16,

    /// field for the gnss2 status
    pub gnss2_status: i16,

    /// field for the wheelspeed status
    pub wheelspeed_status: i16,

    /// Fixposition software version
    pub version: std::string::String,

}



impl Default for VRTK {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VRTK::default())
  }
}

impl rosidl_runtime_rs::Message for VRTK {
  type RmwMsg = super::msg::rmw::VRTK;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        pose_frame: msg.pose_frame.as_str().into(),
        kin_frame: msg.kin_frame.as_str().into(),
        pose: geometry_msgs::msg::PoseWithCovariance::into_rmw_message(std::borrow::Cow::Owned(msg.pose)).into_owned(),
        velocity: geometry_msgs::msg::TwistWithCovariance::into_rmw_message(std::borrow::Cow::Owned(msg.velocity)).into_owned(),
        acceleration: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.acceleration)).into_owned(),
        fusion_status: msg.fusion_status,
        imu_bias_status: msg.imu_bias_status,
        gnss1_status: msg.gnss1_status,
        gnss2_status: msg.gnss2_status,
        wheelspeed_status: msg.wheelspeed_status,
        version: msg.version.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        pose_frame: msg.pose_frame.as_str().into(),
        kin_frame: msg.kin_frame.as_str().into(),
        pose: geometry_msgs::msg::PoseWithCovariance::into_rmw_message(std::borrow::Cow::Borrowed(&msg.pose)).into_owned(),
        velocity: geometry_msgs::msg::TwistWithCovariance::into_rmw_message(std::borrow::Cow::Borrowed(&msg.velocity)).into_owned(),
        acceleration: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.acceleration)).into_owned(),
      fusion_status: msg.fusion_status,
      imu_bias_status: msg.imu_bias_status,
      gnss1_status: msg.gnss1_status,
      gnss2_status: msg.gnss2_status,
      wheelspeed_status: msg.wheelspeed_status,
        version: msg.version.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      pose_frame: msg.pose_frame.to_string(),
      kin_frame: msg.kin_frame.to_string(),
      pose: geometry_msgs::msg::PoseWithCovariance::from_rmw_message(msg.pose),
      velocity: geometry_msgs::msg::TwistWithCovariance::from_rmw_message(msg.velocity),
      acceleration: geometry_msgs::msg::Vector3::from_rmw_message(msg.acceleration),
      fusion_status: msg.fusion_status,
      imu_bias_status: msg.imu_bias_status,
      gnss1_status: msg.gnss1_status,
      gnss2_status: msg.gnss2_status,
      wheelspeed_status: msg.wheelspeed_status,
      version: msg.version.to_string(),
    }
  }
}


// Corresponds to fixposition_driver_ros2__msg__Speed
///    Copyright (c) 2023
///    Fixposition AG
///
/// Wheel speed input to Fixposition ROS Driver

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Speed {
    /// Velocity values in [mm/s] (or [mrad/s]) as integer 32bit
    /// 2 Options:
    /// Option 1: One vehicle speed, of sensor RC
    /// Option 2: One vehicle speed, of sensor RC and the yaw rate of the vehicle
    /// Option 3: Fill in 4 Values of 4 wheels, in the order of FR, FL, RR, RL
    pub speeds: Vec<i32>,

}



impl Default for Speed {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Speed::default())
  }
}

impl rosidl_runtime_rs::Message for Speed {
  type RmwMsg = super::msg::rmw::Speed;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        speeds: msg.speeds.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        speeds: msg.speeds.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      speeds: msg.speeds
          .into_iter()
          .collect(),
    }
  }
}


