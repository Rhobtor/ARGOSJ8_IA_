#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "fixposition_driver_ros2__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fixposition_driver_ros2__msg__VRTK() -> *const std::ffi::c_void;
}

#[link(name = "fixposition_driver_ros2__rosidl_generator_c")]
extern "C" {
    fn fixposition_driver_ros2__msg__VRTK__init(msg: *mut VRTK) -> bool;
    fn fixposition_driver_ros2__msg__VRTK__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VRTK>, size: usize) -> bool;
    fn fixposition_driver_ros2__msg__VRTK__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VRTK>);
    fn fixposition_driver_ros2__msg__VRTK__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VRTK>, out_seq: *mut rosidl_runtime_rs::Sequence<VRTK>) -> bool;
}

// Corresponds to fixposition_driver_ros2__msg__VRTK
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

///    Copyright (c) 2023
///    Fixposition AG
///
/// Fixposition VRTK Message

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VRTK {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,

    /// frame of the pose values [pose, quaternion]
    pub pose_frame: rosidl_runtime_rs::String,

    /// frame of the kinematic values [linear/angular velocity, acceleration]
    pub kin_frame: rosidl_runtime_rs::String,

    /// position, orientation
    pub pose: geometry_msgs::msg::rmw::PoseWithCovariance,

    /// linear, angular
    pub velocity: geometry_msgs::msg::rmw::TwistWithCovariance,

    /// linear acceleration
    pub acceleration: geometry_msgs::msg::rmw::Vector3,

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
    pub version: rosidl_runtime_rs::String,

}



impl Default for VRTK {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fixposition_driver_ros2__msg__VRTK__init(&mut msg as *mut _) {
        panic!("Call to fixposition_driver_ros2__msg__VRTK__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VRTK {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fixposition_driver_ros2__msg__VRTK__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fixposition_driver_ros2__msg__VRTK__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fixposition_driver_ros2__msg__VRTK__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VRTK {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VRTK where Self: Sized {
  const TYPE_NAME: &'static str = "fixposition_driver_ros2/msg/VRTK";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fixposition_driver_ros2__msg__VRTK() }
  }
}


#[link(name = "fixposition_driver_ros2__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__fixposition_driver_ros2__msg__Speed() -> *const std::ffi::c_void;
}

#[link(name = "fixposition_driver_ros2__rosidl_generator_c")]
extern "C" {
    fn fixposition_driver_ros2__msg__Speed__init(msg: *mut Speed) -> bool;
    fn fixposition_driver_ros2__msg__Speed__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Speed>, size: usize) -> bool;
    fn fixposition_driver_ros2__msg__Speed__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Speed>);
    fn fixposition_driver_ros2__msg__Speed__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Speed>, out_seq: *mut rosidl_runtime_rs::Sequence<Speed>) -> bool;
}

// Corresponds to fixposition_driver_ros2__msg__Speed
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

///    Copyright (c) 2023
///    Fixposition AG
///
/// Wheel speed input to Fixposition ROS Driver

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Speed {
    /// Velocity values in [mm/s] (or [mrad/s]) as integer 32bit
    /// 2 Options:
    /// Option 1: One vehicle speed, of sensor RC
    /// Option 2: One vehicle speed, of sensor RC and the yaw rate of the vehicle
    /// Option 3: Fill in 4 Values of 4 wheels, in the order of FR, FL, RR, RL
    pub speeds: rosidl_runtime_rs::Sequence<i32>,

}



impl Default for Speed {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !fixposition_driver_ros2__msg__Speed__init(&mut msg as *mut _) {
        panic!("Call to fixposition_driver_ros2__msg__Speed__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Speed {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fixposition_driver_ros2__msg__Speed__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fixposition_driver_ros2__msg__Speed__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { fixposition_driver_ros2__msg__Speed__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Speed {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Speed where Self: Sized {
  const TYPE_NAME: &'static str = "fixposition_driver_ros2/msg/Speed";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__fixposition_driver_ros2__msg__Speed() }
  }
}


