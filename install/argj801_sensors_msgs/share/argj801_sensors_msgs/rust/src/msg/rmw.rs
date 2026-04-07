#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "argj801_sensors_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__argj801_sensors_msgs__msg__MotorTelemetry() -> *const std::ffi::c_void;
}

#[link(name = "argj801_sensors_msgs__rosidl_generator_c")]
extern "C" {
    fn argj801_sensors_msgs__msg__MotorTelemetry__init(msg: *mut MotorTelemetry) -> bool;
    fn argj801_sensors_msgs__msg__MotorTelemetry__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MotorTelemetry>, size: usize) -> bool;
    fn argj801_sensors_msgs__msg__MotorTelemetry__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MotorTelemetry>);
    fn argj801_sensors_msgs__msg__MotorTelemetry__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MotorTelemetry>, out_seq: *mut rosidl_runtime_rs::Sequence<MotorTelemetry>) -> bool;
}

// Corresponds to argj801_sensors_msgs__msg__MotorTelemetry
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotorTelemetry {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub endocer: i64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub rpm: f32,

}



impl Default for MotorTelemetry {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !argj801_sensors_msgs__msg__MotorTelemetry__init(&mut msg as *mut _) {
        panic!("Call to argj801_sensors_msgs__msg__MotorTelemetry__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MotorTelemetry {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { argj801_sensors_msgs__msg__MotorTelemetry__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { argj801_sensors_msgs__msg__MotorTelemetry__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { argj801_sensors_msgs__msg__MotorTelemetry__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MotorTelemetry {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MotorTelemetry where Self: Sized {
  const TYPE_NAME: &'static str = "argj801_sensors_msgs/msg/MotorTelemetry";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__argj801_sensors_msgs__msg__MotorTelemetry() }
  }
}


#[link(name = "argj801_sensors_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__argj801_sensors_msgs__msg__Odometer() -> *const std::ffi::c_void;
}

#[link(name = "argj801_sensors_msgs__rosidl_generator_c")]
extern "C" {
    fn argj801_sensors_msgs__msg__Odometer__init(msg: *mut Odometer) -> bool;
    fn argj801_sensors_msgs__msg__Odometer__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Odometer>, size: usize) -> bool;
    fn argj801_sensors_msgs__msg__Odometer__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Odometer>);
    fn argj801_sensors_msgs__msg__Odometer__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Odometer>, out_seq: *mut rosidl_runtime_rs::Sequence<Odometer>) -> bool;
}

// Corresponds to argj801_sensors_msgs__msg__Odometer
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Odometer {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub odometer: i64,

}



impl Default for Odometer {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !argj801_sensors_msgs__msg__Odometer__init(&mut msg as *mut _) {
        panic!("Call to argj801_sensors_msgs__msg__Odometer__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Odometer {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { argj801_sensors_msgs__msg__Odometer__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { argj801_sensors_msgs__msg__Odometer__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { argj801_sensors_msgs__msg__Odometer__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Odometer {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Odometer where Self: Sized {
  const TYPE_NAME: &'static str = "argj801_sensors_msgs/msg/Odometer";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__argj801_sensors_msgs__msg__Odometer() }
  }
}


