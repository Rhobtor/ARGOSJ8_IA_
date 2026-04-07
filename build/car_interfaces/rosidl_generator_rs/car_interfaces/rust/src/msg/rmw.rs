#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "car_interfaces__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__car_interfaces__msg__Graph() -> *const std::ffi::c_void;
}

#[link(name = "car_interfaces__rosidl_generator_c")]
extern "C" {
    fn car_interfaces__msg__Graph__init(msg: *mut Graph) -> bool;
    fn car_interfaces__msg__Graph__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Graph>, size: usize) -> bool;
    fn car_interfaces__msg__Graph__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Graph>);
    fn car_interfaces__msg__Graph__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Graph>, out_seq: *mut rosidl_runtime_rs::Sequence<Graph>) -> bool;
}

// Corresponds to car_interfaces__msg__Graph
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Graph.msg

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Graph {

    // This member is not documented.
    #[allow(missing_docs)]
    pub nodes: geometry_msgs::msg::rmw::PoseArray,

    /// Se interpretan en pares: [nodo0, nodo1, nodo2, nodo3, ...]
    pub edges: rosidl_runtime_rs::Sequence<i32>,

}



impl Default for Graph {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !car_interfaces__msg__Graph__init(&mut msg as *mut _) {
        panic!("Call to car_interfaces__msg__Graph__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Graph {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { car_interfaces__msg__Graph__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { car_interfaces__msg__Graph__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { car_interfaces__msg__Graph__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Graph {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Graph where Self: Sized {
  const TYPE_NAME: &'static str = "car_interfaces/msg/Graph";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__car_interfaces__msg__Graph() }
  }
}


