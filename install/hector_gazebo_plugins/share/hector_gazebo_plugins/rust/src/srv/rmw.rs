#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "hector_gazebo_plugins__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hector_gazebo_plugins__srv__SetBias_Request() -> *const std::ffi::c_void;
}

#[link(name = "hector_gazebo_plugins__rosidl_generator_c")]
extern "C" {
    fn hector_gazebo_plugins__srv__SetBias_Request__init(msg: *mut SetBias_Request) -> bool;
    fn hector_gazebo_plugins__srv__SetBias_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetBias_Request>, size: usize) -> bool;
    fn hector_gazebo_plugins__srv__SetBias_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetBias_Request>);
    fn hector_gazebo_plugins__srv__SetBias_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetBias_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetBias_Request>) -> bool;
}

// Corresponds to hector_gazebo_plugins__srv__SetBias_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetBias_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub bias: geometry_msgs::msg::rmw::Vector3,

}



impl Default for SetBias_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hector_gazebo_plugins__srv__SetBias_Request__init(&mut msg as *mut _) {
        panic!("Call to hector_gazebo_plugins__srv__SetBias_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetBias_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hector_gazebo_plugins__srv__SetBias_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hector_gazebo_plugins__srv__SetBias_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hector_gazebo_plugins__srv__SetBias_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetBias_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetBias_Request where Self: Sized {
  const TYPE_NAME: &'static str = "hector_gazebo_plugins/srv/SetBias_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hector_gazebo_plugins__srv__SetBias_Request() }
  }
}


#[link(name = "hector_gazebo_plugins__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hector_gazebo_plugins__srv__SetBias_Response() -> *const std::ffi::c_void;
}

#[link(name = "hector_gazebo_plugins__rosidl_generator_c")]
extern "C" {
    fn hector_gazebo_plugins__srv__SetBias_Response__init(msg: *mut SetBias_Response) -> bool;
    fn hector_gazebo_plugins__srv__SetBias_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetBias_Response>, size: usize) -> bool;
    fn hector_gazebo_plugins__srv__SetBias_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetBias_Response>);
    fn hector_gazebo_plugins__srv__SetBias_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetBias_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetBias_Response>) -> bool;
}

// Corresponds to hector_gazebo_plugins__srv__SetBias_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetBias_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for SetBias_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hector_gazebo_plugins__srv__SetBias_Response__init(&mut msg as *mut _) {
        panic!("Call to hector_gazebo_plugins__srv__SetBias_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetBias_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hector_gazebo_plugins__srv__SetBias_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hector_gazebo_plugins__srv__SetBias_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hector_gazebo_plugins__srv__SetBias_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetBias_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetBias_Response where Self: Sized {
  const TYPE_NAME: &'static str = "hector_gazebo_plugins/srv/SetBias_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hector_gazebo_plugins__srv__SetBias_Response() }
  }
}


#[link(name = "hector_gazebo_plugins__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hector_gazebo_plugins__srv__SetReferenceGeoPose_Request() -> *const std::ffi::c_void;
}

#[link(name = "hector_gazebo_plugins__rosidl_generator_c")]
extern "C" {
    fn hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__init(msg: *mut SetReferenceGeoPose_Request) -> bool;
    fn hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetReferenceGeoPose_Request>, size: usize) -> bool;
    fn hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetReferenceGeoPose_Request>);
    fn hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetReferenceGeoPose_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetReferenceGeoPose_Request>) -> bool;
}

// Corresponds to hector_gazebo_plugins__srv__SetReferenceGeoPose_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetReferenceGeoPose_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub geo_pose: geographic_msgs::msg::rmw::GeoPose,

}



impl Default for SetReferenceGeoPose_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__init(&mut msg as *mut _) {
        panic!("Call to hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetReferenceGeoPose_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hector_gazebo_plugins__srv__SetReferenceGeoPose_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetReferenceGeoPose_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetReferenceGeoPose_Request where Self: Sized {
  const TYPE_NAME: &'static str = "hector_gazebo_plugins/srv/SetReferenceGeoPose_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hector_gazebo_plugins__srv__SetReferenceGeoPose_Request() }
  }
}


#[link(name = "hector_gazebo_plugins__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__hector_gazebo_plugins__srv__SetReferenceGeoPose_Response() -> *const std::ffi::c_void;
}

#[link(name = "hector_gazebo_plugins__rosidl_generator_c")]
extern "C" {
    fn hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__init(msg: *mut SetReferenceGeoPose_Response) -> bool;
    fn hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetReferenceGeoPose_Response>, size: usize) -> bool;
    fn hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetReferenceGeoPose_Response>);
    fn hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetReferenceGeoPose_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetReferenceGeoPose_Response>) -> bool;
}

// Corresponds to hector_gazebo_plugins__srv__SetReferenceGeoPose_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetReferenceGeoPose_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for SetReferenceGeoPose_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__init(&mut msg as *mut _) {
        panic!("Call to hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetReferenceGeoPose_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { hector_gazebo_plugins__srv__SetReferenceGeoPose_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetReferenceGeoPose_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetReferenceGeoPose_Response where Self: Sized {
  const TYPE_NAME: &'static str = "hector_gazebo_plugins/srv/SetReferenceGeoPose_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__hector_gazebo_plugins__srv__SetReferenceGeoPose_Response() }
  }
}






#[link(name = "hector_gazebo_plugins__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__hector_gazebo_plugins__srv__SetBias() -> *const std::ffi::c_void;
}

// Corresponds to hector_gazebo_plugins__srv__SetBias
#[allow(missing_docs, non_camel_case_types)]
pub struct SetBias;

impl rosidl_runtime_rs::Service for SetBias {
    type Request = SetBias_Request;
    type Response = SetBias_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__hector_gazebo_plugins__srv__SetBias() }
    }
}




#[link(name = "hector_gazebo_plugins__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__hector_gazebo_plugins__srv__SetReferenceGeoPose() -> *const std::ffi::c_void;
}

// Corresponds to hector_gazebo_plugins__srv__SetReferenceGeoPose
#[allow(missing_docs, non_camel_case_types)]
pub struct SetReferenceGeoPose;

impl rosidl_runtime_rs::Service for SetReferenceGeoPose {
    type Request = SetReferenceGeoPose_Request;
    type Response = SetReferenceGeoPose_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__hector_gazebo_plugins__srv__SetReferenceGeoPose() }
    }
}


