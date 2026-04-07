#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "slg_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__slg_msgs__msg__Segment() -> *const std::ffi::c_void;
}

#[link(name = "slg_msgs__rosidl_generator_c")]
extern "C" {
    fn slg_msgs__msg__Segment__init(msg: *mut Segment) -> bool;
    fn slg_msgs__msg__Segment__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Segment>, size: usize) -> bool;
    fn slg_msgs__msg__Segment__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Segment>);
    fn slg_msgs__msg__Segment__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Segment>, out_seq: *mut rosidl_runtime_rs::Sequence<Segment>) -> bool;
}

// Corresponds to slg_msgs__msg__Segment
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Message for sorted segment of points.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Segment {
    /// Each segment is assigned an id, which uniquely identifies the segment consistently across multiple frames.
    /// The segments can be linked to previous and next segment using lastPointPriorSeg and firstPointNextSeg, respectively.
    /// Segment messages can be used as an input to a classifier such as a person detector, or for annotating
    /// sensor_msgs/Laserscan messages with groundtruth person tracks.
    /// Id of the segment
    pub id: u64,

    /// Label of the segment
    pub label: u32,

    /// Angular distance to the closest boundary (in rads)
    pub angular_distance: f64,

    /// Last point of the previous segment
    pub last_point_prior_segment: geometry_msgs::msg::rmw::Point,

    /// First point of the next segment
    pub first_point_next_segment: geometry_msgs::msg::rmw::Point,

    /// Points per segment
    pub points: rosidl_runtime_rs::Sequence<geometry_msgs::msg::rmw::Point>,

}

impl Segment {
    /// Type of the labels
    pub const BACKGROUND: u32 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PERSON: u32 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PERSON_CANE: u32 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PERSON_WHEEL_CHAIR: u32 = 3;

}


impl Default for Segment {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !slg_msgs__msg__Segment__init(&mut msg as *mut _) {
        panic!("Call to slg_msgs__msg__Segment__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Segment {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { slg_msgs__msg__Segment__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { slg_msgs__msg__Segment__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { slg_msgs__msg__Segment__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Segment {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Segment where Self: Sized {
  const TYPE_NAME: &'static str = "slg_msgs/msg/Segment";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__slg_msgs__msg__Segment() }
  }
}


#[link(name = "slg_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__slg_msgs__msg__SegmentArray() -> *const std::ffi::c_void;
}

#[link(name = "slg_msgs__rosidl_generator_c")]
extern "C" {
    fn slg_msgs__msg__SegmentArray__init(msg: *mut SegmentArray) -> bool;
    fn slg_msgs__msg__SegmentArray__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SegmentArray>, size: usize) -> bool;
    fn slg_msgs__msg__SegmentArray__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SegmentArray>);
    fn slg_msgs__msg__SegmentArray__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SegmentArray>, out_seq: *mut rosidl_runtime_rs::Sequence<SegmentArray>) -> bool;
}

// Corresponds to slg_msgs__msg__SegmentArray
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// Message for array of Segment

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SegmentArray {
    /// Standard message header.
    pub header: std_msgs::msg::rmw::Header,

    /// List of sorted segments.
    pub segments: rosidl_runtime_rs::Sequence<super::super::msg::rmw::Segment>,

}



impl Default for SegmentArray {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !slg_msgs__msg__SegmentArray__init(&mut msg as *mut _) {
        panic!("Call to slg_msgs__msg__SegmentArray__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SegmentArray {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { slg_msgs__msg__SegmentArray__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { slg_msgs__msg__SegmentArray__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { slg_msgs__msg__SegmentArray__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SegmentArray {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SegmentArray where Self: Sized {
  const TYPE_NAME: &'static str = "slg_msgs/msg/SegmentArray";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__slg_msgs__msg__SegmentArray() }
  }
}


