#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to slg_msgs__msg__Segment
/// Message for sorted segment of points.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub last_point_prior_segment: geometry_msgs::msg::Point,

    /// First point of the next segment
    pub first_point_next_segment: geometry_msgs::msg::Point,

    /// Points per segment
    pub points: Vec<geometry_msgs::msg::Point>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Segment::default())
  }
}

impl rosidl_runtime_rs::Message for Segment {
  type RmwMsg = super::msg::rmw::Segment;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        id: msg.id,
        label: msg.label,
        angular_distance: msg.angular_distance,
        last_point_prior_segment: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.last_point_prior_segment)).into_owned(),
        first_point_next_segment: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.first_point_next_segment)).into_owned(),
        points: msg.points
          .into_iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      id: msg.id,
      label: msg.label,
      angular_distance: msg.angular_distance,
        last_point_prior_segment: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.last_point_prior_segment)).into_owned(),
        first_point_next_segment: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.first_point_next_segment)).into_owned(),
        points: msg.points
          .iter()
          .map(|elem| geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      id: msg.id,
      label: msg.label,
      angular_distance: msg.angular_distance,
      last_point_prior_segment: geometry_msgs::msg::Point::from_rmw_message(msg.last_point_prior_segment),
      first_point_next_segment: geometry_msgs::msg::Point::from_rmw_message(msg.first_point_next_segment),
      points: msg.points
          .into_iter()
          .map(geometry_msgs::msg::Point::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to slg_msgs__msg__SegmentArray
/// Message for array of Segment

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SegmentArray {
    /// Standard message header.
    pub header: std_msgs::msg::Header,

    /// List of sorted segments.
    pub segments: Vec<super::msg::Segment>,

}



impl Default for SegmentArray {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::SegmentArray::default())
  }
}

impl rosidl_runtime_rs::Message for SegmentArray {
  type RmwMsg = super::msg::rmw::SegmentArray;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        segments: msg.segments
          .into_iter()
          .map(|elem| super::msg::Segment::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        segments: msg.segments
          .iter()
          .map(|elem| super::msg::Segment::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      segments: msg.segments
          .into_iter()
          .map(super::msg::Segment::from_rmw_message)
          .collect(),
    }
  }
}


