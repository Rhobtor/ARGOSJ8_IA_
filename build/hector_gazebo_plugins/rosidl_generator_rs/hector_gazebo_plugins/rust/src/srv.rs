#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to hector_gazebo_plugins__srv__SetBias_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetBias_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub bias: geometry_msgs::msg::Vector3,

}



impl Default for SetBias_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetBias_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetBias_Request {
  type RmwMsg = super::srv::rmw::SetBias_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        bias: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.bias)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        bias: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.bias)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      bias: geometry_msgs::msg::Vector3::from_rmw_message(msg.bias),
    }
  }
}


// Corresponds to hector_gazebo_plugins__srv__SetBias_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetBias_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for SetBias_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetBias_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetBias_Response {
  type RmwMsg = super::srv::rmw::SetBias_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to hector_gazebo_plugins__srv__SetReferenceGeoPose_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetReferenceGeoPose_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub geo_pose: geographic_msgs::msg::GeoPose,

}



impl Default for SetReferenceGeoPose_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetReferenceGeoPose_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetReferenceGeoPose_Request {
  type RmwMsg = super::srv::rmw::SetReferenceGeoPose_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        geo_pose: geographic_msgs::msg::GeoPose::into_rmw_message(std::borrow::Cow::Owned(msg.geo_pose)).into_owned(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        geo_pose: geographic_msgs::msg::GeoPose::into_rmw_message(std::borrow::Cow::Borrowed(&msg.geo_pose)).into_owned(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      geo_pose: geographic_msgs::msg::GeoPose::from_rmw_message(msg.geo_pose),
    }
  }
}


// Corresponds to hector_gazebo_plugins__srv__SetReferenceGeoPose_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetReferenceGeoPose_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for SetReferenceGeoPose_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetReferenceGeoPose_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetReferenceGeoPose_Response {
  type RmwMsg = super::srv::rmw::SetReferenceGeoPose_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
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


