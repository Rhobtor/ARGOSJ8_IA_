#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to car_interfaces__msg__Graph
/// Graph.msg

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Graph {

    // This member is not documented.
    #[allow(missing_docs)]
    pub nodes: geometry_msgs::msg::PoseArray,

    /// Se interpretan en pares: [nodo0, nodo1, nodo2, nodo3, ...]
    pub edges: Vec<i32>,

}



impl Default for Graph {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Graph::default())
  }
}

impl rosidl_runtime_rs::Message for Graph {
  type RmwMsg = super::msg::rmw::Graph;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        nodes: geometry_msgs::msg::PoseArray::into_rmw_message(std::borrow::Cow::Owned(msg.nodes)).into_owned(),
        edges: msg.edges.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        nodes: geometry_msgs::msg::PoseArray::into_rmw_message(std::borrow::Cow::Borrowed(&msg.nodes)).into_owned(),
        edges: msg.edges.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      nodes: geometry_msgs::msg::PoseArray::from_rmw_message(msg.nodes),
      edges: msg.edges
          .into_iter()
          .collect(),
    }
  }
}


