#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to ommpc_ros2__msg__PolyTraj

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PolyTraj {

    // This member is not documented.
    #[allow(missing_docs)]
    pub drone_id: i16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub traj_id: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start_time: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub order: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub coef_x: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub coef_y: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub coef_z: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub duration: Vec<f32>,

}



impl Default for PolyTraj {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::PolyTraj::default())
  }
}

impl rosidl_runtime_rs::Message for PolyTraj {
  type RmwMsg = super::msg::rmw::PolyTraj;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        drone_id: msg.drone_id,
        traj_id: msg.traj_id,
        start_time: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.start_time)).into_owned(),
        order: msg.order,
        coef_x: msg.coef_x.into(),
        coef_y: msg.coef_y.into(),
        coef_z: msg.coef_z.into(),
        duration: msg.duration.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      drone_id: msg.drone_id,
      traj_id: msg.traj_id,
        start_time: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.start_time)).into_owned(),
      order: msg.order,
        coef_x: msg.coef_x.as_slice().into(),
        coef_y: msg.coef_y.as_slice().into(),
        coef_z: msg.coef_z.as_slice().into(),
        duration: msg.duration.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      drone_id: msg.drone_id,
      traj_id: msg.traj_id,
      start_time: builtin_interfaces::msg::Time::from_rmw_message(msg.start_time),
      order: msg.order,
      coef_x: msg.coef_x
          .into_iter()
          .collect(),
      coef_y: msg.coef_y
          .into_iter()
          .collect(),
      coef_z: msg.coef_z
          .into_iter()
          .collect(),
      duration: msg.duration
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to ommpc_ros2__msg__ReferenceState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReferenceState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub timestamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: geometry_msgs::msg::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub quaternion: geometry_msgs::msg::Quaternion,


    // This member is not documented.
    #[allow(missing_docs)]
    pub body_rates: geometry_msgs::msg::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub thrust: f64,

}



impl Default for ReferenceState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ReferenceState::default())
  }
}

impl rosidl_runtime_rs::Message for ReferenceState {
  type RmwMsg = super::msg::rmw::ReferenceState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        timestamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.timestamp)).into_owned(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Owned(msg.position)).into_owned(),
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.velocity)).into_owned(),
        quaternion: geometry_msgs::msg::Quaternion::into_rmw_message(std::borrow::Cow::Owned(msg.quaternion)).into_owned(),
        body_rates: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Owned(msg.body_rates)).into_owned(),
        thrust: msg.thrust,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        timestamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.timestamp)).into_owned(),
        position: geometry_msgs::msg::Point::into_rmw_message(std::borrow::Cow::Borrowed(&msg.position)).into_owned(),
        velocity: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.velocity)).into_owned(),
        quaternion: geometry_msgs::msg::Quaternion::into_rmw_message(std::borrow::Cow::Borrowed(&msg.quaternion)).into_owned(),
        body_rates: geometry_msgs::msg::Vector3::into_rmw_message(std::borrow::Cow::Borrowed(&msg.body_rates)).into_owned(),
      thrust: msg.thrust,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      timestamp: builtin_interfaces::msg::Time::from_rmw_message(msg.timestamp),
      position: geometry_msgs::msg::Point::from_rmw_message(msg.position),
      velocity: geometry_msgs::msg::Vector3::from_rmw_message(msg.velocity),
      quaternion: geometry_msgs::msg::Quaternion::from_rmw_message(msg.quaternion),
      body_rates: geometry_msgs::msg::Vector3::from_rmw_message(msg.body_rates),
      thrust: msg.thrust,
    }
  }
}


