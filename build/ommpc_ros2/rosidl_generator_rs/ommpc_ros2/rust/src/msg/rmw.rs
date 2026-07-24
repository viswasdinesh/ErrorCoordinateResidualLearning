#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "ommpc_ros2__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ommpc_ros2__msg__PolyTraj() -> *const std::ffi::c_void;
}

#[link(name = "ommpc_ros2__rosidl_generator_c")]
extern "C" {
    fn ommpc_ros2__msg__PolyTraj__init(msg: *mut PolyTraj) -> bool;
    fn ommpc_ros2__msg__PolyTraj__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PolyTraj>, size: usize) -> bool;
    fn ommpc_ros2__msg__PolyTraj__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PolyTraj>);
    fn ommpc_ros2__msg__PolyTraj__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PolyTraj>, out_seq: *mut rosidl_runtime_rs::Sequence<PolyTraj>) -> bool;
}

// Corresponds to ommpc_ros2__msg__PolyTraj
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
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
    pub start_time: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub order: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub coef_x: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub coef_y: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub coef_z: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub duration: rosidl_runtime_rs::Sequence<f32>,

}



impl Default for PolyTraj {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ommpc_ros2__msg__PolyTraj__init(&mut msg as *mut _) {
        panic!("Call to ommpc_ros2__msg__PolyTraj__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PolyTraj {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ommpc_ros2__msg__PolyTraj__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ommpc_ros2__msg__PolyTraj__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ommpc_ros2__msg__PolyTraj__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PolyTraj {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PolyTraj where Self: Sized {
  const TYPE_NAME: &'static str = "ommpc_ros2/msg/PolyTraj";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ommpc_ros2__msg__PolyTraj() }
  }
}


#[link(name = "ommpc_ros2__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ommpc_ros2__msg__ReferenceState() -> *const std::ffi::c_void;
}

#[link(name = "ommpc_ros2__rosidl_generator_c")]
extern "C" {
    fn ommpc_ros2__msg__ReferenceState__init(msg: *mut ReferenceState) -> bool;
    fn ommpc_ros2__msg__ReferenceState__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ReferenceState>, size: usize) -> bool;
    fn ommpc_ros2__msg__ReferenceState__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ReferenceState>);
    fn ommpc_ros2__msg__ReferenceState__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ReferenceState>, out_seq: *mut rosidl_runtime_rs::Sequence<ReferenceState>) -> bool;
}

// Corresponds to ommpc_ros2__msg__ReferenceState
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReferenceState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub timestamp: builtin_interfaces::msg::rmw::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub position: geometry_msgs::msg::rmw::Point,


    // This member is not documented.
    #[allow(missing_docs)]
    pub velocity: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub quaternion: geometry_msgs::msg::rmw::Quaternion,


    // This member is not documented.
    #[allow(missing_docs)]
    pub body_rates: geometry_msgs::msg::rmw::Vector3,


    // This member is not documented.
    #[allow(missing_docs)]
    pub thrust: f64,

}



impl Default for ReferenceState {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ommpc_ros2__msg__ReferenceState__init(&mut msg as *mut _) {
        panic!("Call to ommpc_ros2__msg__ReferenceState__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ReferenceState {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ommpc_ros2__msg__ReferenceState__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ommpc_ros2__msg__ReferenceState__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ommpc_ros2__msg__ReferenceState__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ReferenceState {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ReferenceState where Self: Sized {
  const TYPE_NAME: &'static str = "ommpc_ros2/msg/ReferenceState";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ommpc_ros2__msg__ReferenceState() }
  }
}


