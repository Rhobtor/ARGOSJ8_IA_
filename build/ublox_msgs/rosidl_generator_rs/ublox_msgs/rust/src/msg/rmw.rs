#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__Ack() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__Ack__init(msg: *mut Ack) -> bool;
    fn ublox_msgs__msg__Ack__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Ack>, size: usize) -> bool;
    fn ublox_msgs__msg__Ack__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Ack>);
    fn ublox_msgs__msg__Ack__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Ack>, out_seq: *mut rosidl_runtime_rs::Sequence<Ack>) -> bool;
}

// Corresponds to ublox_msgs__msg__Ack
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// ACK (0x05)
/// Message Acknowledged / Not-Acknowledged
///
/// Output upon processing of an input message

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Ack {
    /// Class ID of the (Not-)Acknowledged Message
    pub cls_id: u8,

    /// Message ID of the (Not-)Acknowledged Message
    pub msg_id: u8,

}

impl Ack {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NACK_MESSAGE_ID: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ACK_MESSAGE_ID: u8 = 1;

}


impl Default for Ack {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__Ack__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__Ack__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Ack {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__Ack__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__Ack__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__Ack__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Ack {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Ack where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/Ack";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__Ack() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__AidALM() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__AidALM__init(msg: *mut AidALM) -> bool;
    fn ublox_msgs__msg__AidALM__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AidALM>, size: usize) -> bool;
    fn ublox_msgs__msg__AidALM__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AidALM>);
    fn ublox_msgs__msg__AidALM__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AidALM>, out_seq: *mut rosidl_runtime_rs::Sequence<AidALM>) -> bool;
}

// Corresponds to ublox_msgs__msg__AidALM
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// AID-ALM (0x0B 0x30)
/// GPS Aiding Almanach Input/Output Message
///
/// All UBX-AID messages are deprecated; use UBX-MGA messages instead
/// - If the WEEK Value is 0, DWRD0 to DWRD7 are not sent as the almanach is not
///   available for the given SV. This may happen even if NAV-SVINFO and RXM-SVSI
///   are indicating almanac availability as the internal data may not represent
///   the content of an original broadcast almanac (or only parts thereof).
/// - DWORD0 to DWORD7 contain the 8 words following the Hand-Over Word ( HOW )
///   from the GPS navigation message, either pages 1 to 24 of sub-frame 5 or
///   pages 2 to 10 of subframe 4. See IS-GPS-200 for a full description of the
///   contents of the Almanac pages.
/// - In DWORD0 to DWORD7, the parity bits have been removed, and the 24 bits of
///   data are located in Bits 0 to 23. Bits 24 to 31 shall be ignored.
/// - Example: Parameter e (Eccentricity) from Almanach Subframe 4/5, Word 3,
///   Bits 69-84 within the subframe can be found in DWRD0, Bits 15-0 whereas
///   Bit 0 is the LSB.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AidALM {
    /// SV ID for which the receiver shall return its
    /// Almanac Data (Valid Range: 1 .. 32 or 51, 56, 63).
    pub svid: u32,

    /// Issue Date of Almanach (GPS week number)
    pub week: u32,

    /// Start of optional block
    /// Almanac Words
    pub dwrd: rosidl_runtime_rs::Sequence<u32>,

}

impl AidALM {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 11;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 48;

}


impl Default for AidALM {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__AidALM__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__AidALM__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AidALM {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__AidALM__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__AidALM__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__AidALM__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AidALM {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AidALM where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/AidALM";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__AidALM() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__AidEPH() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__AidEPH__init(msg: *mut AidEPH) -> bool;
    fn ublox_msgs__msg__AidEPH__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AidEPH>, size: usize) -> bool;
    fn ublox_msgs__msg__AidEPH__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AidEPH>);
    fn ublox_msgs__msg__AidEPH__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AidEPH>, out_seq: *mut rosidl_runtime_rs::Sequence<AidEPH>) -> bool;
}

// Corresponds to ublox_msgs__msg__AidEPH
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// AID-EPH (0x0B 0x31)
/// GPS Aiding Ephemeris Input/Output Message
///
/// All UBX-AID messages are deprecated; use UBX-MGA messages instead
/// - SF1D0 to SF3D7 is only sent if ephemeris is available for this SV. If not, the payload may
///   be reduced to 8 Bytes, or all bytes are set to zero, indicating that this SV Number does
///   not have valid ephemeris for the moment. This may happen even if NAV-SVINFO and
///   RXM-SVSI are indicating ephemeris availability as the internal data may not represent the
///   content of an original broadcast ephemeris (or only parts thereof).
/// - SF1D0 to SF3D7 contain the 24 words following the Hand-Over Word ( HOW ) from the
///   GPS navigation message, subframes 1 to 3. The Truncated TOW Count is not valid and
///   cannot be used. See IS-GPS-200 for a full description of the contents of the Subframes.
/// - In SF1D0 to SF3D7, the parity bits have been removed, and the 24 bits of data are
///   located in Bits 0 to 23. Bits 24 to 31 shall be ignored.
/// - When polled, the data contained in this message does not represent the full original
///   ephemeris broadcast. Some fields that are irrelevant to u-blox receivers may be missing.
///   The week number in Subframe 1 has already been modified to match the Time Of
///   Ephemeris (TOE).

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AidEPH {
    /// SV ID for which this ephemeris data is
    /// (Valid Range: 1 .. 32).
    pub svid: u32,

    /// Hand-Over Word of first Subframe. This is
    /// required if data is sent to the receiver.
    /// 0 indicates that no Ephemeris Data is following.
    pub how: u32,

    /// Start of optional block
    /// Subframe 1 Words 3..10 (SF1D0..SF1D7)
    pub sf1d: rosidl_runtime_rs::Sequence<u32>,

    /// Subframe 2 Words 3..10 (SF2D0..SF2D7)
    pub sf2d: rosidl_runtime_rs::Sequence<u32>,

    /// Subframe 3 Words 3..10 (SF3D0..SF3D7)
    pub sf3d: rosidl_runtime_rs::Sequence<u32>,

}

impl AidEPH {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 11;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 49;

}


impl Default for AidEPH {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__AidEPH__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__AidEPH__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AidEPH {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__AidEPH__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__AidEPH__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__AidEPH__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AidEPH {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AidEPH where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/AidEPH";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__AidEPH() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__AidHUI() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__AidHUI__init(msg: *mut AidHUI) -> bool;
    fn ublox_msgs__msg__AidHUI__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<AidHUI>, size: usize) -> bool;
    fn ublox_msgs__msg__AidHUI__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<AidHUI>);
    fn ublox_msgs__msg__AidHUI__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<AidHUI>, out_seq: *mut rosidl_runtime_rs::Sequence<AidHUI>) -> bool;
}

// Corresponds to ublox_msgs__msg__AidHUI
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// AID-HUI (0x0B 0x02)
/// GPS Health, UTC and ionosphere parameters
///
/// All UBX-AID messages are deprecated; use UBX-MGA messages instead.
/// This message contains a health bit mask, UTC time and Klobuchar parameters. For more
/// information on these parameters, please see the ICD-GPS-200 documentation.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AidHUI {
    /// Bitmask, every bit represents a GPS SV (1-32).
    /// If the bit is set the SV is healthy.
    pub health: u32,

    /// UTC - parameter A0
    pub utc_a0: f64,

    /// UTC - parameter A1
    pub utc_a1: f64,

    /// UTC - reference time of week
    pub utc_tow: i32,

    /// UTC - reference week number
    pub utc_wnt: i16,

    /// UTC - time difference due to leap seconds before event
    pub utc_ls: i16,

    /// UTC - week number when next leap second event occurs
    pub utc_wnf: i16,

    /// UTC - day of week when next leap second event occurs
    pub utc_dn: i16,

    /// UTC - time difference due to leap seconds after event
    pub utc_lsf: i16,

    /// UTC - Spare to ensure structure is a multiple of 4
    /// bytes
    pub utc_spare: i16,

    /// Klobuchar - alpha 0
    pub klob_a0: f32,

    /// Klobuchar - alpha 1
    pub klob_a1: f32,

    /// Klobuchar - alpha 2
    pub klob_a2: f32,

    /// Klobuchar - alpha 3
    pub klob_a3: f32,

    /// Klobuchar - beta 0
    pub klob_b0: f32,

    /// Klobuchar - beta 1
    pub klob_b1: f32,

    /// Klobuchar - beta 2
    pub klob_b2: f32,

    /// Klobuchar - beta 3
    pub klob_b3: f32,

    /// flags
    pub flags: u32,

}

impl AidHUI {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 11;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 2;

    /// Healthmask field in this message is valid
    pub const FLAGS_HEALTH: u32 = 1;

    /// UTC parameter fields in this message are valid
    pub const FLAGS_UTC: u32 = 2;

    /// Klobuchar parameter fields in this message are
    /// valid
    pub const FLAGS_KLOB: u32 = 4;

}


impl Default for AidHUI {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__AidHUI__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__AidHUI__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for AidHUI {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__AidHUI__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__AidHUI__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__AidHUI__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for AidHUI {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for AidHUI where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/AidHUI";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__AidHUI() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgANT() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgANT__init(msg: *mut CfgANT) -> bool;
    fn ublox_msgs__msg__CfgANT__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgANT>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgANT__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgANT>);
    fn ublox_msgs__msg__CfgANT__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgANT>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgANT>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgANT
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-ANT (0x06 0x13)
/// Antenna Control Settings

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgANT {
    /// Antenna Flag Mask
    pub flags: u16,

    /// Antenna Pin Configuration
    pub pins: u16,

}

impl CfgANT {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 19;

    /// Enable Antenna Supply Voltage Control Signal
    pub const FLAGS_SVCS: u16 = 1;

    /// Enable Short Circuit Detection
    pub const FLAGS_SCD: u16 = 2;

    /// Enable Open Circuit Detection
    pub const FLAGS_OCD: u16 = 4;

    /// Power Down Antenna supply if Short Circuit is
    /// detected. (only in combination with Bit 1)
    pub const FLAGS_PDWN_ON_SCD: u16 = 8;

    /// Enable automatic recovery from short state
    pub const FLAGS_RECOVERY: u16 = 16;

    /// PIO-Pin used for switching antenna supply
    /// (internal to TIM-LP/TIM-LF)
    pub const PIN_SWITCH_MASK: u16 = 31;

    /// PIO-Pin used for detecting a short in the
    /// antenna supply
    pub const PIN_SCD_MASK: u16 = 992;

    /// PIO-Pin used for detecting open/not connected
    /// antenna
    pub const PIN_OCD_MASK: u16 = 31744;

    /// if set to one, and this command is sent to the
    /// receiver, the receiver will reconfigure the
    /// pins as specified.
    pub const PIN_RECONFIG: u16 = 32678;

}


impl Default for CfgANT {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgANT__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgANT__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgANT {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgANT__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgANT__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgANT__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgANT {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgANT where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgANT";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgANT() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgCFG() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgCFG__init(msg: *mut CfgCFG) -> bool;
    fn ublox_msgs__msg__CfgCFG__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgCFG>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgCFG__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgCFG>);
    fn ublox_msgs__msg__CfgCFG__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgCFG>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgCFG>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgCFG
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-CFG (0x06 0x09)
/// Clear, Save and Load configurations

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgCFG {
    /// Mask with configuration sub-sections to Clear
    /// (=Load Default Configurations to Permanent
    /// Configurations in non-volatile memory)
    pub clear_mask: u32,

    /// Mask with configuration sub-section to Save
    /// (=Save Current Configuration to Non-volatile
    /// Memory)
    pub save_mask: u32,

    /// Mask with configuration sub-sections to Load
    /// (=Load Permanent Configurations from
    /// Non-volatile Memory to Current Configurations)
    pub load_mask: u32,

    /// Mask which selects the devices for this command
    pub device_mask: u8,

}

impl CfgCFG {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 9;

    /// Communications port settings. Modifying this
    /// sub-section results in an IO system reset.
    /// Because of this undefined data may be output
    /// for a short period of time after receiving the
    /// message.
    pub const MASK_IO_PORT: u32 = 1;

    /// Message Configuration
    pub const MASK_MSG_CONF: u32 = 2;

    /// INF Message Configuration
    pub const MASK_INF_MSG: u32 = 4;

    /// Navigation Configuration
    pub const MASK_NAV_CONF: u32 = 8;

    /// Receiver Manager Configuration
    pub const MASK_RXM_CONF: u32 = 16;

    /// Sensor Interface Configuration, protocol >= 19
    pub const MASK_SEN_CONF: u32 = 256;

    /// Remote Inventory Configuration
    pub const MASK_RINV_CONF: u32 = 512;

    /// Antenna Configuration
    pub const MASK_ANT_CONF: u32 = 1024;

    /// Logging Configuration
    pub const MASK_LOG_CONF: u32 = 2048;

    /// FTS Configuration. Only applicable to the
    /// FTS product variant.
    pub const MASK_FTS_CONF: u32 = 4096;

    /// device battery backed RAM
    pub const DEV_BBR: u8 = 1;

    /// device Flash
    pub const DEV_FLASH: u8 = 2;

    /// device EEPROM
    pub const DEV_EEPROM: u8 = 4;

    /// device SPI Flash
    pub const DEV_SPI_FLASH: u8 = 16;

}


impl Default for CfgCFG {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgCFG__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgCFG__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgCFG {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgCFG__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgCFG__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgCFG__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgCFG {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgCFG where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgCFG";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgCFG() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgDAT() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgDAT__init(msg: *mut CfgDAT) -> bool;
    fn ublox_msgs__msg__CfgDAT__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgDAT>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgDAT__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgDAT>);
    fn ublox_msgs__msg__CfgDAT__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgDAT>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgDAT>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgDAT
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-DAT (0x06 0x06)
/// Set User-defined Datum
///
/// For more information see the description of Geodetic Systems and Frames.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgDAT {
    /// Only for GET, these values are not used for write
    /// Datum Number: 0 = WGS84, 0xFFFF = user-defined
    pub datum_num: u16,

    /// ASCII String: WGS84 or USER
    pub datum_name: [u8; 6],

    /// Semi-major Axis
    /// accepted range = 6,300,000.0 to 6,500,000.0 meters
    pub maj_a: f64,

    /// 1.0 / Flattening
    /// accepted range is 0.0 to 500.0
    pub flat: f64,

    /// X Axis shift at the origin
    /// accepted range is +/- 5000.0 meters
    pub d_x: f32,

    /// Y Axis shift at the origin
    /// accepted range is +/- 5000.0 meters
    pub d_y: f32,

    /// Z Axis shift at the origin
    /// accepted range is +/- 5000.0 meters
    pub d_z: f32,

    /// Rotation about the X Axis
    /// accepted range is +/- 20.0 milli-arc seconds
    pub rot_x: f32,

    /// Rotation about the Y Axis
    /// accepted range is +/- 20.0 milli-arc seconds
    pub rot_y: f32,

    /// Rotation about the Z Axis
    /// accepted range is +/- 20.0 milli-arc seconds
    pub rot_z: f32,

    /// Scale change
    /// accepted range is 0.0 to 50.0 parts per million
    pub scale: f32,

}

impl CfgDAT {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DATUM_NUM_WGS84: u16 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DATUM_NUM_USER: u16 = 65535;

}


impl Default for CfgDAT {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgDAT__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgDAT__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgDAT {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgDAT__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgDAT__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgDAT__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgDAT {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgDAT where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgDAT";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgDAT() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgDGNSS() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgDGNSS__init(msg: *mut CfgDGNSS) -> bool;
    fn ublox_msgs__msg__CfgDGNSS__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgDGNSS>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgDGNSS__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgDGNSS>);
    fn ublox_msgs__msg__CfgDGNSS__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgDGNSS>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgDGNSS>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgDGNSS
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-DGNSS (0x06 0x70)
/// DGNSS configuration
///
/// This message allows the user to configure the DGNSS configuration of the
/// receiver.
/// Supported on:
///  - u-blox 8 / u-blox M8 from protocol version 20.01 up to version 23.01 (only
///    with High Precision GNSS products)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgDGNSS {
    /// Specifies differential mode:
    pub dgnss_mode: u8,

    /// Reserved
    pub reserved0: [u8; 3],

}

impl CfgDGNSS {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 112;

    /// RTK float: No attempts are made to fix
    /// ambiguities.
    pub const DGNSS_MODE_RTK_FLOAT: u8 = 2;

    /// RTK fixed: Ambiguities are fixed whenever
    /// possible.
    pub const DGNSS_MODE_RTK_FIXED: u8 = 3;

}


impl Default for CfgDGNSS {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgDGNSS__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgDGNSS__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgDGNSS {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgDGNSS__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgDGNSS__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgDGNSS__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgDGNSS {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgDGNSS where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgDGNSS";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgDGNSS() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgGNSSBlock() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgGNSSBlock__init(msg: *mut CfgGNSSBlock) -> bool;
    fn ublox_msgs__msg__CfgGNSSBlock__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgGNSSBlock>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgGNSSBlock__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgGNSSBlock>);
    fn ublox_msgs__msg__CfgGNSSBlock__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgGNSSBlock>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgGNSSBlock>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgGNSSBlock
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// see Cfg-GNSS message

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgGNSSBlock {
    /// System identifier (see Satellite Numbering)
    pub gnss_id: u8,

    /// (Read only in protocol versions greater than 23)
    /// Number of reserved (minimum) tracking channels
    /// for this GNSS system
    pub res_trk_ch: u8,

    /// (Read only in protocol versions greater than 23)
    /// Maximum number of tracking channels used for this
    /// system. Must be > 0, >= resTrkChn, <= numTrkChUse and
    /// <= maximum number of tracking channels supported for
    /// this system
    pub max_trk_ch: u8,

    /// Reserved
    pub reserved1: u8,

    /// Bitfield of flags. At least one signal must be
    /// configured in every enabled system.
    pub flags: u32,

}

impl CfgGNSSBlock {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GNSS_ID_GPS: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GNSS_ID_SBAS: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GNSS_ID_GALILEO: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GNSS_ID_BEIDOU: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GNSS_ID_IMES: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GNSS_ID_QZSS: u8 = 5;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GNSS_ID_GLONASS: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RES_TRK_CH_GPS: u8 = 8;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RES_TRK_CH_QZSS: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RES_TRK_CH_SBAS: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const RES_TRK_CH_GLONASS: u8 = 8;

    /// max_trk_ch must have this minimum value
    /// for each enabled major GNSS
    pub const MAX_TRK_CH_MAJOR_MIN: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MAX_TRK_CH_GPS: u8 = 16;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MAX_TRK_CH_GLONASS: u8 = 14;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MAX_TRK_CH_QZSS: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MAX_TRK_CH_SBAS: u8 = 3;

    /// Enable this system
    pub const FLAGS_ENABLE: u32 = 1;

    /// Signal configuration mask
    pub const FLAGS_SIG_CFG_MASK: u32 = 16711680;

    /// When gnssId is 0 (GPS)
    /// * 0x01 = GPS L1C/A
    pub const SIG_CFG_GPS_L1CA: u32 = 65536;

    /// When gnssId is 1 (SBAS)
    /// * 0x01 = SBAS L1C/A
    pub const SIG_CFG_SBAS_L1CA: u32 = 65536;

    /// When gnssId is 2 (Galileo)
    /// * 0x01 = Galileo E1OS (not supported in
    ///   protocol versions less than 18)
    pub const SIG_CFG_GALILEO_E1OS: u32 = 65536;

    /// When gnssId is 3 (BeiDou)
    /// * 0x01 = BeiDou B1I
    pub const SIG_CFG_BEIDOU_B1I: u32 = 65536;

    /// When gnssId is 4 (IMES)
    /// * 0x01 = IMES L1
    pub const SIG_CFG_IMES_L1: u32 = 65536;

    /// When gnssId is 5 (QZSS)
    /// * 0x01 = QZSS L1C/A
    pub const SIG_CFG_QZSS_L1CA: u32 = 65536;

    /// * 0x04 = QZSS L1SAIF
    pub const SIG_CFG_QZSS_L1SAIF: u32 = 262144;

    /// When gnssId is 6 (GLONASS)
    /// * 0x01 = GLONASS L1OF
    pub const SIG_CFG_GLONASS_L1OF: u32 = 65536;

}


impl Default for CfgGNSSBlock {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgGNSSBlock__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgGNSSBlock__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgGNSSBlock {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgGNSSBlock__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgGNSSBlock__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgGNSSBlock__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgGNSSBlock {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgGNSSBlock where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgGNSSBlock";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgGNSSBlock() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgGNSS() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgGNSS__init(msg: *mut CfgGNSS) -> bool;
    fn ublox_msgs__msg__CfgGNSS__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgGNSS>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgGNSS__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgGNSS>);
    fn ublox_msgs__msg__CfgGNSS__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgGNSS>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgGNSS>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgGNSS
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-GNSS (0x06 0x3E)
/// GNSS Configuration
///
/// Gets or sets the GNSS system channel sharing configuration.
/// If the receiver is sent a valid new configuration, it will respond with a
/// UBX-ACK-ACK message and immediately change to the new configuration. Otherwise
/// the receiver will reject the request, by issuing a UBX-ACK-NAK and continuing
/// operation with the previous configuration.
/// Configuration requirements:
///  - It is necessary for at least one major GNSS to be enabled, after applying
///    the new configuration to the current one.
///  - It is also required that at least 4 tracking channels are available to each
///    enabled major GNSS, i.e. maxTrkCh must have a minimum value of 4 for each
///    enabled major GNSS.
///  - The number of tracking channels in use must not exceed the number of
///    tracking channels available in hardware, and the sum of all reserved
///    tracking channels needs to be less than or equal to the number of tracking
///    channels in use.
/// Notes:
///  - To avoid cross-correlation issues, it is recommended that GPS and QZSS are
///    always both enabled or both disabled.
///  - Polling this message returns the configuration of all supported GNSS,
///    whether enabled or not; it may also include GNSS unsupported by the
///    particular product, but in such cases the enable flag will always be unset.
///  - See section GNSS Configuration for a discussion of the use of this message
///    and section Satellite Numbering for a description of the GNSS IDs available
///  - Configuration specific to the GNSS system can be done via other messages
///    (e.g. UBX-CFG-SBAS).

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgGNSS {
    /// Message version (= 0 for this version)
    pub msg_ver: u8,

    /// Number of tracking channels in hardware (read only)
    pub num_trk_ch_hw: u8,

    /// (Read only in protocol versions greater than 23)
    /// Number of tracking channels to use (<= numTrkChHw)
    /// If 0xFF, then number of tracking channels to use will
    /// be set to numTrkChHw
    pub num_trk_ch_use: u8,

    /// Number of configuration blocks following
    pub num_config_blocks: u8,

    /// Start of repeated block (numConfigBlocks times)
    pub blocks: rosidl_runtime_rs::Sequence<super::super::msg::rmw::CfgGNSSBlock>,

}

impl CfgGNSS {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 62;

}


impl Default for CfgGNSS {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgGNSS__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgGNSS__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgGNSS {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgGNSS__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgGNSS__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgGNSS__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgGNSS {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgGNSS where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgGNSS";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgGNSS() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgHNR() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgHNR__init(msg: *mut CfgHNR) -> bool;
    fn ublox_msgs__msg__CfgHNR__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgHNR>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgHNR__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgHNR>);
    fn ublox_msgs__msg__CfgHNR__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgHNR>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgHNR>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgHNR
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-HNR (0x06 0x5C)
/// High Navigation Rate Settings
///
/// The u-blox receivers support high rates of navigation update up to 30 Hz.
/// The navigation solution output (NAV-HNR) will not be aligned to the top of a
/// second.
/// The update rate has a direct influence on the power consumption. The more
/// fixes that are required, the more CPU power and communication resources are
/// required.
/// For most applications a 1 Hz update rate would be sufficient.
///
/// (only with ADR or UDR products)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgHNR {
    /// Rate of navigation solution output
    pub high_nav_rate: u8,

    /// Reserved
    pub reserved0: [u8; 3],

}

impl CfgHNR {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 92;

}


impl Default for CfgHNR {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgHNR__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgHNR__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgHNR {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgHNR__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgHNR__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgHNR__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgHNR {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgHNR where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgHNR";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgHNR() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgINFBlock() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgINFBlock__init(msg: *mut CfgINFBlock) -> bool;
    fn ublox_msgs__msg__CfgINFBlock__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgINFBlock>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgINFBlock__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgINFBlock>);
    fn ublox_msgs__msg__CfgINFBlock__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgINFBlock>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgINFBlock>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgINFBlock
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// See CfgINF message

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgINFBlock {
    /// Protocol Identifier, identifying for which
    /// protocol the configuration is set/get. The
    /// following are valid Protocol Identifiers:
    /// 0: UBX Protocol
    /// 1: NMEA Protocol
    /// 2-255: Reserved
    pub protocol_id: u8,

    /// Reserved
    pub reserved1: [u8; 3],

    /// A bit mask, saying which information messages
    /// are enabled on each I/O port
    pub inf_msg_mask: [u8; 6],

}

impl CfgINFBlock {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PROTOCOL_ID_UBX: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PROTOCOL_ID_NMEA: u8 = 1;

    /// enable ERROR
    pub const INF_MSG_ERROR: u8 = 1;

    /// enable WARNING
    pub const INF_MSG_WARNING: u8 = 2;

    /// enable NOTICE
    pub const INF_MSG_NOTICE: u8 = 4;

    /// enable TEST
    pub const INF_MSG_TEST: u8 = 8;

    /// enable DEBUG
    pub const INF_MSG_DEBUG: u8 = 16;

}


impl Default for CfgINFBlock {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgINFBlock__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgINFBlock__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgINFBlock {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgINFBlock__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgINFBlock__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgINFBlock__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgINFBlock {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgINFBlock where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgINFBlock";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgINFBlock() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgINF() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgINF__init(msg: *mut CfgINF) -> bool;
    fn ublox_msgs__msg__CfgINF__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgINF>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgINF__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgINF>);
    fn ublox_msgs__msg__CfgINF__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgINF>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgINF>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgINF
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-INF  (0x06 0x02)
/// Information message configuration
///
/// The value of infMsgMask below are that each bit represents one of the INF
/// class messages (Bit 0 for ERROR, Bit 1 for WARNING and so on.). For a complete
/// list, see the Message Class INF. Several configurations can be concatenated to
/// one input message.
/// In this case the payload length can be a multiple of the normal length. Output
/// messages from the module contain only one configuration unit. Note that I/O
/// Ports 1 and 2 correspond to serial ports 1 and 2. I/O port 0 is DDC. I/O port
/// 3 is USB. I/O port 4 is SPI. I/O port 5 is reserved for future use

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgINF {
    /// start of repeated block
    pub blocks: rosidl_runtime_rs::Sequence<super::super::msg::rmw::CfgINFBlock>,

}

impl CfgINF {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 2;

}


impl Default for CfgINF {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgINF__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgINF__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgINF {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgINF__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgINF__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgINF__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgINF {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgINF where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgINF";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgINF() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgMSG() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgMSG__init(msg: *mut CfgMSG) -> bool;
    fn ublox_msgs__msg__CfgMSG__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgMSG>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgMSG__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgMSG>);
    fn ublox_msgs__msg__CfgMSG__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgMSG>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgMSG>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgMSG
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-MSG (0x06 0x01)
/// Message Rate(s)
///
/// Set message rate for the current port

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgMSG {
    /// Message Class
    pub msg_class: u8,

    /// Message Identifier
    pub msg_id: u8,

    /// Send rate on current port
    /// [number of navigation solutions]
    pub rate: u8,

}

impl CfgMSG {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 1;

}


impl Default for CfgMSG {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgMSG__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgMSG__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgMSG {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgMSG__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgMSG__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgMSG__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgMSG {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgMSG where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgMSG";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgMSG() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNAV5() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgNAV5__init(msg: *mut CfgNAV5) -> bool;
    fn ublox_msgs__msg__CfgNAV5__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgNAV5>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgNAV5__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgNAV5>);
    fn ublox_msgs__msg__CfgNAV5__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgNAV5>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgNAV5>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgNAV5
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-NAV5 (0x06 0x24)
/// Navigation Engine Settings

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgNAV5 {
    /// Parameters Bitmask. Only the masked
    /// parameters will be applied.
    pub mask: u16,

    /// Dynamic Platform model:
    pub dyn_model: u8,

    /// Position Fixing Mode.
    pub fix_mode: u8,

    /// Fixed altitude (mean sea level) for 2D fix mode.
    /// [m / 0.01]
    pub fixed_alt: i32,

    /// Fixed altitude variance for 2D mode.
    pub fixed_alt_var: u32,

    /// Minimum Elevation for a GNSS satellite to be used in
    /// NAV
    pub min_elev: i8,

    /// Maximum time to perform dead reckoning
    /// (linear extrapolation) in case of GPS signal loss
    pub dr_limit: u8,

    /// Position DOP Mask to use
    pub p_dop: u16,

    /// Time DOP Mask to use
    pub t_dop: u16,

    /// Position Accuracy Mask
    pub p_acc: u16,

    /// Time Accuracy Mask
    pub t_acc: u16,

    /// Static hold threshold
    pub static_hold_thresh: u8,

    /// DGNSS timeout, firmware 7 and newer only
    pub dgnss_time_out: u8,

    /// Number of satellites required to have C/N0 above
    /// cnoThresh for a fix to be attempted
    pub cno_thresh_num_svs: u8,

    /// C/N0 threshold for deciding whether to attempt a fix
    /// [dBHz]
    pub cno_thresh: u8,

    /// Reserved
    pub reserved1: [u8; 2],

    /// Static hold distance threshold (before quitting
    /// static hold)
    pub static_hold_max_dist: u16,

    /// UTC standard to be used:
    pub utc_standard: u8,

    /// Reserved
    pub reserved2: [u8; 5],

}

impl CfgNAV5 {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 36;

    /// Apply dynamic model settings
    pub const MASK_DYN: u16 = 1;

    /// Apply minimum elevation settings
    pub const MASK_MIN_EL: u16 = 2;

    /// Apply fix mode settings
    pub const MASK_FIX_MODE: u16 = 4;

    /// Apply DR limit settings
    pub const MASK_DR_LIM: u16 = 8;

    /// Apply position mask settings
    pub const MASK_POS_MASK: u16 = 16;

    /// Apply time mask settings
    pub const MASK_TIME_MASK: u16 = 32;

    /// Apply static hold settings
    pub const MASK_STATIC_HOLD_MASK: u16 = 64;

    /// Apply DGPS settings, firmware >= 7 only
    pub const MASK_DGPS_MASK: u16 = 128;

    /// Apply CNO threshold settings
    pub const MASK_CNO: u16 = 256;

    /// Apply UTC settings, protocol >= 16 only
    pub const MASK_UTC: u16 = 1024;

    /// Portable
    pub const DYN_MODEL_PORTABLE: u8 = 0;

    /// Stationary
    pub const DYN_MODEL_STATIONARY: u8 = 2;

    /// Pedestrian
    pub const DYN_MODEL_PEDESTRIAN: u8 = 3;

    /// Automotive
    pub const DYN_MODEL_AUTOMOTIVE: u8 = 4;

    /// Sea
    pub const DYN_MODEL_SEA: u8 = 5;

    /// Airborne with <1g Acceleration
    pub const DYN_MODEL_AIRBORNE_1G: u8 = 6;

    /// Airborne with <2g Acceleration
    pub const DYN_MODEL_AIRBORNE_2G: u8 = 7;

    /// Airborne with <4g Acceleration
    pub const DYN_MODEL_AIRBORNE_4G: u8 = 8;

    /// Wrist watch, protocol >= 18
    pub const DYN_MODEL_WRIST_WATCH: u8 = 9;

    /// 2D only
    pub const FIX_MODE_2D_ONLY: u8 = 1;

    /// 3D only
    pub const FIX_MODE_3D_ONLY: u8 = 2;

    /// Auto 2D/3D
    pub const FIX_MODE_AUTO: u8 = 3;

    /// receiver selects based on GNSS configuration
    pub const UTC_STANDARD_AUTOMATIC: u8 = 0;

    /// UTC as operated by the U.S. Naval Observatory
    /// (USNO); derived from GPS time
    pub const UTC_STANDARD_GPS: u8 = 3;

    /// UTC as operated by the former Soviet Union;
    /// derived from GLONASS time
    pub const UTC_STANDARD_GLONASS: u8 = 6;

    /// UTC as operated by the National Time Service
    /// Center, China; derived from BeiDou time
    pub const UTC_STANDARD_BEIDOU: u8 = 7;

}


impl Default for CfgNAV5 {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgNAV5__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgNAV5__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgNAV5 {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNAV5__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNAV5__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNAV5__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgNAV5 {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgNAV5 where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgNAV5";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNAV5() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNAVX5() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgNAVX5__init(msg: *mut CfgNAVX5) -> bool;
    fn ublox_msgs__msg__CfgNAVX5__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgNAVX5>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgNAVX5__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgNAVX5>);
    fn ublox_msgs__msg__CfgNAVX5__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgNAVX5>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgNAVX5>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgNAVX5
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-NAVX5 (0x06 0x23)
/// Navigation Engine Expert Settings
///
/// Warning: Refer to u-blox protocol spec before changing these settings.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgNAVX5 {
    /// Message version (set to 0)
    pub version: u16,

    /// First parameters bitmask (possible values below)
    pub mask1: u16,

    /// Second parameters bitmask (possible values below)
    /// Firmware >=8 only
    pub mask2: u32,

    /// Always set to zero
    pub reserved1: [u8; 2],

    /// Minimum number of satellites for navigation
    pub min_svs: u8,

    /// Maximum number of satellites for navigation
    pub max_svs: u8,

    /// Minimum satellite signal level for navigation
    pub min_cno: u8,

    /// Always set to zero
    pub reserved2: u8,

    /// If set to 1, initial fix must be 3D
    pub ini_fix3d: u8,

    /// Always set to zero
    pub reserved3: [u8; 2],

    /// If set to 1, issue acknowledgments for assistance
    pub ack_aiding: u8,

    /// GPS week rollover number, GPS week numbers will be set
    /// correctly from this week up to 1024 weeks after this
    /// week
    pub wkn_rollover: u16,

    /// Permanently attenuated signal compensation
    /// 0 = disabled, 255 = automatic
    /// 1..63 = maximum expected C/N0 value
    /// Firmware 8 only
    pub sig_atten_comp_mode: u8,

    /// Always set to zero
    pub reserved4: [u8; 5],

    /// Enable/disable PPP (on supported units)
    pub use_ppp: u8,

    /// AssistNow Autonomous configuration, 1: enabled
    pub aop_cfg: u8,

    /// Always set to zero
    pub reserved5: [u8; 2],

    /// Maximum acceptable (modeled) autonomous orbit
    /// error
    /// valid range = 5..1000
    /// 0 = reset to firmware default
    pub aop_orb_max_err: u16,

    /// Always set to zero
    pub reserved6: [u8; 7],

    /// Enable/disable ADR sensor fusion
    /// 1: enabled, 0: disabled
    /// Only supported on certain products
    pub use_adr: u8,

}

impl CfgNAVX5 {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 35;

    /// apply min/max SVs settings
    pub const MASK1_MIN_MAX: u16 = 4;

    /// apply minimum C/N0 setting
    pub const MASK1_MIN_CNO: u16 = 8;

    /// apply initial 3D fix settings
    pub const MASK1_INITIAL_FIX_3D: u16 = 64;

    /// apply GPS week number rollover settings
    pub const MASK1_WKN_ROLL: u16 = 512;

    /// apply assistance acknowledgment
    /// settings
    pub const MASK1_ACK_AID: u16 = 1024;

    /// apply usePPP flag
    pub const MASK1_PPP: u16 = 8192;

    /// apply aopCfg (useAOP flag) and
    /// aopOrbMaxErr settings
    /// (AssistNow Autonomous)
    pub const MASK1_AOP: u16 = 16384;

    /// Apply ADR sensor fusion on/off
    /// setting
    pub const MASK2_ADR: u32 = 64;

    /// Apply signal attenuation
    /// compensation feature settings
    pub const MASK2_SIG_ATTEN_COMP_MODE: u32 = 128;

}


impl Default for CfgNAVX5 {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgNAVX5__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgNAVX5__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgNAVX5 {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNAVX5__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNAVX5__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNAVX5__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgNAVX5 {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgNAVX5 where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgNAVX5";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNAVX5() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNMEA6() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgNMEA6__init(msg: *mut CfgNMEA6) -> bool;
    fn ublox_msgs__msg__CfgNMEA6__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgNMEA6>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgNMEA6__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgNMEA6>);
    fn ublox_msgs__msg__CfgNMEA6__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgNMEA6>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgNMEA6>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgNMEA6
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-NMEA (0x06 0x17)
/// NMEA protocol configuration
///
/// Set/Get the NMEA protocol configuration. See section NMEA Protocol
/// Configuration for a detailed description of the configuration effects on
/// NMEA output
///
/// Supported on u-blox 6 from firmware version 6.00 up to version 7.03.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgNMEA6 {
    /// filter flags
    pub filter: u8,

    /// NMEA version
    pub version: u8,

    /// Maximum Number of SVs to report in NMEA
    /// protocol.
    /// This does not affect the receiver's operation.
    /// It only limits the number of SVs reported in
    /// NMEA mode (this might be needed with older
    /// mapping applications which only support 8- or
    /// 12-channel receivers)
    pub num_sv: u8,

    /// flags
    pub flags: u8,

}

impl CfgNMEA6 {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 23;

    /// Disable position filtering
    pub const FILTER_POS: u8 = 1;

    /// Disable masked position filtering
    pub const FILTER_MSK_POS: u8 = 2;

    /// Disable time filtering
    pub const FILTER_TIME: u8 = 4;

    /// Disable date filtering
    pub const FILTER_DATE: u8 = 8;

    /// Enable SBAS filtering
    pub const FILTER_SBAS_FILT: u8 = 16;

    /// Disable track filtering
    pub const FILTER_TRACK: u8 = 32;

    /// Version 2.3
    pub const NMEA_VERSION_2_3: u8 = 35;

    /// Version 2.1
    pub const NMEA_VERSION_2_1: u8 = 33;

    /// enable compatibility mode.
    /// This might be needed for certain applications
    /// when customer's NMEA parser expects a fixed
    /// number of  digits in position coordinates
    pub const FLAGS_COMPAT: u8 = 1;

    /// enable considering mode
    pub const FLAGS_CONSIDER: u8 = 2;

}


impl Default for CfgNMEA6 {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgNMEA6__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgNMEA6__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgNMEA6 {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNMEA6__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNMEA6__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNMEA6__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgNMEA6 {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgNMEA6 where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgNMEA6";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNMEA6() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNMEA7() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgNMEA7__init(msg: *mut CfgNMEA7) -> bool;
    fn ublox_msgs__msg__CfgNMEA7__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgNMEA7>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgNMEA7__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgNMEA7>);
    fn ublox_msgs__msg__CfgNMEA7__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgNMEA7>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgNMEA7>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgNMEA7
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-NMEA (0x06 0x17)
/// NMEA protocol configuration V0
///
/// Set/Get the NMEA protocol configuration. See section NMEA Protocol
/// Configuration for a detailed description of the configuration effects on
/// NMEA output
///
/// Supported on: u-blox 7 firmware version 1.00

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgNMEA7 {
    /// filter flags
    pub filter: u8,

    /// NMEA version
    pub nmea_version: u8,

    /// Maximum Number of SVs to report per TalkerId:
    /// unlimited (0) or 8, 12, 16
    pub num_sv: u8,

    /// flags
    pub flags: u8,

    /// Filters out satellites based on their GNSS.
    /// If a bitfield is enabled, the corresponding
    /// satellites will be not output.
    pub gnss_to_filter: u32,

    /// Configures the display of satellites that do not
    /// have an NMEA-defined value. Note: this does not
    /// apply to satellites with an unknown ID.
    pub sv_numbering: u8,

    /// By default the main Talker ID (i.e. the Talker
    /// ID used  for all messages other than GSV) is
    /// determined by the  GNSS assignment of the
    /// receiver's channels (see CfgGNSS).
    /// This field enables the main Talker ID to be
    /// overridden
    pub main_talker_id: u8,

    /// By default the Talker ID for GSV messages is
    /// GNSS specific (as defined by NMEA). This field
    /// enables the GSV Talker ID to be overridden.
    pub gsv_talker_id: u8,

    /// Reserved
    pub reserved: u8,

}

impl CfgNMEA7 {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 23;

    /// Enable position output for failed or
    /// invalid fixes
    pub const FILTER_POS: u8 = 1;

    /// Enable position output for invalid fixes
    pub const FILTER_MSK_POS: u8 = 2;

    /// Enable time output for invalid times
    pub const FILTER_TIME: u8 = 4;

    /// Enable date output for invalid dates
    pub const FILTER_DATE: u8 = 8;

    /// Restrict output to GPS satellites only
    pub const FILTER_GPS_ONLY: u8 = 16;

    /// Enable COG output even if COG is frozen
    pub const FILTER_TRACK: u8 = 32;

    /// Version 2.3
    pub const NMEA_VERSION_2_3: u8 = 35;

    /// Version 2.1
    pub const NMEA_VERSION_2_1: u8 = 33;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NUM_SV_UNLIMITED: u8 = 0;

    /// enable compatibility mode.
    /// This might be needed for certain applications
    /// when customer's NMEA parser expects a fixed
    /// number of  digits in position coordinates
    pub const FLAGS_COMPAT: u8 = 1;

    /// enable considering mode
    pub const FLAGS_CONSIDER: u8 = 2;

    /// Disable reporting of GPS satellites
    pub const GNSS_TO_FILTER_GPS: u32 = 1;

    /// Disable reporting of SBAS satellites
    pub const GNSS_TO_FILTER_SBAS: u32 = 2;

    /// Disable reporting of QZSS satellites
    pub const GNSS_TO_FILTER_QZSS: u32 = 16;

    /// Disable reporting of GLONASS satellites
    pub const GNSS_TO_FILTER_GLONASS: u32 = 32;

    /// Strict - Satellites are not output
    pub const SV_NUMBERING_STRICT: u8 = 0;

    /// Extended - Use proprietary numbering
    pub const SV_NUMBERING_EXTENDED: u8 = 1;

    /// Main Talker ID is not overridden
    pub const MAIN_TALKER_ID_NOT_OVERRIDDEN: u8 = 0;

    /// Set main Talker ID to 'GP'
    pub const MAIN_TALKER_ID_GP: u8 = 1;

    /// Set main Talker ID to 'GL'
    pub const MAIN_TALKER_ID_GL: u8 = 2;

    /// Set main Talker ID to 'GN'
    pub const MAIN_TALKER_ID_GN: u8 = 3;

    /// Use GNSS specific Talker ID
    /// (as defined by NMEA)
    pub const GSV_TALKER_ID_GNSS_SPECIFIC: u8 = 0;

    /// Use the main Talker ID
    pub const GSV_TALKER_ID_MAIN: u8 = 1;

}


impl Default for CfgNMEA7 {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgNMEA7__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgNMEA7__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgNMEA7 {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNMEA7__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNMEA7__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNMEA7__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgNMEA7 {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgNMEA7 where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgNMEA7";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNMEA7() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNMEA() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgNMEA__init(msg: *mut CfgNMEA) -> bool;
    fn ublox_msgs__msg__CfgNMEA__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgNMEA>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgNMEA__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgNMEA>);
    fn ublox_msgs__msg__CfgNMEA__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgNMEA>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgNMEA>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgNMEA
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-NMEA (0x06 0x17)
/// NMEA protocol configuration V1
///
/// Set/Get the NMEA protocol configuration. See section NMEA Protocol
/// Configuration for a detailed description of the configuration effects on
/// NMEA output

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgNMEA {
    /// filter flags
    pub filter: u8,

    /// NMEA version
    pub nmea_version: u8,

    /// Maximum Number of SVs to report per TalkerId:
    /// unlimited (0) or 8, 12, 16
    pub num_sv: u8,

    /// flags
    pub flags: u8,

    /// Filters out satellites based on their GNSS.
    /// If a bitfield is enabled, the corresponding
    /// satellites will be not output.
    pub gnss_to_filter: u32,

    /// Configures the display of satellites that do not
    /// have an NMEA-defined value. Note: this does not
    /// apply to satellites with an unknown ID.
    pub sv_numbering: u8,

    /// By default the main Talker ID (i.e. the Talker
    /// ID used  for all messages other than GSV) is
    /// determined by the  GNSS assignment of the
    /// receiver's channels (see CfgGNSS).
    /// This field enables the main Talker ID to be
    /// overridden
    pub main_talker_id: u8,

    /// By default the Talker ID for GSV messages is
    /// GNSS specific (as defined by NMEA). This field
    /// enables the GSV Talker ID to be overridden.
    pub gsv_talker_id: u8,

    /// Message version (set to 1 for this version)
    pub version: u8,

    /// Sets the two characters that should be used
    /// for the BeiDou Talker ID. If these are set to
    /// zero, the default BeiDou TalkerId will be used
    pub bds_talker_id: [u8; 2],

    /// Reserved
    pub reserved1: [u8; 6],

}

impl CfgNMEA {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 23;

    /// Enable position output for failed or
    /// invalid fixes
    pub const FILTER_POS: u8 = 1;

    /// Enable position output for invalid fixes
    pub const FILTER_MSK_POS: u8 = 2;

    /// Enable time output for invalid times
    pub const FILTER_TIME: u8 = 4;

    /// Enable date output for invalid dates
    pub const FILTER_DATE: u8 = 8;

    /// Restrict output to GPS satellites only
    pub const FILTER_GPS_ONLY: u8 = 16;

    /// Enable COG output even if COG is frozen
    pub const FILTER_TRACK: u8 = 32;

    /// Version 4.1
    pub const NMEA_VERSION_4_1: u8 = 65;

    /// Version 4.0
    pub const NMEA_VERSION_4_0: u8 = 64;

    /// Version 2.3
    pub const NMEA_VERSION_2_3: u8 = 35;

    /// Version 2.1
    pub const NMEA_VERSION_2_1: u8 = 33;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const NUM_SV_UNLIMITED: u8 = 0;

    /// enable compatibility mode.
    /// This might be needed for certain applications
    /// when customer's NMEA parser expects a fixed
    /// number of  digits in position coordinates
    pub const FLAGS_COMPAT: u8 = 1;

    /// enable considering mode
    pub const FLAGS_CONSIDER: u8 = 2;

    /// enable strict limit to 82 characters maximum
    pub const FLAGS_LIMIT82: u8 = 4;

    /// enable high precision mode
    /// This flag cannot be set in conjunction with
    /// either Compatibility Mode or Limit82 Mode.
    /// (not supported in protocol versions < 20.01)
    pub const FLAGS_HIGH_PREC: u8 = 8;

    /// Disable reporting of GPS satellites
    pub const GNSS_TO_FILTER_GPS: u32 = 1;

    /// Disable reporting of SBAS satellites
    pub const GNSS_TO_FILTER_SBAS: u32 = 2;

    /// Disable reporting of QZSS satellites
    pub const GNSS_TO_FILTER_QZSS: u32 = 16;

    /// Disable reporting of GLONASS satellites
    pub const GNSS_TO_FILTER_GLONASS: u32 = 32;

    /// Disable reporting of BeiDou satellites
    pub const GNSS_TO_FILTER_BEIDOU: u32 = 64;

    /// Strict - Satellites are not output
    pub const SV_NUMBERING_STRICT: u8 = 0;

    /// Extended - Use proprietary numbering
    pub const SV_NUMBERING_EXTENDED: u8 = 1;

    /// Main Talker ID is not overridden
    pub const MAIN_TALKER_ID_NOT_OVERRIDDEN: u8 = 0;

    /// Set main Talker ID to 'GP'
    pub const MAIN_TALKER_ID_GP: u8 = 1;

    /// Set main Talker ID to 'GL'
    pub const MAIN_TALKER_ID_GL: u8 = 2;

    /// Set main Talker ID to 'GN'
    pub const MAIN_TALKER_ID_GN: u8 = 3;

    /// Set main Talker ID to 'GA'
    pub const MAIN_TALKER_ID_GA: u8 = 4;

    /// Set main Talker ID to 'GB'
    pub const MAIN_TALKER_ID_GB: u8 = 5;

    /// Use GNSS specific Talker ID
    /// (as defined by NMEA)
    pub const GSV_TALKER_ID_GNSS_SPECIFIC: u8 = 0;

    /// Use the main Talker ID
    pub const GSV_TALKER_ID_MAIN: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const VERSION: u8 = 1;

}


impl Default for CfgNMEA {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgNMEA__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgNMEA__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgNMEA {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNMEA__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNMEA__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgNMEA__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgNMEA {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgNMEA where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgNMEA";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgNMEA() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgPRT() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgPRT__init(msg: *mut CfgPRT) -> bool;
    fn ublox_msgs__msg__CfgPRT__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgPRT>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgPRT__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgPRT>);
    fn ublox_msgs__msg__CfgPRT__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgPRT>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgPRT>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgPRT
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-PRT (0x06 0x00)
/// Port Configuration for DDC, UART, USB, SPI
///
/// Several configurations can be concatenated to one input message. In this case
/// the payload length can be a multiple of the normal length (see the other
/// versions of CFG-PRT). Output messages from the module contain only one
/// configuration unit.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgPRT {
    /// Port Identifier Number
    pub port_id: u8,

    /// Reserved
    pub reserved0: u8,

    /// TX ready PIN configuration (since Firmware 7.01)
    /// reserved (Always set to zero) up to Firmware 7.01
    pub tx_ready: u16,

    /// A bit mask describing the DDC, UART or SPI mode
    /// Reserved for USB
    pub mode: u32,

    /// UART Baudrate in bits/second
    /// Reserved for USB, SPI, DDC
    /// Possible values: 4800, 9600, 19200, 38400, 57600,
    ///                  115200, 230400, 460800
    pub baud_rate: u32,

    /// A mask describing which input protocols are active.
    /// Each bit of this mask is used for a protocol.
    /// Through that, multiple protocols can be defined
    /// on a single port.
    pub in_proto_mask: u16,

    /// A mask describing which output protocols are active.
    /// Each bit of this mask is used for a protocol.
    /// Through that, multiple protocols can be defined
    /// on a single port.
    pub out_proto_mask: u16,

    /// Flags for UART & SPI, Reserved for USB
    pub flags: u16,

    /// Always set to zero
    pub reserved1: u16,

}

impl CfgPRT {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PORT_ID_DDC: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PORT_ID_UART1: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PORT_ID_UART2: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PORT_ID_USB: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PORT_ID_SPI: u8 = 4;

    /// Enable TX ready feature for this
    /// port
    pub const TX_READY_EN: u16 = 1;

    /// Polarity High-active
    pub const TX_READY_POLARITY_HIGH_ACTIVE: u16 = 0;

    /// Polarity Low-active
    pub const TX_READY_POLARITY_LOW_ACTIVE: u16 = 2;

    /// PIO to be used (must not be in use
    /// already by another function)
    pub const TX_READY_PIN_SHIFT: u16 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TX_READY_PIN_MASK: u16 = 124;

    /// Threshold
    pub const TX_READY_THRES_SHIFT: u16 = 7;

    /// The given threshold is multiplied by
    /// 8 bytes.
    /// The TX ready PIN goes active
    /// after >= thres*8 bytes are pending
    /// for the port and going inactive
    /// after the last pending bytes have
    /// been written to hardware (0-4 bytes
    /// before end of stream).
    pub const TX_READY_THRES_MASK: u16 = 65408;

    /// DDC Mode Constants
    pub const MODE_DDC_SLAVE_ADDR_SHIFT: u32 = 1;

    /// Slave address
    /// Range: 0x07 < slaveAddr < 0x78.
    pub const MODE_DDC_SLAVE_ADDR_MASK: u32 = 254;

    /// UART Mode Constants
    /// Default 1 for compatibility with A4
    pub const MODE_RESERVED1: u32 = 16;

    /// Character Length
    pub const MODE_CHAR_LEN_MASK: u32 = 192;

    /// 5bit (not supported)
    pub const MODE_CHAR_LEN_5BIT: u32 = 0;

    /// 6bit (not supported)
    pub const MODE_CHAR_LEN_6BIT: u32 = 64;

    /// 7bit (supported only with parity)
    pub const MODE_CHAR_LEN_7BIT: u32 = 128;

    /// 8bit
    pub const MODE_CHAR_LEN_8BIT: u32 = 192;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MODE_PARITY_MASK: u32 = 3584;

    /// Even Parity
    pub const MODE_PARITY_EVEN: u32 = 0;

    /// Odd Parity
    pub const MODE_PARITY_ODD: u32 = 512;

    /// No Parity (even/odd ignored)
    pub const MODE_PARITY_NO: u32 = 2048;

    /// Number of Stop Bits
    pub const MODE_STOP_BITS_MASK: u32 = 12288;

    /// 1 Stop Bit
    pub const MODE_STOP_BITS_1: u32 = 0;

    /// 1.5 Stop Bit
    pub const MODE_STOP_BITS_15: u32 = 4096;

    /// 2 Stop Bit
    pub const MODE_STOP_BITS_2: u32 = 8192;

    /// 0.5 Stop Bit
    pub const MODE_STOP_BITS_05: u32 = 12288;

    /// SPI Mode Constants
    /// SPI Mode CPOL (0/1)
    pub const MODE_SPI_SPI_MODE_CPOL: u32 = 4;

    /// SPI Mode CPH (0/1)
    /// (both CPOL/CPHA) can be = 1
    pub const MODE_SPI_SPI_MODE_CPHA: u32 = 2;

    /// (u-blox 6 only)
    /// 0 Flow control disabled
    /// 1 Flow control enabled (9-bit mode)
    pub const MODE_SPI_FLOW_CONTROL: u32 = 64;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MODE_SPI_FF_COUNT_SHIFT: u32 = 8;

    /// Number of bytes containing 0xFF to
    /// receive before switching off
    /// reception.
    /// Range: 0(mechanism off)-63
    pub const MODE_SPI_FF_COUNT_MASK: u32 = 16128;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PROTO_UBX: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const PROTO_NMEA: u16 = 2;

    /// only for inProtoMask
    pub const PROTO_RTCM: u16 = 4;

    /// (not supported in protocol versions less than 20)
    pub const PROTO_RTCM3: u16 = 32;

    /// if set, the port will timeout if
    /// allocated TX memory >=4 kB and no
    /// activity for 1.5s. If not set the port
    /// will timoout if no activity for 1.5s
    /// regardless on the amount of allocated
    /// TX memory.
    pub const FLAGS_EXTENDED_TX_TIMEOUT: u16 = 2;

}


impl Default for CfgPRT {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgPRT__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgPRT__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgPRT {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgPRT__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgPRT__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgPRT__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgPRT {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgPRT where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgPRT";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgPRT() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgRATE() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgRATE__init(msg: *mut CfgRATE) -> bool;
    fn ublox_msgs__msg__CfgRATE__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgRATE>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgRATE__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgRATE>);
    fn ublox_msgs__msg__CfgRATE__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgRATE>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgRATE>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgRATE
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-RATE (0x06 0x08)
/// Navigation/Measurement Rate Settings
///
/// This message allows the user to alter the rate at which navigation solutions
/// (and the measurements that they depend on) are generated by the receiver. The
/// calculation of the navigation solution will always be aligned to the top of a
/// second zero (first second of the week) of the configured reference time
/// system. For protocol version 18 and later the navigation period is an integer
/// multiple of the measurement period.
///  - Each measurement triggers the measurements generation and raw data output.
///  - The navRate value defines that every nth measurement triggers a navigation
///    epoch.
///  - The update rate has a direct influence on the power consumption. The more
///    fixes that are required, the more CPU power and communication resources
///    are required.
///  - For most applications a 1 Hz update rate would be sufficient.
///  - When using Power Save Mode, measurement and navigation rate can differ from
///    the values configured here

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgRATE {
    /// Measurement Rate, GPS measurements are
    /// taken every measRate milliseconds
    pub meas_rate: u16,

    /// Navigation Rate, in number of measurement
    /// cycles. On u-blox 5 and u-blox 6, this parameter
    /// cannot be changed, and always equals 1.
    pub nav_rate: u16,

    /// The time system to which measurements are aligned
    pub time_ref: u16,

}

impl CfgRATE {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 8;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TIME_REF_UTC: u16 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TIME_REF_GPS: u16 = 1;

    /// not supported in protocol versions less than 18
    pub const TIME_REF_GLONASS: u16 = 2;

    /// not supported in protocol versions less than 18
    pub const TIME_REF_BEIDOU: u16 = 3;

    /// not supported in protocol versions less than 18
    pub const TIME_REF_GALILEO: u16 = 4;

}


impl Default for CfgRATE {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgRATE__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgRATE__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgRATE {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgRATE__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgRATE__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgRATE__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgRATE {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgRATE where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgRATE";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgRATE() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgRST() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgRST__init(msg: *mut CfgRST) -> bool;
    fn ublox_msgs__msg__CfgRST__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgRST>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgRST__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgRST>);
    fn ublox_msgs__msg__CfgRST__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgRST>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgRST>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgRST
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-RST (0x06 0x04)
/// Reset Receiver / Clear Backup Data Structures
///
/// Don't expect this message to be acknowledged by the receiver.
///  - Newer FW version won't acknowledge this message at all.
///  - Older FW version will acknowledge this message but the acknowledge may not
///    be sent completely before the receiver is reset.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgRST {
    /// BBR Sections to clear.
    pub nav_bbr_mask: u16,

    /// Reset Type:
    pub reset_mode: u8,

    /// Reserved
    pub reserved1: u8,

}

impl CfgRST {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 4;

    /// The following Special Sets apply:
    /// Hot start the device
    pub const NAV_BBR_HOT_START: u16 = 0;

    /// Warm start the device
    pub const NAV_BBR_WARM_START: u16 = 1;

    /// Cold start the device
    pub const NAV_BBR_COLD_START: u16 = 65535;

    /// Ephemeris
    pub const NAV_BBR_EPH: u16 = 1;

    /// Almanac
    pub const NAV_BBR_ALM: u16 = 2;

    /// Health
    pub const NAV_BBR_HEALTH: u16 = 4;

    /// Klobuchar parameters
    pub const NAV_BBR_KLOB: u16 = 8;

    /// Position
    pub const NAV_BBR_POS: u16 = 16;

    /// Clock Drift
    pub const NAV_BBR_CLKD: u16 = 32;

    /// Oscillator Parameter
    pub const NAV_BBR_OSC: u16 = 64;

    /// UTC Correction + GPS Leap Seconds Parameters
    pub const NAV_BBR_UTC: u16 = 128;

    /// RTC
    pub const NAV_BBR_RTC: u16 = 256;

    /// Autonomous Orbit Parameters
    pub const NAV_BBR_AOP: u16 = 32768;

    /// Hardware reset (Watchdog) immediately
    pub const RESET_MODE_HW_IMMEDIATE: u8 = 0;

    /// Controlled Software reset
    pub const RESET_MODE_SW: u8 = 1;

    /// Controlled Software reset (GNSS only)
    pub const RESET_MODE_GNSS: u8 = 2;

    /// Hardware reset (Watchdog) after
    /// shutdown
    pub const RESET_MODE_HW_AFTER_SHUTDOWN: u8 = 4;

    /// Controlled GNSS stop
    pub const RESET_MODE_GNSS_STOP: u8 = 8;

    /// Controlled GNSS start
    pub const RESET_MODE_GNSS_START: u8 = 9;

}


impl Default for CfgRST {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgRST__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgRST__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgRST {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgRST__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgRST__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgRST__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgRST {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgRST where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgRST";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgRST() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgSBAS() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgSBAS__init(msg: *mut CfgSBAS) -> bool;
    fn ublox_msgs__msg__CfgSBAS__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgSBAS>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgSBAS__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgSBAS>);
    fn ublox_msgs__msg__CfgSBAS__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgSBAS>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgSBAS>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgSBAS
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-SBAS (0x06 0x16)
/// SBAS Configuration
///
/// This message configures the SBAS receiver subsystem (i.e. WAAS, EGNOS, MSAS).
/// See the SBAS Configuration Settings Description for a detailed description of
/// how these settings affect receiver operation

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgSBAS {
    /// SBAS Mode
    pub mode: u8,

    /// SBAS Usage
    pub usage: u8,

    /// Maximum Number of SBAS prioritized tracking
    /// channels (valid range: 0 - 3) to use
    /// (obsolete and superseeded by UBX-CFG-GNSS in protocol
    /// versions 14+).
    pub max_sbas: u8,

    /// Continuation of scanmode bitmask below
    /// PRN 152...158
    pub scanmode2: u8,

    /// Which SBAS PRN numbers to search for (Bitmask)
    /// If all Bits are set to zero, auto-scan (i.e. all valid
    /// PRNs) are searched. Every bit corresponds to a PRN
    /// number.
    /// PRN 120..151
    pub scanmode1: u32,

}

impl CfgSBAS {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 22;

    /// SBAS Enabled (1) / Disabled (0)
    ///  This field is deprecated; use UBX-CFG-GNSS to
    /// enable/disable SBAS operation
    pub const MODE_ENABLED: u8 = 1;

    /// SBAS Testbed: Use data anyhow (1) / Ignore data when
    /// in Test Mode (SBAS Msg 0)
    pub const MODE_TEST: u8 = 2;

    /// Use SBAS GEOs as a ranging source (for navigation)
    pub const USAGE_RANGE: u8 = 1;

    /// Use SBAS Differential Corrections
    pub const USAGE_DIFF_CORR: u8 = 2;

    /// Use SBAS Integrity Information
    pub const USAGE_INTEGRITY: u8 = 4;

}


impl Default for CfgSBAS {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgSBAS__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgSBAS__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgSBAS {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgSBAS__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgSBAS__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgSBAS__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgSBAS {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgSBAS where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgSBAS";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgSBAS() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgTMODE3() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgTMODE3__init(msg: *mut CfgTMODE3) -> bool;
    fn ublox_msgs__msg__CfgTMODE3__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgTMODE3>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgTMODE3__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgTMODE3>);
    fn ublox_msgs__msg__CfgTMODE3__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgTMODE3>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgTMODE3>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgTMODE3
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// CFG-TMODE3 (0x06, 0x71)
/// Time Mode Settings 3
///
/// Configures the receiver to be in Time Mode. The position referred to in this
/// message is that of the Antenna Reference Point (ARP). See the Time Mode
/// Description for details.
///
/// Supported on:
///  - u-blox 8 / u-blox M8 with protocol version 20 (only with High Precision
///    GNSS products)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgTMODE3 {
    /// Message version (0x00 for this version)
    pub version: u8,

    /// Reserved
    pub reserved1: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub flags: u16,

    /// WGS84 ECEF X coordinate (or latitude) of
    /// the ARP position, depending on flags above
    /// [cm] or [deg / 1e-7]
    pub ecef_x_or_lat: i32,

    /// WGS84 ECEF Y coordinate (or longitude) of
    /// the ARP position, depending on flags above
    /// [cm] or [deg / 1e-7]
    pub ecef_y_or_lon: i32,

    /// WGS84 ECEF Z coordinate (or altitude) of
    /// the ARP position, depending on flags above
    /// [cm]
    pub ecef_z_or_alt: i32,

    /// High-precision WGS84 ECEF X coordinate (or
    /// latitude) of the ARP position, depending on
    /// flags above. Must be in the range -99..+99.
    /// The precise WGS84 ECEF X coordinate in units
    /// of cm, or the precise WGS84 ECEF latitude in
    /// units of 1e-7 degrees, is given by
    /// ecefXOrLat + (ecefXOrLatHP * 1e-2)
    /// [0.1 mm] or [deg * 1e-9]
    pub ecef_x_or_lat_hp: i8,

    /// High-precision WGS84 ECEF Y coordinate (or
    /// longitude) of the ARP position, depending on
    /// flags above. Must be in the range -99..+99.
    /// The precise WGS84 ECEF Y coordinate in units
    /// of cm, or the precise WGS84 ECEF longitude
    /// in units of 1e-7 degrees, is given by
    /// ecefYOrLon + (ecefYOrLonHP * 1e-2)
    /// [0.1 mm] or [deg * 1e-9]
    pub ecef_y_or_lon_hp: i8,

    /// High-precision WGS84 ECEF Z coordinate (or
    /// altitude) of the ARP position, depending on
    /// flags above. Must be in the range -99..+99.
    /// The precise WGS84 ECEF Z coordinate, or
    /// altitude coordinate, in units of cm is given
    /// by ecefZOrAlt + (ecefZOrAltHP * 1e-2)
    /// [0.1 mm]
    pub ecef_z_or_alt_hp: i8,

    /// Reserved
    pub reserved2: u8,

    /// Fixed position 3D accuracy
    /// [0.1 mm]
    pub fixed_pos_acc: u32,

    /// Survey-in minimum duration
    /// [s]
    pub svin_min_dur: u32,

    /// Survey-in position accuracy limit
    /// [0.1 mm]
    pub svin_acc_limit: u32,

    /// Reserved
    pub reserved3: [u8; 8],

}

impl CfgTMODE3 {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 113;

    /// Receiver Mode:
    pub const FLAGS_MODE_MASK: u16 = 255;

    /// Disabled
    pub const FLAGS_MODE_DISABLED: u16 = 0;

    /// Survey In
    pub const FLAGS_MODE_SURVEY_IN: u16 = 1;

    /// Fixed Mode (true ARP position required)
    pub const FLAGS_MODE_FIXED: u16 = 2;

    /// Position is given in LAT/LON/ALT
    /// (default is ECEF)
    pub const FLAGS_LLA: u16 = 256;

}


impl Default for CfgTMODE3 {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgTMODE3__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgTMODE3__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgTMODE3 {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgTMODE3__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgTMODE3__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgTMODE3__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgTMODE3 {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgTMODE3 where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgTMODE3";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgTMODE3() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgUSB() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__CfgUSB__init(msg: *mut CfgUSB) -> bool;
    fn ublox_msgs__msg__CfgUSB__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CfgUSB>, size: usize) -> bool;
    fn ublox_msgs__msg__CfgUSB__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CfgUSB>);
    fn ublox_msgs__msg__CfgUSB__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CfgUSB>, out_seq: *mut rosidl_runtime_rs::Sequence<CfgUSB>) -> bool;
}

// Corresponds to ublox_msgs__msg__CfgUSB
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// UBX-CFG-USB (0x06 0x1B)
/// USB Configuration

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgUSB {
    /// Only set to registered Vendor IDs.
    /// Changing this field requires special Host drivers.
    pub vendor_id: u16,

    /// Product ID. Changing this field requires special
    /// Host drivers.
    pub product_id: u16,

    /// Reserved
    pub reserved1: [u8; 2],

    /// Reserved
    pub reserved2: [u8; 2],

    /// Power consumed by the device
    pub power_consumption: u16,

    /// various configuration flags (see graphic below)
    pub flags: u16,

    /// String containing the vendor name.
    /// 32 ASCII bytes including 0-termination.
    pub vendor_string: [i8; 32],

    /// String containing the product name.
    /// 32 ASCII bytes including 0-termination.
    pub product_string: [i8; 32],

    /// String containing the serial number.
    /// 32 ASCII bytes including 0-termination.
    /// Changing the String fields requires special Host
    /// drivers.
    pub serial_number: [i8; 32],

}

impl CfgUSB {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 27;

    /// force re-enumeration
    pub const FLAGS_RE_ENUM: u16 = 0;

    /// self-powered (1), bus-powered (0)
    pub const FLAGS_POWER_MODE: u16 = 2;

}


impl Default for CfgUSB {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__CfgUSB__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__CfgUSB__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CfgUSB {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgUSB__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgUSB__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__CfgUSB__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CfgUSB {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CfgUSB where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/CfgUSB";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__CfgUSB() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfINS() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__EsfINS__init(msg: *mut EsfINS) -> bool;
    fn ublox_msgs__msg__EsfINS__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<EsfINS>, size: usize) -> bool;
    fn ublox_msgs__msg__EsfINS__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<EsfINS>);
    fn ublox_msgs__msg__EsfINS__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<EsfINS>, out_seq: *mut rosidl_runtime_rs::Sequence<EsfINS>) -> bool;
}

// Corresponds to ublox_msgs__msg__EsfINS
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// ESF-INS (0x10 0x15)
/// Vehicle dynamics information
///
/// This message outputs information about vehicle dynamics computed by the
/// Inertial Navigation System (INS) during ESF-based navigation.
/// For ADR products, the output dynamics information (angular rates and
/// accelerations) is expressed with respect to the vehicle-frame.
/// For UDR products, the output dynamics information (angular rates and
/// accelerations) is expressed with respect to the body-frame.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EsfINS {
    /// Bitfield (see graphic below)
    pub bitfield0: u32,

    /// Reserved
    pub reserved1: [u8; 4],

    /// GPS time of week of the navigation epoch
    pub i_tow: u32,

    /// Compensated x-axis angular rate
    pub x_ang_rate: i32,

    /// Compensated y-axis angular rate
    pub y_ang_rate: i32,

    /// Compensated z-axis angular rate
    pub z_ang_rate: i32,

    /// Compensated x-axis acceleration (gravity-free)
    pub x_accel: i32,

    /// Compensated y-axis acceleration (gravity-free)
    pub y_accel: i32,

    /// Compensated z-axis acceleration (gravity-free)
    pub z_accel: i32,

}

impl EsfINS {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 16;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 21;

    /// Message version (1 for this version).
    pub const BITFIELD0_VERSION: u32 = 255;

    /// Compensated x-axis angular rate data
    /// validity flag
    pub const BITFIELD0_X_ANG_RATE_VALID: u32 = 256;

    /// Compensated y-axis angular rate data
    /// validity flag
    pub const BITFIELD0_Y_ANG_RATE_VALID: u32 = 512;

    /// Compensated z-axis angular rate data
    /// validity flag
    pub const BITFIELD0_Z_ANG_RATE_VALID: u32 = 1024;

    /// Compensated x-axis acceleration data
    /// validity flag
    pub const BITFIELD0_X_ACCEL_VALID: u32 = 2048;

    /// Compensated y-axis acceleration data
    /// validity flag
    pub const BITFIELD0_Y_ACCEL_VALID: u32 = 4096;

    /// Compensated z-axis acceleration data
    /// validity flag
    pub const BITFIELD0_Z_ACCEL_VALID: u32 = 8192;

}


impl Default for EsfINS {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__EsfINS__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__EsfINS__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for EsfINS {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfINS__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfINS__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfINS__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for EsfINS {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for EsfINS where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/EsfINS";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfINS() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfMEAS() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__EsfMEAS__init(msg: *mut EsfMEAS) -> bool;
    fn ublox_msgs__msg__EsfMEAS__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<EsfMEAS>, size: usize) -> bool;
    fn ublox_msgs__msg__EsfMEAS__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<EsfMEAS>);
    fn ublox_msgs__msg__EsfMEAS__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<EsfMEAS>, out_seq: *mut rosidl_runtime_rs::Sequence<EsfMEAS>) -> bool;
}

// Corresponds to ublox_msgs__msg__EsfMEAS
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// ESF-MEAS (0x10 0x02)
/// External Sensor Fusion Measurements
///
/// Possible data types for the data field are described in the ESF Measurement
/// Data section

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EsfMEAS {
    /// Time tag of measurement generated by external
    /// sensor
    pub time_tag: u32,

    /// Flags. Set all unused bits to zero:
    pub flags: u16,

    /// Identification number of data provider
    pub id: u16,

    /// Start of repeated block (N times)
    /// data, see mask below
    pub data: rosidl_runtime_rs::Sequence<u32>,

    /// End of repeated block
    /// Start of optional block (size is either 0 or 1)
    /// Receiver local time calibrated.
    /// This field must not be supplied when
    /// calibTtagValid is set to 0
    pub calib_t_tag: rosidl_runtime_rs::Sequence<u32>,

}

impl EsfMEAS {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 16;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 2;

    /// Time mark signal was supplied just
    /// prior to sending this message: 0 =
    pub const FLAGS_TIME_MARK_SENT_MASK: u16 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TIME_MARK_NONE: u16 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TIME_MARK_EXT0: u16 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const TIME_MARK_EXT: u16 = 2;

    /// Trigger on rising (0) or falling
    /// (1) edge of time mark signal
    pub const FLAGS_TIME_MARK_EDGE: u16 = 4;

    /// Calibration time tag available.
    /// Always set to zero.
    pub const FLAGS_CALIB_T_TAG_VALID: u16 = 8;

    /// data
    pub const DATA_FIELD_MASK: u32 = 16777215;

    /// type of data (1..63)
    pub const DATA_TYPE_MASK: u32 = 1056964608;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DATA_TYPE_SHIFT: u32 = 24;

    /// data field contains no data
    pub const DATA_TYPE_NONE: u32 = 0;

    /// z-axis gyroscope angular rate
    /// [deg/s *2^-12 signed]
    pub const DATA_TYPE_Z_AXIS_GYRO: u32 = 5;

    /// front-left wheel ticks
    /// Bits 0-22: unsigned tick value.
    /// Bit 23: direction indicator
    /// (0=forward, 1=backward)
    pub const DATA_TYPE_WHEEL_TICKS_FRONT_LEFT: u32 = 6;

    /// front-right wheel ticks
    /// Bits 0-22: unsigned tick value.
    /// Bit 23: direction indicator
    /// (0=forward, 1=backward)
    pub const DATA_TYPE_WHEEL_TICKS_FRONT_RIGHT: u32 = 7;

    /// rear-left wheel ticks
    /// Bits 0-22: unsigned tick value.
    /// Bit 23: direction indicator
    /// (0=forward, 1=backward)
    pub const DATA_TYPE_WHEEL_TICKS_REAR_LEFT: u32 = 8;

    /// rear-right wheel ticks
    /// Bits 0-22: unsigned tick value.
    /// Bit 23: direction indicator
    /// (0=forward, 1=backward)
    pub const DATA_TYPE_WHEEL_TICKS_REAR_RIGHT: u32 = 9;

    /// single tick (speed tick)
    /// Bits 0-22: unsigned tick value.
    /// Bit 23: direction indicator
    /// (0=forward, 1=backward)
    pub const DATA_TYPE_SINGLE_TICK: u32 = 10;

    /// speed m/s * 1e-3 signed
    pub const DATA_TYPE_SPEED: u32 = 11;

    /// gyroscope temperature
    /// [deg Celsius * 1e-2 signed]
    pub const DATA_TYPE_GYRO_TEMPERATURE: u32 = 12;

    /// y-axis gyroscope angular rate
    /// [deg/s *2^-12 signed]
    pub const DATA_TYPE_GYRO_ANG_RATE_Y: u32 = 13;

    /// x-axis gyroscope angular rate
    /// [deg/s *2^-12 signed]
    pub const DATA_TYPE_GYRO_ANG_RATE_X: u32 = 14;

    /// x-axis accelerometer specific
    /// [force m/s^2 *2^-10 signed]
    pub const DATA_TYPE_ACCELEROMETER_X: u32 = 16;

    /// y-axis accelerometer specific
    /// [force m/s^2 *2^-10 signed]
    pub const DATA_TYPE_ACCELEROMETER_Y: u32 = 17;

    /// z-axis accelerometer specific
    /// [force m/s^2 *2^-10 signed]
    pub const DATA_TYPE_ACCELEROMETER_Z: u32 = 18;

}


impl Default for EsfMEAS {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__EsfMEAS__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__EsfMEAS__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for EsfMEAS {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfMEAS__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfMEAS__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfMEAS__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for EsfMEAS {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for EsfMEAS where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/EsfMEAS";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfMEAS() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfRAWBlock() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__EsfRAWBlock__init(msg: *mut EsfRAWBlock) -> bool;
    fn ublox_msgs__msg__EsfRAWBlock__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<EsfRAWBlock>, size: usize) -> bool;
    fn ublox_msgs__msg__EsfRAWBlock__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<EsfRAWBlock>);
    fn ublox_msgs__msg__EsfRAWBlock__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<EsfRAWBlock>, out_seq: *mut rosidl_runtime_rs::Sequence<EsfRAWBlock>) -> bool;
}

// Corresponds to ublox_msgs__msg__EsfRAWBlock
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// See ESF-RAW

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EsfRAWBlock {
    /// Its scaling and unit depends on the type and is
    /// the same as in ESF-MEAS
    pub data: u32,

    /// sensor time tag
    pub s_t_tag: u32,

}

impl EsfRAWBlock {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DATA_FIELD_MASK: u32 = 16777215;

    /// type of data
    /// (0 = no data; 1..255 = data type)
    pub const DATA_TYPE_MASK: u32 = 4278190080;

}


impl Default for EsfRAWBlock {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__EsfRAWBlock__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__EsfRAWBlock__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for EsfRAWBlock {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfRAWBlock__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfRAWBlock__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfRAWBlock__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for EsfRAWBlock {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for EsfRAWBlock where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/EsfRAWBlock";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfRAWBlock() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfRAW() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__EsfRAW__init(msg: *mut EsfRAW) -> bool;
    fn ublox_msgs__msg__EsfRAW__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<EsfRAW>, size: usize) -> bool;
    fn ublox_msgs__msg__EsfRAW__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<EsfRAW>);
    fn ublox_msgs__msg__EsfRAW__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<EsfRAW>, out_seq: *mut rosidl_runtime_rs::Sequence<EsfRAW>) -> bool;
}

// Corresponds to ublox_msgs__msg__EsfRAW
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// ESF-RAW (0x10 0x03)
/// Raw sensor measurements
///
/// The message contains measurements from the active inertial sensors connected
/// to the GNSS chip. Possible data types for the data field are accelerometer,
/// gyroscope and temperature readings as described in the ESF Measurement Data
/// section. Note that the rate selected in CFG-MSG is not respected. If a
/// positive rate is selected then all raw measurements will be output.
///
/// Supported on ADR/UDR products.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EsfRAW {
    /// Reserved
    pub reserved0: [u8; 4],


    // This member is not documented.
    #[allow(missing_docs)]
    pub blocks: rosidl_runtime_rs::Sequence<super::super::msg::rmw::EsfRAWBlock>,

}

impl EsfRAW {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 16;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 3;

}


impl Default for EsfRAW {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__EsfRAW__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__EsfRAW__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for EsfRAW {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfRAW__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfRAW__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfRAW__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for EsfRAW {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for EsfRAW where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/EsfRAW";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfRAW() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfSTATUS() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__EsfSTATUS__init(msg: *mut EsfSTATUS) -> bool;
    fn ublox_msgs__msg__EsfSTATUS__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<EsfSTATUS>, size: usize) -> bool;
    fn ublox_msgs__msg__EsfSTATUS__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<EsfSTATUS>);
    fn ublox_msgs__msg__EsfSTATUS__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<EsfSTATUS>, out_seq: *mut rosidl_runtime_rs::Sequence<EsfSTATUS>) -> bool;
}

// Corresponds to ublox_msgs__msg__EsfSTATUS
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// ESF-STATUS (0x10 0x10)
/// External Sensor Fusion (ESF) status information
///
/// Supported on UDR/ADR products

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EsfSTATUS {
    /// GPS time of week of the navigation epoch
    pub i_tow: u32,

    /// Message version (2 for this version)
    pub version: u8,

    /// Reserved
    pub reserved1: [u8; 7],

    /// Fusion mode:
    pub fusion_mode: u8,

    /// Reserved
    pub reserved2: [u8; 2],

    /// Number of sensors
    pub num_sens: u8,

    /// Start of repeated block (numSens times)
    pub sens: rosidl_runtime_rs::Sequence<super::super::msg::rmw::EsfSTATUSSens>,

}

impl EsfSTATUS {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 16;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 16;

    /// receiver is initializing some unknown values
    /// required for doing sensor fusion
    pub const FUSION_MODE_INIT: u8 = 0;

    /// GNSS and sensor data are
    /// used for navigation solution computation
    pub const FUSION_MODE_FUSION: u8 = 1;

    /// sensor fusion is temporarily disabled
    /// due to e.g. invalid sensor data or detected
    /// ferry
    pub const FUSION_MODE_SUSPENDED: u8 = 2;

    /// sensor fusion is permanently disabled
    /// until receiver reset due e.g. to sensor
    /// error
    pub const FUSION_MODE_DISABLED: u8 = 3;

}


impl Default for EsfSTATUS {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__EsfSTATUS__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__EsfSTATUS__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for EsfSTATUS {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfSTATUS__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfSTATUS__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfSTATUS__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for EsfSTATUS {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for EsfSTATUS where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/EsfSTATUS";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfSTATUS() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfSTATUSSens() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__EsfSTATUSSens__init(msg: *mut EsfSTATUSSens) -> bool;
    fn ublox_msgs__msg__EsfSTATUSSens__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<EsfSTATUSSens>, size: usize) -> bool;
    fn ublox_msgs__msg__EsfSTATUSSens__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<EsfSTATUSSens>);
    fn ublox_msgs__msg__EsfSTATUSSens__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<EsfSTATUSSens>, out_seq: *mut rosidl_runtime_rs::Sequence<EsfSTATUSSens>) -> bool;
}

// Corresponds to ublox_msgs__msg__EsfSTATUSSens
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// See Esf-STATUS

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EsfSTATUSSens {
    /// Sensor status, part 1 (see graphic below)
    pub sens_status1: u8,

    /// Sensor status, part 2 (see graphic below)
    pub sens_status2: u8,

    /// Observation frequency
    pub freq: u8,

    /// Sensor faults (see graphic below)
    pub faults: u8,

}



impl Default for EsfSTATUSSens {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__EsfSTATUSSens__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__EsfSTATUSSens__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for EsfSTATUSSens {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfSTATUSSens__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfSTATUSSens__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__EsfSTATUSSens__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for EsfSTATUSSens {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for EsfSTATUSSens where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/EsfSTATUSSens";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__EsfSTATUSSens() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__HnrPVT() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__HnrPVT__init(msg: *mut HnrPVT) -> bool;
    fn ublox_msgs__msg__HnrPVT__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<HnrPVT>, size: usize) -> bool;
    fn ublox_msgs__msg__HnrPVT__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<HnrPVT>);
    fn ublox_msgs__msg__HnrPVT__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<HnrPVT>, out_seq: *mut rosidl_runtime_rs::Sequence<HnrPVT>) -> bool;
}

// Corresponds to ublox_msgs__msg__HnrPVT
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// HNR-PVT (0x28 0x00)
/// High Rate Output of PVT Solution
///
/// Note that during a leap second there may be more (or less) than 60 seconds in
/// a minute; see the description of leap seconds for details.
///
/// This message provides the position, velocity and time solution with high
/// output rate.
///
/// Supported on ADR and UDR products.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct HnrPVT {
    /// GPS Millisecond time of week
    pub i_tow: u32,

    /// Year (UTC)
    pub year: u16,

    /// Month, range 1..12 (UTC)
    pub month: u8,

    /// Day of month, range 1..31 (UTC)
    pub day: u8,

    /// Hour of day, range 0..23 (UTC)
    pub hour: u8,

    /// Minute of hour, range 0..59 (UTC)
    pub min: u8,

    /// Seconds of minute, range 0..60 (UTC)
    pub sec: u8,

    /// Validity flags
    pub valid: u8,

    /// fraction of a second, range -1e9 .. 1e9 (UTC)
    pub nano: i32,

    /// GPS fix Type, range 0..5
    pub gps_fix: u8,

    /// Fix Status Flags
    pub flags: u8,

    /// Reserved
    pub reserved0: [u8; 2],

    /// Longitude
    pub lon: i32,

    /// Latitude
    pub lat: i32,

    /// Height above Ellipsoid
    pub height: i32,

    /// Height above mean sea level
    pub h_msl: i32,

    /// Ground Speed (2-D)
    pub g_speed: i32,

    /// Speed (3-D)
    pub speed: i32,

    /// Heading of motion (2-D)
    pub head_mot: i32,

    /// Heading of vehicle (2-D)
    pub head_veh: i32,

    /// Horizontal Accuracy Estimate
    pub h_acc: u32,

    /// Vertical Accuracy Estimate
    pub v_acc: u32,

    /// Speed Accuracy Estimate
    pub s_acc: u32,

    /// Heading Accuracy Estimate (both motion & vehicle)
    /// [deg / 1e-5]
    pub head_acc: u32,

    /// Reserved
    pub reserved1: [u8; 4],

}

impl HnrPVT {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 40;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 0;

    /// Valid UTC Date
    pub const VALID_DATE: u8 = 1;

    /// Valid
    pub const VALID_TIME: u8 = 2;

    /// UTC time of day has been fully resolved
    /// (no seconds uncertainty)
    pub const VALID_FULLY_RESOLVED: u8 = 4;

    /// Valid Magnetic Declination
    pub const VALID_MAG: u8 = 8;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FIX_TYPE_NO_FIX: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FIX_TYPE_DEAD_RECKONING_ONLY: u8 = 1;

    /// Signal from only 3 SVs,
    /// constant altitude assumed
    pub const FIX_TYPE_2D: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FIX_TYPE_3D: u8 = 3;

    /// GPS + Dead reckoning
    pub const FIX_TYPE_GPS_DEAD_RECKONING_COMBINED: u8 = 4;

    /// Time only fix
    pub const FIX_TYPE_TIME_ONLY: u8 = 5;

    /// i.e. within DOP & accuracy masks
    pub const FLAGS_GNSS_FIX_OK: u8 = 1;

    /// DGPS used
    pub const FLAGS_DIFF_SOLN: u8 = 2;

    /// Valid GPS week number
    pub const FLAGS_WKN_SET: u8 = 4;

    /// Valid GPS time of week (iTOW & fTOW)
    pub const FLAGS_TOW_SET: u8 = 8;

    /// heading of vehicle is valid
    pub const FLAGS_HEAD_VEH_VALID: u8 = 32;

}


impl Default for HnrPVT {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__HnrPVT__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__HnrPVT__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for HnrPVT {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__HnrPVT__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__HnrPVT__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__HnrPVT__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for HnrPVT {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for HnrPVT where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/HnrPVT";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__HnrPVT() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__Inf() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__Inf__init(msg: *mut Inf) -> bool;
    fn ublox_msgs__msg__Inf__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Inf>, size: usize) -> bool;
    fn ublox_msgs__msg__Inf__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Inf>);
    fn ublox_msgs__msg__Inf__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Inf>, out_seq: *mut rosidl_runtime_rs::Sequence<Inf>) -> bool;
}

// Corresponds to ublox_msgs__msg__Inf
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// UBX-INF (0x04, 0x00...0x04)
/// ASCII output
///
/// This message has a variable length payload, representing an ASCII string.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Inf {

    // This member is not documented.
    #[allow(missing_docs)]
    pub str: rosidl_runtime_rs::Sequence<u8>,

}

impl Inf {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 4;

}


impl Default for Inf {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__Inf__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__Inf__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Inf {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__Inf__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__Inf__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__Inf__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Inf {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Inf where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/Inf";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__Inf() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MgaGAL() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__MgaGAL__init(msg: *mut MgaGAL) -> bool;
    fn ublox_msgs__msg__MgaGAL__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MgaGAL>, size: usize) -> bool;
    fn ublox_msgs__msg__MgaGAL__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MgaGAL>);
    fn ublox_msgs__msg__MgaGAL__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MgaGAL>, out_seq: *mut rosidl_runtime_rs::Sequence<MgaGAL>) -> bool;
}

// Corresponds to ublox_msgs__msg__MgaGAL
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// MGA-GAL (0x13 0x02)
/// Galileo Ephemeris Assistance
///
/// This message allows the delivery of Galileo ephemeris assistance to a
/// receiver. See the description of AssistNow Online for details.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MgaGAL {
    /// Message type (0x01 for this type)
    pub type_: u8,

    /// Message version (0x00 for this version)
    pub version: u8,

    /// Galileo Satellite identifier
    pub svid: u8,

    /// Reserved
    pub reserved0: u8,

    /// Ephemeris and clock correction issue of Data
    pub iod_nav: u16,

    /// Mean motion difference from computed value
    /// [semi-cir cles/s * 2^-43]
    pub delta_n: i16,

    /// Mean anomaly at reference time
    pub m0: i32,

    /// Eccentricity
    pub e: u32,

    /// Square root of the semi-major axis
    pub sqrt_a: u32,

    /// Longitude of ascending node of orbital plane at weekly
    /// epoch
    pub omega0: i32,

    /// inclination angle at reference time
    /// [semi-cir cles 2^-31]
    pub i0: i32,

    /// Argument of perigee
    pub omega: i32,

    /// Rate of change of right ascension
    /// [semi-cir cles/s 2^-43]
    pub omega_dot: i32,

    /// Rate of change of inclination angle
    /// [semi-cir cles/s 2^-43]
    pub i_dot: i16,

    /// Amplitude of the cosine harmonic correction term to
    /// the argument of latitude
    pub cuc: i16,

    /// Amplitude of the sine harmonic correction term to
    /// the argument of latitude
    pub cus: i16,

    /// Amplitude of the cosine harmonic correction term
    /// to the orbit radius
    pub crc: i16,

    /// Amplitude of the sine harmonic correction term to the
    /// orbit radius
    pub crs: i16,

    /// Amplitude of the cosine harmonic correction term to
    /// the angle of inclination
    pub cic: i16,

    /// Amplitude of the sine harmonic correction term to the
    /// angle of inclination
    pub cis: i16,

    /// Ephemeris reference time
    pub toe: u16,

    /// clock bias correction coefficient
    pub af0: i32,

    /// SV clock drift correction coefficient
    pub af1: i32,

    /// SV clock drift rate correction coefficient
    /// [s/s^2 * 2^-59]
    pub af2: i8,

    /// Signal-in-Space Accuracy index for dual frequency
    /// E1-E5b
    pub sisaindex_e1_e5b: u8,

    /// Clock correction data reference Time of Week
    pub toc: u16,

    /// E1-E5b Broadcast Group Delay
    pub bgd_e1_e5b: i16,

    /// Reserved
    pub reserved1: [u8; 2],

    /// E1-B Signal Health Status
    pub health_e1b: u8,

    /// E1-B Data Validity Status
    pub data_validity_e1b: u8,

    /// E5b Signal Health Status
    pub health_e5b: u8,

    /// E5b Data Validity Status
    pub data_validity_e5b: u8,

    /// Reserved
    pub reserved2: [u8; 4],

}

impl MgaGAL {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 19;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 2;

}


impl Default for MgaGAL {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__MgaGAL__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__MgaGAL__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MgaGAL {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MgaGAL__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MgaGAL__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MgaGAL__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MgaGAL {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MgaGAL where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/MgaGAL";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MgaGAL() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonGNSS() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__MonGNSS__init(msg: *mut MonGNSS) -> bool;
    fn ublox_msgs__msg__MonGNSS__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MonGNSS>, size: usize) -> bool;
    fn ublox_msgs__msg__MonGNSS__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MonGNSS>);
    fn ublox_msgs__msg__MonGNSS__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MonGNSS>, out_seq: *mut rosidl_runtime_rs::Sequence<MonGNSS>) -> bool;
}

// Corresponds to ublox_msgs__msg__MonGNSS
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// MON-GNSS (0x0A 0x28)
/// Information message major GNSS selection
///
/// This message reports major GNSS selection. Augmentation systems are not
/// reported.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MonGNSS {
    /// Message version (0x01 for this version)
    pub version: u8,

    /// The major GNSS that can be supported by this receiver
    pub supported: u8,

    /// Default major GNSS selection. If the default major GNSS
    /// selection is currently configured in the efuse for this
    /// receiver, it takes precedence over the default major
    /// GNSS selection configured in the executing firmware of
    /// this receiver.
    /// see bit mask constants
    pub default_gnss: u8,

    /// Current major GNSS selection enabled for this receiver
    /// see bit mask constants
    pub enabled: u8,

    /// Maximum number of concurrent major GNSS that can be
    /// supported by this receiver
    pub simultaneous: u8,

    /// Reserved
    pub reserved1: [u8; 3],

}

impl MonGNSS {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 10;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 40;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const BIT_MASK_GPS: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const BIT_MASK_GLONASS: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const BIT_MASK_BEIDOU: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const BIT_MASK_GALILEO: u8 = 8;

}


impl Default for MonGNSS {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__MonGNSS__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__MonGNSS__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MonGNSS {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonGNSS__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonGNSS__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonGNSS__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MonGNSS {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MonGNSS where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/MonGNSS";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonGNSS() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonHW6() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__MonHW6__init(msg: *mut MonHW6) -> bool;
    fn ublox_msgs__msg__MonHW6__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MonHW6>, size: usize) -> bool;
    fn ublox_msgs__msg__MonHW6__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MonHW6>);
    fn ublox_msgs__msg__MonHW6__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MonHW6>, out_seq: *mut rosidl_runtime_rs::Sequence<MonHW6>) -> bool;
}

// Corresponds to ublox_msgs__msg__MonHW6
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// MON-HW (0x0A 0x09)
/// Hardware Status
/// Firmware 6
///
/// Status of different aspect of the hardware, such as Antenna, PIO/Peripheral
/// Pins, Noise Level, Automatic Gain Control (AGC)
///
/// WARNING: this message is a different length than the MonHW message for
/// firmware version 7 & 8

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MonHW6 {
    /// Mask of Pins Set as Peripheral/PIO
    pub pin_sel: u32,

    /// Mask of Pins Set as Bank A/B
    pub pin_bank: u32,

    /// Mask of Pins Set as Input/Output
    pub pin_dir: u32,

    /// Mask of Pins Value Low/High
    pub pin_val: u32,

    /// Noise Level as measured by the GPS Core
    pub noise_per_ms: u16,

    /// AGC Monitor (counts SIGHI xor SIGLO,
    /// range 0 to 8191)
    pub agc_cnt: u16,

    /// Status of the Antenna Supervisor State Machine
    pub a_status: u8,

    /// Current PowerStatus of Antenna
    pub a_power: u8,

    /// Flags:
    pub flags: u8,

    /// Reserved
    pub reserved0: u8,

    /// Mask of Pins that are used by the Virtual Pin
    /// Manager
    pub used_mask: u32,

    /// Array of Pin Mappings for each of the 25
    /// Physical Pins
    pub vp: [u8; 25],

    /// CW Jamming indicator, scaled:
    pub jam_ind: u8,

    /// Reserved
    pub reserved1: [u8; 2],

    /// Mask of Pins Value using the PIO Irq
    pub pin_irq: u32,

    /// Mask of Pins Value using the PIO Pull High
    /// Resistor
    pub pull_h: u32,

    /// Mask of Pins Value using the PIO Pull Low
    /// Resistor
    pub pull_l: u32,

}

impl MonHW6 {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 10;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 9;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_STATUS_INIT: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_STATUS_UNKNOWN: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_STATUS_OK: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_STATUS_SHORT: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_STATUS_OPEN: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_POWER_OFF: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_POWER_ON: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_POWER_UNKNOWN: u8 = 2;

    /// RTC is calibrated
    pub const FLAGS_RTC_CALIB: u8 = 1;

    /// Safe boot mode (0 = inactive, 1 = active)
    pub const FLAGS_SAFE_BOOT: u8 = 2;

    /// output from Jamming/Interference Monitor:
    pub const FLAGS_JAMMING_STATE_MASK: u8 = 12;

    /// unknown or feature disabled
    pub const JAMMING_STATE_UNKNOWN_OR_DISABLED: u8 = 0;

    /// ok - no significant jamming
    pub const JAMMING_STATE_OK: u8 = 4;

    /// interference visible but fix OK
    pub const JAMMING_STATE_WARNING: u8 = 8;

    /// interference visible and no fix
    pub const JAMMING_STATE_CRITICAL: u8 = 12;

    /// RTC XTAL is absent
    /// (not supported in protocol versions < 18)
    pub const FLAGS_XTAL_ABSENT: u8 = 16;

    /// No CW Jamming
    pub const JAM_IND_NONE: u8 = 0;

    /// Strong CW Jamming
    pub const JAM_IND_STRONG: u8 = 255;

}


impl Default for MonHW6 {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__MonHW6__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__MonHW6__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MonHW6 {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonHW6__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonHW6__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonHW6__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MonHW6 {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MonHW6 where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/MonHW6";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonHW6() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonHW() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__MonHW__init(msg: *mut MonHW) -> bool;
    fn ublox_msgs__msg__MonHW__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MonHW>, size: usize) -> bool;
    fn ublox_msgs__msg__MonHW__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MonHW>);
    fn ublox_msgs__msg__MonHW__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MonHW>, out_seq: *mut rosidl_runtime_rs::Sequence<MonHW>) -> bool;
}

// Corresponds to ublox_msgs__msg__MonHW
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// MON-HW (0x0A 0x09)
/// Hardware Status
///
/// Status of different aspect of the hardware, such as Antenna, PIO/Peripheral
/// Pins, Noise Level, Automatic Gain Control (AGC)
///
/// WARNING: this message is a different length than the MonHW message for
/// firmware version 6

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MonHW {
    /// Mask of Pins Set as Peripheral/PIO
    pub pin_sel: u32,

    /// Mask of Pins Set as Bank A/B
    pub pin_bank: u32,

    /// Mask of Pins Set as Input/Output
    pub pin_dir: u32,

    /// Mask of Pins Value Low/High
    pub pin_val: u32,

    /// Noise Level as measured by the GPS Core
    pub noise_per_ms: u16,

    /// AGC Monitor (counts SIGHI xor SIGLO,
    /// range 0 to 8191)
    pub agc_cnt: u16,

    /// Status of the Antenna Supervisor State Machine
    pub a_status: u8,

    /// Current PowerStatus of Antenna
    pub a_power: u8,

    /// Flags:
    pub flags: u8,

    /// Reserved
    pub reserved0: u8,

    /// Mask of Pins that are used by the Virtual Pin
    /// Manager
    pub used_mask: u32,

    /// Array of Pin Mappings for each of the 17
    /// Physical Pins
    pub vp: [u8; 17],

    /// CW Jamming indicator, scaled:
    pub jam_ind: u8,

    /// Reserved
    pub reserved1: [u8; 2],

    /// Mask of Pins Value using the PIO Irq
    pub pin_irq: u32,

    /// Mask of Pins Value using the PIO Pull High
    /// Resistor
    pub pull_h: u32,

    /// Mask of Pins Value using the PIO Pull Low
    /// Resistor
    pub pull_l: u32,

}

impl MonHW {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 10;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 9;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_STATUS_INIT: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_STATUS_UNKNOWN: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_STATUS_OK: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_STATUS_SHORT: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_STATUS_OPEN: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_POWER_OFF: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_POWER_ON: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const A_POWER_UNKNOWN: u8 = 2;

    /// RTC is calibrated
    pub const FLAGS_RTC_CALIB: u8 = 1;

    /// Safe boot mode (0 = inactive, 1 = active)
    pub const FLAGS_SAFE_BOOT: u8 = 2;

    /// output from Jamming/Interference Monitor:
    pub const FLAGS_JAMMING_STATE_MASK: u8 = 12;

    /// unknown or feature disabled
    pub const JAMMING_STATE_UNKNOWN_OR_DISABLED: u8 = 0;

    /// ok - no significant jamming
    pub const JAMMING_STATE_OK: u8 = 4;

    /// interference visible but fix OK
    pub const JAMMING_STATE_WARNING: u8 = 8;

    /// interference visible and no fix
    pub const JAMMING_STATE_CRITICAL: u8 = 12;

    /// RTC XTAL is absent
    /// (not supported in protocol versions < 18)
    pub const FLAGS_XTAL_ABSENT: u8 = 16;

    /// No CW Jamming
    pub const JAM_IND_NONE: u8 = 0;

    /// Strong CW Jamming
    pub const JAM_IND_STRONG: u8 = 255;

}


impl Default for MonHW {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__MonHW__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__MonHW__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MonHW {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonHW__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonHW__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonHW__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MonHW {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MonHW where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/MonHW";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonHW() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonVERExtension() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__MonVERExtension__init(msg: *mut MonVERExtension) -> bool;
    fn ublox_msgs__msg__MonVERExtension__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MonVERExtension>, size: usize) -> bool;
    fn ublox_msgs__msg__MonVERExtension__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MonVERExtension>);
    fn ublox_msgs__msg__MonVERExtension__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MonVERExtension>, out_seq: *mut rosidl_runtime_rs::Sequence<MonVERExtension>) -> bool;
}

// Corresponds to ublox_msgs__msg__MonVERExtension
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// see MonVER message

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MonVERExtension {

    // This member is not documented.
    #[allow(missing_docs)]
    pub field: [u8; 30],

}



impl Default for MonVERExtension {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__MonVERExtension__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__MonVERExtension__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MonVERExtension {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonVERExtension__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonVERExtension__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonVERExtension__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MonVERExtension {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MonVERExtension where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/MonVERExtension";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonVERExtension() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonVER() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__MonVER__init(msg: *mut MonVER) -> bool;
    fn ublox_msgs__msg__MonVER__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MonVER>, size: usize) -> bool;
    fn ublox_msgs__msg__MonVER__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MonVER>);
    fn ublox_msgs__msg__MonVER__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MonVER>, out_seq: *mut rosidl_runtime_rs::Sequence<MonVER>) -> bool;
}

// Corresponds to ublox_msgs__msg__MonVER
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// MON-VER (0x0A 0x04)
///
/// Receiver/Software Version
/// Returned when the version is polled.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MonVER {
    /// Zero-terminated software version string.
    pub sw_version: [u8; 30],

    /// Zero-terminated hardware version string.
    pub hw_version: [u8; 10],

    /// Start of repeated block (N times)
    pub extension: rosidl_runtime_rs::Sequence<super::super::msg::rmw::MonVERExtension>,

}

impl MonVER {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 10;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 4;

}


impl Default for MonVER {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__MonVER__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__MonVER__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MonVER {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonVER__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonVER__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__MonVER__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MonVER {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MonVER where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/MonVER";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__MonVER() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavATT() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavATT__init(msg: *mut NavATT) -> bool;
    fn ublox_msgs__msg__NavATT__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavATT>, size: usize) -> bool;
    fn ublox_msgs__msg__NavATT__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavATT>);
    fn ublox_msgs__msg__NavATT__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavATT>, out_seq: *mut rosidl_runtime_rs::Sequence<NavATT>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavATT
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-ATT (0x01 0x05)
/// Attitude Solution
///
/// This message outputs the attitude solution as roll, pitch and heading angles.
/// Supported on ADR and UDR products.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavATT {
    /// GPS time of week of the navigation epoch
    pub i_tow: u32,

    /// Message version (0 for this version)
    pub version: u8,

    /// Reserved
    pub reserved0: [u8; 3],

    /// Vehicle roll.
    pub roll: i32,

    /// Vehicle pitch.
    pub pitch: i32,

    /// Vehicle heading.
    pub heading: i32,

    /// Vehicle roll accuracy (if null, roll angle is not
    /// available).
    pub acc_roll: u32,

    /// Vehicle pitch accuracy (if null, pitch angle is not
    /// available).
    pub acc_pitch: u32,

    /// Vehicle heading accuracy
    pub acc_heading: u32,

}

impl NavATT {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 5;

}


impl Default for NavATT {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavATT__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavATT__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavATT {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavATT__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavATT__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavATT__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavATT {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavATT where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavATT";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavATT() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavCLOCK() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavCLOCK__init(msg: *mut NavCLOCK) -> bool;
    fn ublox_msgs__msg__NavCLOCK__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavCLOCK>, size: usize) -> bool;
    fn ublox_msgs__msg__NavCLOCK__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavCLOCK>);
    fn ublox_msgs__msg__NavCLOCK__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavCLOCK>, out_seq: *mut rosidl_runtime_rs::Sequence<NavCLOCK>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavCLOCK
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-CLOCK (0x01 0x22)
/// Clock Solution

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavCLOCK {
    /// GPS Millisecond Time of week
    pub i_tow: u32,

    /// Clock bias in nanoseconds
    pub clk_b: i32,

    /// Clock drift in nanoseconds per second
    pub clk_d: i32,

    /// Time Accuracy Estimate
    pub t_acc: u32,

    /// Frequency Accuracy Estimate
    pub f_acc: u32,

}

impl NavCLOCK {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 34;

}


impl Default for NavCLOCK {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavCLOCK__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavCLOCK__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavCLOCK {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavCLOCK__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavCLOCK__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavCLOCK__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavCLOCK {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavCLOCK where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavCLOCK";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavCLOCK() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavCOV() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavCOV__init(msg: *mut NavCOV) -> bool;
    fn ublox_msgs__msg__NavCOV__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavCOV>, size: usize) -> bool;
    fn ublox_msgs__msg__NavCOV__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavCOV>);
    fn ublox_msgs__msg__NavCOV__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavCOV>, out_seq: *mut rosidl_runtime_rs::Sequence<NavCOV>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavCOV
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-COV (0x01 0x36)
/// Covariance matrices for position and velocity solutions

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavCOV {
    /// GPS Millisecond time of week
    pub i_tow: u32,

    /// Message version (0x00 for this version)
    pub version: u8,

    /// Position covariance matrix validity flag
    pub pos_cov_valid: u8,

    /// Velocity covariance matrix validity flag
    pub vel_cov_valid: u8,

    /// Reserved
    pub reserved_0: [u8; 9],

    /// Position covariance matrix value p_NN
    pub pos_cov_nn: f32,

    /// Position covariance matrix value p_NE
    pub pos_cov_ne: f32,

    /// Position covariance matrix value p_ND
    pub pos_cov_nd: f32,

    /// Position covariance matrix value p_EE
    pub pos_cov_ee: f32,

    /// Position covariance matrix value p_ED
    pub pos_cov_ed: f32,

    /// Position covariance matrix value p_DD
    pub pos_cov_dd: f32,

    /// Velocity covariance matrix value v_NN
    pub vel_cov_nn: f32,

    /// Velocity covariance matrix value v_NE
    pub vel_cov_ne: f32,

    /// Velocity covariance matrix value v_ND
    pub vel_cov_nd: f32,

    /// Velocity covariance matrix value v_EE
    pub vel_cov_ee: f32,

    /// Velocity covariance matrix value v_ED
    pub vel_cov_ed: f32,

    /// Velocity covariance matrix value v_DD
    pub vel_cov_dd: f32,

}

impl NavCOV {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 54;

}


impl Default for NavCOV {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavCOV__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavCOV__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavCOV {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavCOV__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavCOV__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavCOV__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavCOV {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavCOV where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavCOV";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavCOV() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavDGPS() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavDGPS__init(msg: *mut NavDGPS) -> bool;
    fn ublox_msgs__msg__NavDGPS__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavDGPS>, size: usize) -> bool;
    fn ublox_msgs__msg__NavDGPS__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavDGPS>);
    fn ublox_msgs__msg__NavDGPS__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavDGPS>, out_seq: *mut rosidl_runtime_rs::Sequence<NavDGPS>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavDGPS
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-DGPS (0x01 0x31)
/// DGPS Data Used for NAV
///
/// This message outputs the Correction data as it has been applied to the current
/// NAV Solution. See also the notes on the RTCM protocol.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavDGPS {
    /// GPS Millisecond time of week
    pub i_tow: u32,

    /// Age of newest correction data
    pub age: i32,

    /// DGPS Base Station ID
    pub base_id: i16,

    /// DGPS Base Station Health Status
    pub base_health: i16,

    /// Number of channels for which correction data is
    /// following
    pub num_ch: i8,

    /// DGPS Correction Type Status
    pub status: u8,

    /// Reserved
    pub reserved1: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sv: rosidl_runtime_rs::Sequence<super::super::msg::rmw::NavDGPSSV>,

}

impl NavDGPS {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 49;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DGPS_CORRECTION_NONE: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const DGPS_CORRECTION_PR_PRR: u8 = 1;

}


impl Default for NavDGPS {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavDGPS__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavDGPS__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavDGPS {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavDGPS__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavDGPS__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavDGPS__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavDGPS {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavDGPS where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavDGPS";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavDGPS() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavDGPSSV() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavDGPSSV__init(msg: *mut NavDGPSSV) -> bool;
    fn ublox_msgs__msg__NavDGPSSV__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavDGPSSV>, size: usize) -> bool;
    fn ublox_msgs__msg__NavDGPSSV__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavDGPSSV>);
    fn ublox_msgs__msg__NavDGPSSV__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavDGPSSV>, out_seq: *mut rosidl_runtime_rs::Sequence<NavDGPSSV>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavDGPSSV
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// see message NavDGPS

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavDGPSSV {
    /// Satellite ID
    pub svid: u8,

    /// Bitmask / Channel Number and Usage:
    pub flags: u8,

    /// Age of latest correction data
    pub age_c: u16,

    /// Pseudo Range Correction
    pub prc: f32,

    /// Pseudo Range Rate Correction
    pub prrc: f32,

}

impl NavDGPSSV {
    /// Bitmask for channel number, range 0..15
    /// Channel numbers > 15 marked as 15
    pub const FLAGS_CHANNEL_MASK: u8 = 15;

    /// DGPS Used for this SV
    pub const FLAGS_DGPS: u8 = 16;

}


impl Default for NavDGPSSV {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavDGPSSV__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavDGPSSV__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavDGPSSV {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavDGPSSV__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavDGPSSV__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavDGPSSV__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavDGPSSV {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavDGPSSV where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavDGPSSV";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavDGPSSV() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavDOP() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavDOP__init(msg: *mut NavDOP) -> bool;
    fn ublox_msgs__msg__NavDOP__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavDOP>, size: usize) -> bool;
    fn ublox_msgs__msg__NavDOP__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavDOP>);
    fn ublox_msgs__msg__NavDOP__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavDOP>, out_seq: *mut rosidl_runtime_rs::Sequence<NavDOP>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavDOP
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-DOP (0x01 0x04)
/// Dilution of precision
///
/// - DOP values are dimensionless.
/// - All DOP values are scaled by a factor of 100. If the unit transmits a value
///   of e.g. 156, the DOP value is 1.56.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavDOP {
    /// GPS Millisecond Time of Week
    pub i_tow: u32,

    /// Geometric DOP
    pub g_dop: u16,

    /// Position DOP
    pub p_dop: u16,

    /// Time DOP
    pub t_dop: u16,

    /// Vertical DOP
    pub v_dop: u16,

    /// Horizontal DOP
    pub h_dop: u16,

    /// Northing DOP
    pub n_dop: u16,

    /// Easting DOP
    pub e_dop: u16,

}

impl NavDOP {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 4;

}


impl Default for NavDOP {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavDOP__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavDOP__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavDOP {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavDOP__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavDOP__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavDOP__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavDOP {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavDOP where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavDOP";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavDOP() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavPOSECEF() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavPOSECEF__init(msg: *mut NavPOSECEF) -> bool;
    fn ublox_msgs__msg__NavPOSECEF__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavPOSECEF>, size: usize) -> bool;
    fn ublox_msgs__msg__NavPOSECEF__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavPOSECEF>);
    fn ublox_msgs__msg__NavPOSECEF__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavPOSECEF>, out_seq: *mut rosidl_runtime_rs::Sequence<NavPOSECEF>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavPOSECEF
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-POSECEF (0x01 0x01)
/// Position Solution in ECEF
///
/// See important comments concerning validity of position given in section
/// Navigation Output Filters.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavPOSECEF {
    /// GPS Millisecond Time of Week
    pub i_tow: u32,

    /// ECEF X coordinate
    pub ecef_x: i32,

    /// ECEF Y coordinate
    pub ecef_y: i32,

    /// ECEF Z coordinate
    pub ecef_z: i32,

    /// Position Accuracy Estimate
    pub p_acc: u32,

}

impl NavPOSECEF {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 1;

}


impl Default for NavPOSECEF {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavPOSECEF__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavPOSECEF__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavPOSECEF {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavPOSECEF__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavPOSECEF__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavPOSECEF__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavPOSECEF {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavPOSECEF where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavPOSECEF";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavPOSECEF() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavPOSLLH() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavPOSLLH__init(msg: *mut NavPOSLLH) -> bool;
    fn ublox_msgs__msg__NavPOSLLH__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavPOSLLH>, size: usize) -> bool;
    fn ublox_msgs__msg__NavPOSLLH__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavPOSLLH>);
    fn ublox_msgs__msg__NavPOSLLH__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavPOSLLH>, out_seq: *mut rosidl_runtime_rs::Sequence<NavPOSLLH>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavPOSLLH
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-POSLLH (0x01 0x02)
/// Geodetic Position Solution
///
/// See important comments concerning validity of position given in section
/// Navigation Output Filters.
/// This message outputs the Geodetic position in the currently selected
/// Ellipsoid. The default is the WGS84 Ellipsoid, but can be changed with the
/// message CFG-DAT.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavPOSLLH {
    /// GPS Millisecond Time of Week
    pub i_tow: u32,

    /// Longitude
    pub lon: i32,

    /// Latitude
    pub lat: i32,

    /// Height above Ellipsoid
    pub height: i32,

    /// Height above mean sea level
    pub h_msl: i32,

    /// Horizontal Accuracy Estimate
    pub h_acc: u32,

    /// Vertical Accuracy Estimate
    pub v_acc: u32,

}

impl NavPOSLLH {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 2;

}


impl Default for NavPOSLLH {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavPOSLLH__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavPOSLLH__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavPOSLLH {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavPOSLLH__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavPOSLLH__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavPOSLLH__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavPOSLLH {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavPOSLLH where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavPOSLLH";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavPOSLLH() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavPVT7() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavPVT7__init(msg: *mut NavPVT7) -> bool;
    fn ublox_msgs__msg__NavPVT7__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavPVT7>, size: usize) -> bool;
    fn ublox_msgs__msg__NavPVT7__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavPVT7>);
    fn ublox_msgs__msg__NavPVT7__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavPVT7>, out_seq: *mut rosidl_runtime_rs::Sequence<NavPVT7>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavPVT7
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-PVT (0x01 0x07)
/// Navigation Position Velocity Time Solution Firmware version 7
///
/// Note that during a leap second there may be more (or less) than 60 seconds in
/// a minute; see the description of leap seconds for details.
///
/// This message combines Position, velocity and time solution in LLH,
/// including accuracy figures
///
/// WARNING: For firmware version 7, this message is a different length.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavPVT7 {
    /// GPS Millisecond time of week
    pub i_tow: u32,

    /// Year (UTC)
    pub year: u16,

    /// Month, range 1..12 (UTC)
    pub month: u8,

    /// Day of month, range 1..31 (UTC)
    pub day: u8,

    /// Hour of day, range 0..23 (UTC)
    pub hour: u8,

    /// Minute of hour, range 0..59 (UTC)
    pub min: u8,

    /// Seconds of minute, range 0..60 (UTC)
    pub sec: u8,

    /// Validity flags
    pub valid: u8,

    /// time accuracy estimate (UTC)
    pub t_acc: u32,

    /// fraction of a second, range -1e9 .. 1e9 (UTC)
    pub nano: i32,

    /// GNSS fix Type, range 0..5
    pub fix_type: u8,

    /// Fix Status Flags
    pub flags: u8,

    /// Additional Flags
    pub flags2: u8,

    /// Number of SVs used in Nav Solution
    pub num_sv: u8,

    /// Longitude
    pub lon: i32,

    /// Latitude
    pub lat: i32,

    /// Height above Ellipsoid
    pub height: i32,

    /// Height above mean sea level
    pub h_msl: i32,

    /// Horizontal Accuracy Estimate
    pub h_acc: u32,

    /// Vertical Accuracy Estimate
    pub v_acc: u32,

    /// NED north velocity
    pub vel_n: i32,

    /// NED east velocity
    pub vel_e: i32,

    /// NED down velocity
    pub vel_d: i32,

    /// Ground Speed (2-D)
    pub g_speed: i32,

    /// Heading of motion 2-D
    pub heading: i32,

    /// Speed Accuracy Estimate
    pub s_acc: u32,

    /// Heading Accuracy Estimate (both motion & vehicle)
    /// [deg / 1e-5]
    pub head_acc: u32,

    /// Position DOP
    pub p_dop: u16,

    /// Reserved
    pub reserved1: [u8; 6],

}

impl NavPVT7 {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 7;

    /// Valid UTC Date
    pub const VALID_DATE: u8 = 1;

    /// Valid
    pub const VALID_TIME: u8 = 2;

    /// UTC time of day has been fully resolved
    /// (no seconds uncertainty)
    pub const VALID_FULLY_RESOLVED: u8 = 4;

    /// Valid Magnetic Declination
    pub const VALID_MAG: u8 = 8;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FIX_TYPE_NO_FIX: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FIX_TYPE_DEAD_RECKONING_ONLY: u8 = 1;

    /// Signal from only 3 SVs,
    /// constant altitude assumed
    pub const FIX_TYPE_2D: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FIX_TYPE_3D: u8 = 3;

    /// GNSS + Dead reckoning
    pub const FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED: u8 = 4;

    /// Time only fix (High precision
    /// devices)
    pub const FIX_TYPE_TIME_ONLY: u8 = 5;

    /// i.e. within DOP & accuracy masks
    pub const FLAGS_GNSS_FIX_OK: u8 = 1;

    /// DGPS used
    pub const FLAGS_DIFF_SOLN: u8 = 2;

    /// Power Save Mode
    pub const FLAGS_PSM_MASK: u8 = 28;

    /// PSM is off
    pub const PSM_OFF: u8 = 0;

    /// Enabled (state before acquisition)
    pub const PSM_ENABLED: u8 = 4;

    /// Acquisition
    pub const PSM_ACQUIRED: u8 = 8;

    /// Tracking
    pub const PSM_TRACKING: u8 = 12;

    /// Power Optimized Tracking
    pub const PSM_POWER_OPTIMIZED_TRACKING: u8 = 16;

    /// Inactive
    pub const PSM_INACTIVE: u8 = 20;

    /// heading of vehicle is valid
    pub const FLAGS_HEAD_VEH_VALID: u8 = 32;

    /// Carrier Phase Range Solution Status
    pub const FLAGS_CARRIER_PHASE_MASK: u8 = 192;

    /// no carrier phase range solution
    pub const CARRIER_PHASE_NO_SOLUTION: u8 = 0;

    /// carrier phase float solution (no fixed
    /// integer measurements have been used to
    /// calculate the solution)
    pub const CARRIER_PHASE_FLOAT: u8 = 64;

    /// fixed solution (>=1 fixed integer
    /// carrier phase range measurements have
    /// been used to calculate  the solution)
    pub const CARRIER_PHASE_FIXED: u8 = 128;

    /// information about UTC Date and Time of
    /// Day validity confirmation is available
    pub const FLAGS2_CONFIRMED_AVAILABLE: u8 = 32;

    /// UTC Date validity could be confirmed
    pub const FLAGS2_CONFIRMED_DATE: u8 = 64;

    /// UTC Time of Day could be confirmed
    pub const FLAGS2_CONFIRMED_TIME: u8 = 128;

}


impl Default for NavPVT7 {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavPVT7__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavPVT7__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavPVT7 {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavPVT7__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavPVT7__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavPVT7__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavPVT7 {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavPVT7 where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavPVT7";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavPVT7() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavPVT() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavPVT__init(msg: *mut NavPVT) -> bool;
    fn ublox_msgs__msg__NavPVT__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavPVT>, size: usize) -> bool;
    fn ublox_msgs__msg__NavPVT__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavPVT>);
    fn ublox_msgs__msg__NavPVT__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavPVT>, out_seq: *mut rosidl_runtime_rs::Sequence<NavPVT>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavPVT
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-PVT (0x01 0x07)
/// Navigation Position Velocity Time Solution
///
/// Note that during a leap second there may be more (or less) than 60 seconds in
/// a minute; see the description of leap seconds for details.
///
/// This message combines Position, velocity and time solution in LLH,
/// including accuracy figures
///
/// WARNING: For firmware version 7, this message is a different length.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavPVT {
    /// GPS Millisecond time of week
    pub i_tow: u32,

    /// Year (UTC)
    pub year: u16,

    /// Month, range 1..12 (UTC)
    pub month: u8,

    /// Day of month, range 1..31 (UTC)
    pub day: u8,

    /// Hour of day, range 0..23 (UTC)
    pub hour: u8,

    /// Minute of hour, range 0..59 (UTC)
    pub min: u8,

    /// Seconds of minute, range 0..60 (UTC)
    pub sec: u8,

    /// Validity flags
    pub valid: u8,

    /// time accuracy estimate (UTC)
    pub t_acc: u32,

    /// fraction of a second, range -1e9 .. 1e9 (UTC)
    pub nano: i32,

    /// GNSS fix Type, range 0..5
    pub fix_type: u8,

    /// Fix Status Flags
    pub flags: u8,

    /// Additional Flags
    pub flags2: u8,

    /// Number of SVs used in Nav Solution
    pub num_sv: u8,

    /// Longitude
    pub lon: i32,

    /// Latitude
    pub lat: i32,

    /// Height above Ellipsoid
    pub height: i32,

    /// Height above mean sea level
    pub h_msl: i32,

    /// Horizontal Accuracy Estimate
    pub h_acc: u32,

    /// Vertical Accuracy Estimate
    pub v_acc: u32,

    /// NED north velocity
    pub vel_n: i32,

    /// NED east velocity
    pub vel_e: i32,

    /// NED down velocity
    pub vel_d: i32,

    /// Ground Speed (2-D)
    pub g_speed: i32,

    /// Heading of motion 2-D
    pub heading: i32,

    /// Speed Accuracy Estimate
    pub s_acc: u32,

    /// Heading Accuracy Estimate (both motion & vehicle)
    /// [deg / 1e-5]
    pub head_acc: u32,

    /// Position DOP
    pub p_dop: u16,

    /// Reserved
    pub reserved1: [u8; 6],

    /// Heading of vehicle (2-D)
    pub head_veh: i32,

    /// Magnetic declination
    pub mag_dec: i16,

    /// Magnetic declination accuracy
    pub mag_acc: u16,

}

impl NavPVT {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 7;

    /// Valid UTC Date
    pub const VALID_DATE: u8 = 1;

    /// Valid
    pub const VALID_TIME: u8 = 2;

    /// UTC time of day has been fully resolved
    /// (no seconds uncertainty)
    pub const VALID_FULLY_RESOLVED: u8 = 4;

    /// Valid Magnetic Declination
    pub const VALID_MAG: u8 = 8;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FIX_TYPE_NO_FIX: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FIX_TYPE_DEAD_RECKONING_ONLY: u8 = 1;

    /// Signal from only 3 SVs,
    /// constant altitude assumed
    pub const FIX_TYPE_2D: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const FIX_TYPE_3D: u8 = 3;

    /// GNSS + Dead reckoning
    pub const FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED: u8 = 4;

    /// Time only fix (High precision
    /// devices)
    pub const FIX_TYPE_TIME_ONLY: u8 = 5;

    /// i.e. within DOP & accuracy masks
    pub const FLAGS_GNSS_FIX_OK: u8 = 1;

    /// DGPS used
    pub const FLAGS_DIFF_SOLN: u8 = 2;

    /// Power Save Mode
    pub const FLAGS_PSM_MASK: u8 = 28;

    /// PSM is off
    pub const PSM_OFF: u8 = 0;

    /// Enabled (state before acquisition)
    pub const PSM_ENABLED: u8 = 4;

    /// Acquisition
    pub const PSM_ACQUIRED: u8 = 8;

    /// Tracking
    pub const PSM_TRACKING: u8 = 12;

    /// Power Optimized Tracking
    pub const PSM_POWER_OPTIMIZED_TRACKING: u8 = 16;

    /// Inactive
    pub const PSM_INACTIVE: u8 = 20;

    /// heading of vehicle is valid
    pub const FLAGS_HEAD_VEH_VALID: u8 = 32;

    /// Carrier Phase Range Solution Status
    pub const FLAGS_CARRIER_PHASE_MASK: u8 = 192;

    /// no carrier phase range solution
    pub const CARRIER_PHASE_NO_SOLUTION: u8 = 0;

    /// carrier phase float solution (no fixed
    /// integer measurements have been used to
    /// calculate the solution)
    pub const CARRIER_PHASE_FLOAT: u8 = 64;

    /// fixed solution (>=1 fixed integer
    /// carrier phase range measurements have
    /// been used to calculate  the solution)
    pub const CARRIER_PHASE_FIXED: u8 = 128;

    /// information about UTC Date and Time of
    /// Day validity confirmation is available
    pub const FLAGS2_CONFIRMED_AVAILABLE: u8 = 32;

    /// UTC Date validity could be confirmed
    pub const FLAGS2_CONFIRMED_DATE: u8 = 64;

    /// UTC Time of Day could be confirmed
    pub const FLAGS2_CONFIRMED_TIME: u8 = 128;

}


impl Default for NavPVT {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavPVT__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavPVT__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavPVT {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavPVT__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavPVT__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavPVT__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavPVT {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavPVT where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavPVT";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavPVT() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavRELPOSNED9() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavRELPOSNED9__init(msg: *mut NavRELPOSNED9) -> bool;
    fn ublox_msgs__msg__NavRELPOSNED9__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavRELPOSNED9>, size: usize) -> bool;
    fn ublox_msgs__msg__NavRELPOSNED9__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavRELPOSNED9>);
    fn ublox_msgs__msg__NavRELPOSNED9__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavRELPOSNED9>, out_seq: *mut rosidl_runtime_rs::Sequence<NavRELPOSNED9>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavRELPOSNED9
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-RELPOSNED (0x01 0x3C)
/// Relative Positioning Information in NED frame
///
/// The NED frame is defined as the local topological system at the reference
/// station. The relative position vector components in this message, along with
/// their associated accuracies, are given in that local topological system
/// This message contains the relative position vector from the Reference Station
/// to the Rover, including accuracy figures, in the local topological system
/// defined at the reference station
///
/// Supported on:
///  - u-blox 9 from protocol version 27.11 (only with High Precision GNSS products)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavRELPOSNED9 {
    /// Message version (0x00 for this version)
    pub version: u8,

    /// Reserved
    pub reserved1: u8,

    /// Reference Station ID. Must be in the range
    /// 0..4095
    pub ref_station_id: u16,

    /// GPS time of week of the navigation epoch
    /// [ms]
    pub i_tow: u32,

    /// North component of relative position vector
    /// [cm]
    pub rel_pos_n: i32,

    /// East component of relative position vector
    /// [cm]
    pub rel_pos_e: i32,

    /// Down component of relative position vector
    /// [cm]
    pub rel_pos_d: i32,

    /// Length of the relative position vector
    /// [cm]
    pub rel_pos_length: i32,

    /// Heading of the relative position vector
    /// [1e-5 deg]
    pub rel_pos_heading: i32,

    /// Reserved
    pub reserved2: [u8; 4],

    /// High-precision North component of relative
    /// position vector.
    /// Must be in the range -99 to +99.
    /// The full North component of the relative
    /// position vector, in units of cm, is given by
    /// relPosN + (relPosHPN * 1e-2)
    pub rel_pos_hpn: i8,

    /// High-precision East component of relative
    /// position vector.
    /// Must be in the range -99 to +99.
    /// The full East component of the relative
    /// position vector, in units of cm, is given by
    /// relPosE + (relPosHPE * 1e-2)
    pub rel_pos_hpe: i8,

    /// High-precision Down component of relative
    /// position vector.
    /// Must be in the range -99 to +99.
    /// The full Down component of the relative
    /// position vector, in units of cm, is given by
    /// relPosD + (relPosHPD * 1e-2)
    pub rel_pos_hpd: i8,

    /// High-precision component of the length of
    /// the relative position vector.
    /// Must be in the range -99 to +99.
    /// The full length of the relative position
    /// vector, in units of cm, is given by
    /// relPosLength + (relPosHPLength * 1e-2)
    pub rel_pos_hp_length: i8,

    /// Accuracy of relative position North
    /// component
    pub acc_n: u32,

    /// Accuracy of relative position East component
    /// [0.1 mm]
    pub acc_e: u32,

    /// Accuracy of relative position Down component
    /// [0.1 mm]
    pub acc_d: u32,

    /// Accuracy of length of the relative position
    /// vector
    pub acc_length: u32,

    /// Accuracy of heading of the relative position
    /// vector
    pub acc_heading: u32,

    /// Reserved
    pub reserved3: [u8; 4],


    // This member is not documented.
    #[allow(missing_docs)]
    pub flags: u32,

}

impl NavRELPOSNED9 {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 60;

    /// A valid fix (i.e within DOP & accuracy
    /// masks)
    pub const FLAGS_GNSS_FIX_OK: u32 = 1;

    /// Set if differential corrections were applied
    pub const FLAGS_DIFF_SOLN: u32 = 2;

    /// Set if relative position components and
    /// accuracies are valid
    pub const FLAGS_REL_POS_VALID: u32 = 4;

    /// Carrier phase range solution status:
    pub const FLAGS_CARR_SOLN_MASK: u32 = 24;

    /// No carrier phase range solution
    pub const FLAGS_CARR_SOLN_NONE: u32 = 0;

    /// Float solution. No fixed integer carrier
    /// phase measurements have been used to
    /// calculate the solution
    pub const FLAGS_CARR_SOLN_FLOAT: u32 = 8;

    /// Fixed solution. One or more fixed
    /// integer carrier phase range measurements
    /// have been used to calculate the solution
    pub const FLAGS_CARR_SOLN_FIXED: u32 = 16;

    /// if the receiver is operating in moving
    /// baseline mode (not supported in protocol
    /// versions less than 20.3)
    pub const FLAGS_IS_MOVING: u32 = 32;

    /// Set if extrapolated reference position was
    /// used to compute moving baseline solution
    /// this epoch (not supported in protocol
    /// versions less than 20.3)
    pub const FLAGS_REF_POS_MISS: u32 = 64;

    /// Set if extrapolated reference observations
    /// were used to compute moving baseline
    /// solution this epoch (not supported in
    /// protocol versions less than 20.3)
    pub const FLAGS_REF_OBS_MISS: u32 = 128;

    /// Set if extrapolated reference observations
    /// were used to compute moving baseline
    /// solution this epoch (not supported in
    /// protocol versions less than 20.3)
    pub const FLAGS_REL_POS_HEAD_VALID: u32 = 256;

    /// Set if extrapolated reference observations
    /// were used to compute moving baseline
    /// solution this epoch (not supported in
    /// protocol versions less than 20.3)
    pub const FLAGS_REL_POS_NORM: u32 = 512;

}


impl Default for NavRELPOSNED9 {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavRELPOSNED9__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavRELPOSNED9__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavRELPOSNED9 {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavRELPOSNED9__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavRELPOSNED9__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavRELPOSNED9__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavRELPOSNED9 {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavRELPOSNED9 where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavRELPOSNED9";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavRELPOSNED9() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavRELPOSNED() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavRELPOSNED__init(msg: *mut NavRELPOSNED) -> bool;
    fn ublox_msgs__msg__NavRELPOSNED__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavRELPOSNED>, size: usize) -> bool;
    fn ublox_msgs__msg__NavRELPOSNED__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavRELPOSNED>);
    fn ublox_msgs__msg__NavRELPOSNED__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavRELPOSNED>, out_seq: *mut rosidl_runtime_rs::Sequence<NavRELPOSNED>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavRELPOSNED
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-RELPOSNED (0x01 0x3C)
/// Relative Positioning Information in NED frame
///
/// The NED frame is defined as the local topological system at the reference
/// station. The relative position vector components in this message, along with
/// their associated accuracies, are given in that local topological system
/// This message contains the relative position vector from the Reference Station
/// to the Rover, including accuracy figures, in the local topological system
/// defined at the reference station
///
/// Supported on:
///  - u-blox 8 / u-blox M8 from protocol version 20 up to version 23.01 (only
///    with High Precision GNSS products)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavRELPOSNED {
    /// Message version (0x00 for this version)
    pub version: u8,

    /// Reserved
    pub reserved0: u8,

    /// Reference Station ID. Must be in the range
    /// 0..4095
    pub ref_station_id: u16,

    /// GPS time of week of the navigation epoch
    /// [ms]
    pub i_tow: u32,

    /// North component of relative position vector
    /// [cm]
    pub rel_pos_n: i32,

    /// East component of relative position vector
    /// [cm]
    pub rel_pos_e: i32,

    /// Down component of relative position vector
    /// [cm]
    pub rel_pos_d: i32,

    /// High-precision North component of relative
    /// position vector.
    /// Must be in the range -99 to +99.
    /// The full North component of the relative
    /// position vector, in units of cm, is given by
    /// relPosN + (relPosHPN * 1e-2)
    pub rel_pos_hpn: i8,

    /// High-precision East component of relative
    /// position vector.
    /// Must be in the range -99 to +99.
    /// The full East component of the relative
    /// position vector, in units of cm, is given by
    /// relPosE + (relPosHPE * 1e-2)
    pub rel_pos_hpe: i8,

    /// High-precision Down component of relative
    /// position vector.
    /// Must be in the range -99 to +99.
    /// The full Down component of the relative
    /// position vector, in units of cm, is given by
    /// relPosD + (relPosHPD * 1e-2)
    pub rel_pos_hpd: i8,

    /// Reserved
    pub reserved1: u8,

    /// Accuracy of relative position North
    /// component
    pub acc_n: u32,

    /// Accuracy of relative position East component
    /// [0.1 mm]
    pub acc_e: u32,

    /// Accuracy of relative position Down component
    /// [0.1 mm]
    pub acc_d: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub flags: u32,

}

impl NavRELPOSNED {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 60;

    /// A valid fix (i.e within DOP & accuracy
    /// masks)
    pub const FLAGS_GNSS_FIX_OK: u32 = 1;

    /// Set if differential corrections were applied
    pub const FLAGS_DIFF_SOLN: u32 = 2;

    /// Set if relative position components and
    /// accuracies are valid
    pub const FLAGS_REL_POS_VALID: u32 = 4;

    /// Carrier phase range solution status:
    pub const FLAGS_CARR_SOLN_MASK: u32 = 24;

    /// No carrier phase range solution
    pub const FLAGS_CARR_SOLN_NONE: u32 = 0;

    /// Float solution. No fixed integer carrier
    /// phase measurements have been used to
    /// calculate the solution
    pub const FLAGS_CARR_SOLN_FLOAT: u32 = 8;

    /// Fixed solution. One or more fixed
    /// integer carrier phase range measurements
    /// have been used to calculate the solution
    pub const FLAGS_CARR_SOLN_FIXED: u32 = 16;

    /// if the receiver is operating in moving
    /// baseline mode (not supported in protocol
    /// versions less than 20.3)
    pub const FLAGS_IS_MOVING: u32 = 32;

    /// Set if extrapolated reference position was
    /// used to compute moving baseline solution
    /// this epoch (not supported in protocol
    /// versions less than 20.3)
    pub const FLAGS_REF_POS_MISS: u32 = 64;

    /// Set if extrapolated reference observations
    /// were used to compute moving baseline
    /// solution this epoch (not supported in
    /// protocol versions less than 20.3)
    pub const FLAGS_REF_OBS_MISS: u32 = 128;

}


impl Default for NavRELPOSNED {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavRELPOSNED__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavRELPOSNED__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavRELPOSNED {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavRELPOSNED__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavRELPOSNED__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavRELPOSNED__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavRELPOSNED {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavRELPOSNED where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavRELPOSNED";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavRELPOSNED() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSAT() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavSAT__init(msg: *mut NavSAT) -> bool;
    fn ublox_msgs__msg__NavSAT__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavSAT>, size: usize) -> bool;
    fn ublox_msgs__msg__NavSAT__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavSAT>);
    fn ublox_msgs__msg__NavSAT__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavSAT>, out_seq: *mut rosidl_runtime_rs::Sequence<NavSAT>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavSAT
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-SAT (0x01 0x35)
/// Satellite Information
///
/// This message displays information about SVs which are either known to be
/// visible or currently tracked by the receiver.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSAT {
    /// GPS time of week of the navigation epoch.
    pub i_tow: u32,

    /// Message version (1 for this version)
    pub version: u8,

    /// Number of satellites
    pub num_svs: u8,

    /// Reserved
    pub reserved0: [u8; 2],

    /// start of repeated block (numSvs times)
    pub sv: rosidl_runtime_rs::Sequence<super::super::msg::rmw::NavSATSV>,

}

impl NavSAT {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 53;

}


impl Default for NavSAT {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavSAT__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavSAT__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavSAT {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSAT__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSAT__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSAT__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavSAT {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavSAT where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavSAT";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSAT() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSATSV() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavSATSV__init(msg: *mut NavSATSV) -> bool;
    fn ublox_msgs__msg__NavSATSV__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavSATSV>, size: usize) -> bool;
    fn ublox_msgs__msg__NavSATSV__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavSATSV>);
    fn ublox_msgs__msg__NavSATSV__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavSATSV>, out_seq: *mut rosidl_runtime_rs::Sequence<NavSATSV>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavSATSV
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// see NAV-SAT message

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSATSV {
    /// GNSS identifier
    pub gnss_id: u8,

    /// Satellite identifier
    pub sv_id: u8,

    /// Carrier to noise ratio (signal strength) ]dBHz
    pub cno: u8,

    /// Elevation (range: +/-90), unknown if out of range
    pub elev: i8,

    /// Azimuth (range 0-360), unknown if elevation is out of range
    /// [deg]
    pub azim: i16,

    /// Pseudo range residual
    pub pr_res: i16,

    /// Bitmask
    pub flags: u32,

}

impl NavSATSV {
    /// Signal quality indicator:
    pub const FLAGS_QUALITY_IND_MASK: u32 = 7;

    /// no signal
    pub const QUALITY_IND_NO_SIGNAL: u8 = 0;

    /// searching signal
    pub const QUALITY_IND_SEARCHING_SIGNAL: u8 = 1;

    /// signal acquired
    pub const QUALITY_IND_SIGNAL_ACQUIRED: u8 = 2;

    /// signal detected but
    /// unusable
    pub const QUALITY_IND_SIGNAL_DETECTED_BUT_UNUSABLE: u8 = 3;

    /// code locked and time
    /// synchronized
    pub const QUALITY_IND_CODE_LOCKED_AND_TIME_SYNC: u8 = 4;

    /// code and carrier
    /// locked and time
    /// synchronized,
    /// quality = 1
    pub const QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC1: u8 = 5;

    /// code and carrier
    /// locked and time
    /// synchronized,
    /// quality = 2
    pub const QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC2: u8 = 6;

    /// code and carrier
    /// locked and time
    /// synchronized,
    /// quality = 3
    pub const QUALITY_IND_CODE_AND_CARR_LOCK_AND_TIME_SYNC3: u8 = 7;

    /// Note: Since IMES signals are not time synchronized, a channel tracking an IMES
    /// signal can never reach a quality indicator value of higher than 3.
    /// whether SV is currently being
    /// used for navigation
    pub const FLAGS_SV_USED: u32 = 8;

    /// SV health flag:
    pub const FLAGS_HEALTH_MASK: u32 = 48;

    /// unknown
    pub const HEALTH_UNKNOWN: u32 = 0;

    /// healthy
    pub const HEALTH_HEALTHY: u32 = 1;

    /// unhealthy
    pub const HEALTH_UNHEALTHY: u32 = 2;

    /// whether differential correction
    /// data is available for this SV
    pub const FLAGS_DIFF_CORR: u32 = 64;

    /// whether carrier smoothed
    /// pseudorange used
    pub const FLAGS_SMOOTHED: u32 = 128;

    /// Orbit source:
    pub const FLAGS_ORBIT_SOURCE_MASK: u32 = 1792;

    /// no orbit information is
    /// available for this SV
    pub const ORBIT_SOURCE_UNAVAILABLE: u32 = 0;

    /// ephemeris is used
    pub const ORBIT_SOURCE_EPH: u32 = 256;

    /// almanac is used
    pub const ORBIT_SOURCE_ALM: u32 = 512;

    /// AssistNow Offline orbit is
    /// used
    pub const ORBIT_SOURCE_ASSIST_OFFLINE: u32 = 768;

    /// AssistNow Autonomous orbit is
    /// used
    pub const ORBIT_SOURCE_ASSIST_AUTONOMOUS: u32 = 1024;

    /// other orbit information is
    /// used
    pub const ORBIT_SOURCE_OTHER1: u32 = 1280;

    /// other orbit information is
    /// used
    pub const ORBIT_SOURCE_OTHER2: u32 = 1536;

    /// other orbit information is
    /// used
    pub const ORBIT_SOURCE_OTHER3: u32 = 1792;

    /// whether ephemeris is available
    /// for this SV
    pub const FLAGS_EPH_AVAIL: u32 = 2048;

    /// whether almanac is available for
    /// this SV
    pub const FLAGS_ALM_AVAIL: u32 = 4096;

    /// whether AssistNow Offline data
    /// is available for this SV
    pub const FLAGS_ANO_AVAIL: u32 = 8192;

    /// whether AssistNow Autonomous
    /// data is available for this SV
    pub const FLAGS_AOP_AVAIL: u32 = 16384;

    /// whether SBAS corrections have
    /// been used for this SV
    pub const FLAGS_SBAS_CORR_USED: u32 = 65536;

    /// whether RTCM corrections have
    /// been used for this SV
    pub const FLAGS_RTCM_CORR_USED: u32 = 131072;

    /// whether Pseudorange corrections
    /// have been used for this SV
    pub const FLAGS_PR_CORR_USED: u32 = 1048576;

    /// whether Carrier range
    /// corrections have been used for
    /// this SV
    pub const FLAGS_CR_CORR_USED: u32 = 2097152;

    /// whether Range rate (Doppler)
    /// corrections have been used for
    /// this SV
    pub const FLAGS_DO_CORR_USED: u32 = 4194304;

}


impl Default for NavSATSV {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavSATSV__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavSATSV__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavSATSV {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSATSV__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSATSV__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSATSV__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavSATSV {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavSATSV where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavSATSV";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSATSV() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSBAS() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavSBAS__init(msg: *mut NavSBAS) -> bool;
    fn ublox_msgs__msg__NavSBAS__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavSBAS>, size: usize) -> bool;
    fn ublox_msgs__msg__NavSBAS__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavSBAS>);
    fn ublox_msgs__msg__NavSBAS__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavSBAS>, out_seq: *mut rosidl_runtime_rs::Sequence<NavSBAS>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavSBAS
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-SBAS (0x01 0x32)
/// SBAS Status Data
///
/// This message outputs the status of the SBAS sub system

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSBAS {
    /// GPS Millisecond time of week
    pub i_tow: u32,

    /// PRN Number of the GEO where correction and integrity
    /// data is used from
    pub geo: u8,

    /// SBAS Mode
    pub mode: u8,

    /// SBAS System (WAAS/EGNOS/...)
    pub sys: i8,

    /// SBAS Services available
    pub service: u8,

    /// Number of SV data following
    pub cnt: u8,

    /// Reserved
    pub reserved0: [u8; 3],


    // This member is not documented.
    #[allow(missing_docs)]
    pub sv: rosidl_runtime_rs::Sequence<super::super::msg::rmw::NavSBASSV>,

}

impl NavSBAS {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 50;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MODE_DISABLED: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MODE_ENABLED_INTEGRITY: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MODE_ENABLED_TESTMODE: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SYS_UNKNOWN: i8 = -1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SYS_WAAS: i8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SYS_EGNOS: i8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SYS_MSAS: i8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SYS_GAGAN: i8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SYS_GPS: i8 = 16;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SERVICE_RANGING: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SERVICE_CORRECTIONS: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SERVICE_INTEGRITY: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const SERVICE_TESTMODE: u8 = 8;

}


impl Default for NavSBAS {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavSBAS__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavSBAS__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavSBAS {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSBAS__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSBAS__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSBAS__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavSBAS {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavSBAS where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavSBAS";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSBAS() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSBASSV() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavSBASSV__init(msg: *mut NavSBASSV) -> bool;
    fn ublox_msgs__msg__NavSBASSV__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavSBASSV>, size: usize) -> bool;
    fn ublox_msgs__msg__NavSBASSV__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavSBASSV>);
    fn ublox_msgs__msg__NavSBASSV__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavSBASSV>, out_seq: *mut rosidl_runtime_rs::Sequence<NavSBASSV>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavSBASSV
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// see message NavSBAS

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSBASSV {
    /// SV Id
    pub svid: u8,

    /// Flags for this SV
    pub flags: u8,

    /// Monitoring status
    pub udre: u8,

    /// System (WAAS/EGNOS/...), same as SYS
    pub sv_sys: u8,

    /// Services available, same as SERVICE
    pub sv_service: u8,

    /// Reserved
    pub reserved1: u8,

    /// Pseudo Range correction in
    pub prc: i16,

    /// Reserved
    pub reserved2: u16,

    /// Ionosphere correction in
    pub ic: i16,

}



impl Default for NavSBASSV {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavSBASSV__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavSBASSV__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavSBASSV {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSBASSV__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSBASSV__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSBASSV__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavSBASSV {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavSBASSV where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavSBASSV";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSBASSV() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSOL() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavSOL__init(msg: *mut NavSOL) -> bool;
    fn ublox_msgs__msg__NavSOL__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavSOL>, size: usize) -> bool;
    fn ublox_msgs__msg__NavSOL__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavSOL>);
    fn ublox_msgs__msg__NavSOL__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavSOL>, out_seq: *mut rosidl_runtime_rs::Sequence<NavSOL>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavSOL
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-SOL (0x01 0x06)
/// Navigation Solution Information
///
/// This message combines Position, velocity and time solution in ECEF, including
/// accuracy figures
/// This message has only been retained for backwards compatibility; users are
/// recommended to use the UBX-NAV-PVT message in preference.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSOL {
    /// GPS Millisecond time of week
    pub i_tow: u32,

    /// Fractional Nanoseconds remainder of rounded
    /// ms above, range -500000 .. 500000
    pub f_tow: i32,

    /// GPS week (GPS time)
    pub week: i16,

    /// GPSfix Type, range 0..5
    pub gps_fix: u8,

    /// Fix Status Flags
    pub flags: u8,

    /// ECEF X coordinate
    pub ecef_x: i32,

    /// ECEF Y coordinate
    pub ecef_y: i32,

    /// ECEF Z coordinate
    pub ecef_z: i32,

    /// 3D Position Accuracy Estimate
    pub p_acc: u32,

    /// ECEF X velocity
    pub ecef_vx: i32,

    /// ECEF Y velocity
    pub ecef_vy: i32,

    /// ECEF Z velocity
    pub ecef_vz: i32,

    /// Speed Accuracy Estimate
    pub s_acc: u32,

    /// Position DOP
    pub p_dop: u16,

    /// Reserved
    pub reserved1: u8,

    /// Number of SVs used in Nav Solution
    pub num_sv: u8,

    /// Reserved
    pub reserved2: u32,

}

impl NavSOL {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 6;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GPS_NO_FIX: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GPS_DEAD_RECKONING_ONLY: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GPS_2D_FIX: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GPS_3D_FIX: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GPS_GPS_DEAD_RECKONING_COMBINED: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GPS_TIME_ONLY_FIX: u8 = 5;

    /// Fix within limits i.e. within DOP & ACC Masks
    pub const FLAGS_GPS_FIX_OK: u8 = 1;

    /// DGPS used
    pub const FLAGS_DIFF_SOLN: u8 = 2;

    /// Week Number valid
    pub const FLAGS_WKNSET: u8 = 4;

    /// Time of Week valid
    pub const FLAGS_TOWSET: u8 = 8;

}


impl Default for NavSOL {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavSOL__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavSOL__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavSOL {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSOL__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSOL__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSOL__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavSOL {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavSOL where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavSOL";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSOL() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSTATUS() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavSTATUS__init(msg: *mut NavSTATUS) -> bool;
    fn ublox_msgs__msg__NavSTATUS__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavSTATUS>, size: usize) -> bool;
    fn ublox_msgs__msg__NavSTATUS__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavSTATUS>);
    fn ublox_msgs__msg__NavSTATUS__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavSTATUS>, out_seq: *mut rosidl_runtime_rs::Sequence<NavSTATUS>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavSTATUS
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-STATUS (0x01 0x03)
/// Receiver Navigation Status
///
/// See important comments concerning validity of position and velocity given in
/// section Navigation Output Filters.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSTATUS {
    /// GPS Millisecond time of week
    pub i_tow: u32,

    /// GPSfix Type, this value does not qualify a fix as
    /// valid and within the limits. See note on flag gpsFixOk
    /// below
    pub gps_fix: u8,

    /// Navigation Status Flags
    pub flags: u8,

    /// Fix Status Information
    pub fix_stat: u8,

    /// further information about navigation output
    pub flags2: u8,

    /// Time to first fix (millisecond time tag)
    pub ttff: u32,

    /// Milliseconds since Startup / Reset
    pub msss: u32,

}

impl NavSTATUS {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GPS_NO_FIX: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GPS_DEAD_RECKONING_ONLY: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GPS_2D_FIX: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GPS_3D_FIX: u8 = 3;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GPS_GPS_DEAD_RECKONING_COMBINED: u8 = 4;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const GPS_TIME_ONLY_FIX: u8 = 5;

    /// position & velocity valid & within DOP & ACC
    /// Masks
    pub const FLAGS_GPS_FIX_OK: u8 = 1;

    /// Differential corrections were applied
    pub const FLAGS_DIFF_SOLN: u8 = 2;

    /// Week Number valid
    pub const FLAGS_WKNSET: u8 = 4;

    /// Time of Week valid
    pub const FLAGS_TOWSET: u8 = 8;

    /// 1 = differential corrections available
    pub const FIX_STAT_DIFF_CORR_MASK: u8 = 1;

    /// map matching status:
    pub const FIX_STAT_MAP_MATCHING_MASK: u8 = 192;

    /// none
    pub const MAP_MATCHING_NONE: u8 = 0;

    /// valid but not used, i.e. map matching data
    /// was received, but was too old
    pub const MAP_MATCHING_VALID: u8 = 64;

    /// valid and used, map matching data was applied
    pub const MAP_MATCHING_USED: u8 = 128;

    /// valid and used, map matching data was
    /// applied. In case of sensor unavailability map
    /// matching data enables dead reckoning.
    /// This requires map matched latitude/longitude
    /// or heading data.
    pub const MAP_MATCHING_DR: u8 = 192;

    /// power safe mode state (Only for FW version >= 7.01; undefined otherwise)
    pub const FLAGS2_PSM_STATE_MASK: u8 = 3;

    /// ACQUISITION
    /// [or when psm disabled]
    pub const PSM_STATE_ACQUISITION: u8 = 0;

    /// TRACKING
    pub const PSM_STATE_TRACKING: u8 = 1;

    /// POWER OPTIMIZED TRACKING
    pub const PSM_STATE_POWER_OPTIMIZED_TRACKING: u8 = 2;

    /// INACTIVE
    pub const PSM_STATE_INACTIVE: u8 = 3;

    /// Note that the spoofing state value only reflects the detector state for the
    /// current navigation epoch. As spoofing can be detected most easily at the
    /// transition from real signal to spoofing signal, this is also where the
    /// detector is triggered the most. I.e. a value of 1 - No spoofing indicated does
    /// not mean that the receiver is not spoofed, it #simply states that the detector
    /// was not triggered in this epoch.
    pub const FLAGS2_SPOOF_DET_STATE_MASK: u8 = 24;

    /// Unknown or deactivated
    pub const SPOOF_DET_STATE_UNKNOWN: u8 = 0;

    /// No spoofing indicated
    pub const SPOOF_DET_STATE_NONE: u8 = 8;

    /// Spoofing indicated
    pub const SPOOF_DET_STATE_SPOOFING: u8 = 16;

    /// Multiple spoofing indication
    pub const SPOOF_DET_STATE_MULTIPLE: u8 = 24;

}


impl Default for NavSTATUS {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavSTATUS__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavSTATUS__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavSTATUS {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSTATUS__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSTATUS__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSTATUS__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavSTATUS {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavSTATUS where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavSTATUS";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSTATUS() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSVINFO() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavSVINFO__init(msg: *mut NavSVINFO) -> bool;
    fn ublox_msgs__msg__NavSVINFO__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavSVINFO>, size: usize) -> bool;
    fn ublox_msgs__msg__NavSVINFO__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavSVINFO>);
    fn ublox_msgs__msg__NavSVINFO__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavSVINFO>, out_seq: *mut rosidl_runtime_rs::Sequence<NavSVINFO>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavSVINFO
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-SVINFO (0x01 0x30)
/// Space Vehicle Information

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSVINFO {
    /// GPS Millisecond time of week
    pub i_tow: u32,

    /// Number of channels
    pub num_ch: u8,

    /// Bitmask
    pub global_flags: u8,

    /// Reserved
    pub reserved2: u16,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sv: rosidl_runtime_rs::Sequence<super::super::msg::rmw::NavSVINFOSV>,

}

impl NavSVINFO {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 48;

    /// Chip Hardware generation flags
    /// Antaris, Antaris 4
    pub const CHIPGEN_ANTARIS: u8 = 0;

    /// u-blox 5
    pub const CHIPGEN_UBLOX5: u8 = 1;

    /// u-blox 6
    pub const CHIPGEN_UBLOX6: u8 = 2;

    /// u-blox 7
    pub const CHIPGEN_UBLOX7: u8 = 3;

    /// u-blox 8 / u-blox M8
    pub const CHIPGEN_UBLOX8: u8 = 4;

}


impl Default for NavSVINFO {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavSVINFO__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavSVINFO__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavSVINFO {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSVINFO__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSVINFO__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSVINFO__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavSVINFO {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavSVINFO where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavSVINFO";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSVINFO() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSVINFOSV() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavSVINFOSV__init(msg: *mut NavSVINFOSV) -> bool;
    fn ublox_msgs__msg__NavSVINFOSV__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavSVINFOSV>, size: usize) -> bool;
    fn ublox_msgs__msg__NavSVINFOSV__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavSVINFOSV>);
    fn ublox_msgs__msg__NavSVINFOSV__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavSVINFOSV>, out_seq: *mut rosidl_runtime_rs::Sequence<NavSVINFOSV>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavSVINFOSV
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// see message NavSVINFO

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSVINFOSV {
    /// Channel number, 255 for SVs not assigned to a channel
    pub chn: u8,

    /// Satellite ID
    pub svid: u8,

    /// Bitmask
    pub flags: u8,

    /// Bitfield
    pub quality: u8,

    /// Carrier to Noise Ratio (Signal Strength)
    pub cno: u8,

    /// Elevation in integer degrees
    pub elev: i8,

    /// Azimuth in integer degrees
    pub azim: i16,

    /// Pseudo range residual in centimetres
    pub pr_res: i32,

}

impl NavSVINFOSV {
    /// SV is used for navigation
    pub const FLAGS_SV_USED: u8 = 1;

    /// Differential correction data
    /// is available for this SV
    pub const FLAGS_DIFF_CORR: u8 = 2;

    /// Orbit information is available for
    /// this SV (Ephemeris or Almanach)
    pub const FLAGS_ORBIT_AVAIL: u8 = 4;

    /// Orbit information is Ephemeris
    pub const FLAGS_ORBIT_EPH: u8 = 8;

    /// SV is unhealthy / shall not be
    /// used
    pub const FLAGS_UNHEALTHY: u8 = 16;

    /// Orbit information is Almanac Plus
    pub const FLAGS_ORBIT_ALM: u8 = 32;

    /// Orbit information is AssistNow
    /// Autonomous
    pub const FLAGS_ORBIT_AOP: u8 = 64;

    /// Carrier smoothed pseudorange used
    pub const FLAGS_SMOOTHED: u8 = 128;

    /// qualityInd: Signal Quality indicator (range 0..7). The following list shows
    /// the meaning of the different QI values:
    /// Note: Since IMES signals are not time synchronized, a channel tracking an IMES
    /// signal can never reach a quality indicator value of higher than 3.
    /// This channel is idle
    pub const QUALITY_IDLE: u8 = 0;

    /// Channel is searching
    pub const QUALITY_SEARCHING: u8 = 1;

    /// Signal acquired
    pub const QUALITY_ACQUIRED: u8 = 2;

    /// Signal detected but unusable
    pub const QUALITY_DETECTED: u8 = 3;

    /// Code Lock on Signal
    pub const QUALITY_CODE_LOCK: u8 = 4;

    /// Code and Carrier locked
    /// and time synchronized
    pub const QUALITY_CODE_AND_CARRIER_LOCKED1: u8 = 5;

    /// Code and Carrier locked
    /// and time synchronized
    pub const QUALITY_CODE_AND_CARRIER_LOCKED2: u8 = 6;

    /// Code and Carrier locked
    /// and time synchronized
    pub const QUALITY_CODE_AND_CARRIER_LOCKED3: u8 = 7;

}


impl Default for NavSVINFOSV {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavSVINFOSV__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavSVINFOSV__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavSVINFOSV {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSVINFOSV__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSVINFOSV__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSVINFOSV__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavSVINFOSV {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavSVINFOSV where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavSVINFOSV";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSVINFOSV() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSVIN() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavSVIN__init(msg: *mut NavSVIN) -> bool;
    fn ublox_msgs__msg__NavSVIN__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavSVIN>, size: usize) -> bool;
    fn ublox_msgs__msg__NavSVIN__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavSVIN>);
    fn ublox_msgs__msg__NavSVIN__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavSVIN>, out_seq: *mut rosidl_runtime_rs::Sequence<NavSVIN>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavSVIN
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-SVIN (0x01 0x3B)
/// Survey-in data
///
/// This message contains information about survey-in parameters.
/// Supported on:
///  - u-blox 8 / u-blox M8 with protocol version 20 (only with High Precision
///    GNSS products)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavSVIN {
    /// Message version (0x00 for this version)
    pub version: u8,

    /// Reserved
    pub reserved0: [u8; 3],

    /// GPS time of week of the navigation epoch
    pub i_tow: u32,

    /// Passed survey-in observation time
    pub dur: u32,

    /// Current survey-in mean position ECEF X coordinate
    pub mean_x: i32,

    /// Current survey-in mean position ECEF Y coordinate
    pub mean_y: i32,

    /// Current survey-in mean position ECEF Z coordinate
    pub mean_z: i32,

    /// Current high-precision survey-in mean position
    /// ECEF X coordinate. 0.1_mm
    /// Must be in the range -99..+99.
    /// The current survey-in mean position ECEF X
    /// coordinate, in units of cm, is given by
    /// meanX + (0.01 * meanXHP)
    pub mean_xhp: i8,

    /// Current high-precision survey-in mean position
    /// ECEF Y coordinate.
    /// Must be in the range -99..+99.
    /// The current survey-in mean position ECEF Y
    /// coordinate, in units of cm, is given by
    /// meanY + (0.01 * meanYHP)
    pub mean_yhp: i8,

    /// Current high-precision survey-in mean position
    /// ECEF Z coordinate.
    /// Must be in the range -99..+99.
    /// The current survey-in mean position ECEF Z
    /// coordinate, in units of cm, is given by
    /// meanZ + (0.01 * meanZHP)
    pub mean_zhp: i8,

    /// Reserved
    pub reserved1: u8,

    /// Current survey-in mean position accuracy
    pub mean_acc: u32,

    /// Number of position observations used during survey-in
    pub obs: u32,

    /// Survey-in position validity flag, 1 = valid
    /// otherwise 0
    pub valid: u8,

    /// Survey-in in progress flag, 1 = in-progress
    /// otherwise 0
    pub active: u8,

    /// Reserved
    pub reserved3: [u8; 2],

}

impl NavSVIN {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 59;

}


impl Default for NavSVIN {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavSVIN__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavSVIN__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavSVIN {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSVIN__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSVIN__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavSVIN__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavSVIN {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavSVIN where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavSVIN";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavSVIN() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavTIMEGPS() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavTIMEGPS__init(msg: *mut NavTIMEGPS) -> bool;
    fn ublox_msgs__msg__NavTIMEGPS__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavTIMEGPS>, size: usize) -> bool;
    fn ublox_msgs__msg__NavTIMEGPS__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavTIMEGPS>);
    fn ublox_msgs__msg__NavTIMEGPS__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavTIMEGPS>, out_seq: *mut rosidl_runtime_rs::Sequence<NavTIMEGPS>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavTIMEGPS
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-TIMEGPS (0x01 0x20)
/// GPS Time Solution

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavTIMEGPS {
    /// GPS Millisecond time of week
    pub i_tow: u32,

    /// Fractional Nanoseconds remainder of rounded
    /// ms above, range -500000 .. 500000
    pub f_tow: i32,

    /// GPS week (GPS time)
    pub week: i16,

    /// Leap Seconds (GPS-UTC)
    pub leap_s: i8,

    /// Validity Flags
    pub valid: u8,

    /// Time Accuracy Estimate
    pub t_acc: u32,

}

impl NavTIMEGPS {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 32;

    /// Valid Time of Week
    pub const VALID_TOW: u8 = 1;

    /// Valid Week Number
    pub const VALID_WEEK: u8 = 2;

    /// Valid Leap Seconds
    pub const VALID_LEAP_S: u8 = 4;

}


impl Default for NavTIMEGPS {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavTIMEGPS__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavTIMEGPS__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavTIMEGPS {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavTIMEGPS__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavTIMEGPS__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavTIMEGPS__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavTIMEGPS {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavTIMEGPS where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavTIMEGPS";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavTIMEGPS() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavTIMEUTC() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavTIMEUTC__init(msg: *mut NavTIMEUTC) -> bool;
    fn ublox_msgs__msg__NavTIMEUTC__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavTIMEUTC>, size: usize) -> bool;
    fn ublox_msgs__msg__NavTIMEUTC__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavTIMEUTC>);
    fn ublox_msgs__msg__NavTIMEUTC__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavTIMEUTC>, out_seq: *mut rosidl_runtime_rs::Sequence<NavTIMEUTC>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavTIMEUTC
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-TIMEUTC (0x01 0x21)
/// UTC Time Solution

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavTIMEUTC {
    /// GPS Millisecond time of week
    pub i_tow: u32,

    /// Time Accuracy Estimate
    pub t_acc: u32,

    /// Fraction of second, range -1e9 .. 1e9 (UTC)
    pub nano: i32,

    /// Year, range 1999..2099 (UTC)
    pub year: u16,

    /// Month, range 1..12 (UTC)
    pub month: u8,

    /// Day of Month, range 1..31 (UTC)
    pub day: u8,

    /// Hour of Day, range 0..23 (UTC)
    pub hour: u8,

    /// Minute of Hour, range 0..59 (UTC)
    pub min: u8,

    /// Seconds of Minute, range 0..60 (UTC) (60 for
    /// leap second)
    pub sec: u8,

    /// Validity Flags
    pub valid: u8,

}

impl NavTIMEUTC {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 33;

    /// Valid Time of Week
    pub const VALID_TOW: u8 = 1;

    /// Valid Week Number
    pub const VALID_WKN: u8 = 2;

    /// Valid Leap Seconds, i.e. Leap Seconds already known
    pub const VALID_UTC: u8 = 4;

    /// UTC standard Identifier Bit mask:
    pub const VALID_UTC_STANDARD_MASK: u8 = 240;

    /// Information not available
    pub const UTC_STANDARD_NOT_AVAILABLE: u8 = 0;

    /// Communications Research Labratory
    pub const UTC_STANDARD_CRL: u8 = 16;

    /// National Institute of Standards and
    /// Technology (NIST)
    pub const UTC_STANDARD_NIST: u8 = 32;

    /// U.S. Naval Observatory (USNO)
    pub const UTC_STANDARD_USNO: u8 = 48;

    /// International Bureau of Weights and
    /// Measures (BIPM)
    pub const UTC_STANDARD_BIPM: u8 = 64;

    /// European Laboratory (tbd)
    pub const UTC_STANDARD_EL: u8 = 80;

    /// Former Soviet Union (SU)
    pub const UTC_STANDARD_SU: u8 = 96;

    /// National Time Service Center, China
    pub const UTC_STANDARD_NTSC: u8 = 112;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UTC_STANDARD_UNKNOWN: u8 = 240;

}


impl Default for NavTIMEUTC {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavTIMEUTC__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavTIMEUTC__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavTIMEUTC {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavTIMEUTC__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavTIMEUTC__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavTIMEUTC__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavTIMEUTC {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavTIMEUTC where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavTIMEUTC";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavTIMEUTC() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavVELECEF() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavVELECEF__init(msg: *mut NavVELECEF) -> bool;
    fn ublox_msgs__msg__NavVELECEF__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavVELECEF>, size: usize) -> bool;
    fn ublox_msgs__msg__NavVELECEF__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavVELECEF>);
    fn ublox_msgs__msg__NavVELECEF__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavVELECEF>, out_seq: *mut rosidl_runtime_rs::Sequence<NavVELECEF>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavVELECEF
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-VELECEF (0x01 0x11)
/// Velocity Solution in ECEF
///
/// See important comments concerning validity of velocity given in section
/// Navigation Output Filters.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavVELECEF {
    /// GPS Millisecond time of week
    pub i_tow: u32,

    /// ECEF X velocity
    pub ecef_vx: i32,

    /// ECEF Y velocity
    pub ecef_vy: i32,

    /// ECEF Z velocity
    pub ecef_vz: i32,

    /// Speed Accuracy Estimate
    pub s_acc: u32,

}

impl NavVELECEF {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 17;

}


impl Default for NavVELECEF {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavVELECEF__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavVELECEF__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavVELECEF {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavVELECEF__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavVELECEF__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavVELECEF__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavVELECEF {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavVELECEF where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavVELECEF";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavVELECEF() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavVELNED() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__NavVELNED__init(msg: *mut NavVELNED) -> bool;
    fn ublox_msgs__msg__NavVELNED__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<NavVELNED>, size: usize) -> bool;
    fn ublox_msgs__msg__NavVELNED__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<NavVELNED>);
    fn ublox_msgs__msg__NavVELNED__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<NavVELNED>, out_seq: *mut rosidl_runtime_rs::Sequence<NavVELNED>) -> bool;
}

// Corresponds to ublox_msgs__msg__NavVELNED
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// NAV-VELNED (0x01 0x12)
/// Velocity Solution in NED
///
/// See important comments concerning validity of velocity given in section
/// Navigation Output Filters.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct NavVELNED {
    /// GPS Millisecond time of week
    pub i_tow: u32,

    /// NED north velocity
    pub vel_n: i32,

    /// NED east velocity
    pub vel_e: i32,

    /// NED down velocity
    pub vel_d: i32,

    /// Speed (3-D)
    pub speed: u32,

    /// Ground Speed (2-D)
    pub g_speed: u32,

    /// Heading of motion 2-D
    pub heading: i32,

    /// Speed Accuracy Estimate
    pub s_acc: u32,

    /// Course / Heading Accuracy Estimate
    pub c_acc: u32,

}

impl NavVELNED {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 18;

}


impl Default for NavVELNED {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__NavVELNED__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__NavVELNED__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for NavVELNED {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavVELNED__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavVELNED__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__NavVELNED__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for NavVELNED {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for NavVELNED where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/NavVELNED";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__NavVELNED() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmALM() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__RxmALM__init(msg: *mut RxmALM) -> bool;
    fn ublox_msgs__msg__RxmALM__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RxmALM>, size: usize) -> bool;
    fn ublox_msgs__msg__RxmALM__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RxmALM>);
    fn ublox_msgs__msg__RxmALM__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RxmALM>, out_seq: *mut rosidl_runtime_rs::Sequence<RxmALM>) -> bool;
}

// Corresponds to ublox_msgs__msg__RxmALM
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// RXM-ALM (0x02 0x30)
/// GPS Aiding Almanach Input/Output Message
///
/// This message is provided considered obsolete, please use AID-ALM instead!
/// - If the WEEK Value is 0, DWRD0 to DWRD7 are not sent as the almanach is not 
///   available for the given SV.
/// - DWORD0 to DWORD7 contain the 8 words following the Hand-Over Word ( HOW )
///   from the GPS navigation message, either pages 1 to 24 of sub-frame 5 or 
///   pages 2 to 10 of subframe 4. See IS-GPS-200 for a full description of the 
///   contents of the Almanac pages.
/// - In DWORD0 to DWORD7, the parity bits have been removed, and the 24 bits of 
///   data are located in Bits 0 to 23. Bits 24 to 31 shall be ignored.
/// - Example: Parameter e (Eccentricity) from Almanach Subframe 4/5, Word 3, 
///   Bits 69-84 within the subframe can be found in DWRD0, Bits 15-0 whereas 
///   Bit 0 is the LSB.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RxmALM {
    /// SV ID for which this Almanach Data is
    /// (Valid Range: 1 .. 32 or 51, 56, 63).
    pub svid: u32,

    /// Issue Date of Almanach (GPS week number)
    pub week: u32,

    /// Start of optional block
    /// Almanach Words
    pub dwrd: rosidl_runtime_rs::Sequence<u32>,

}

impl RxmALM {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 48;

}


impl Default for RxmALM {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__RxmALM__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__RxmALM__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RxmALM {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmALM__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmALM__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmALM__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RxmALM {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RxmALM where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/RxmALM";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmALM() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmEPH() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__RxmEPH__init(msg: *mut RxmEPH) -> bool;
    fn ublox_msgs__msg__RxmEPH__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RxmEPH>, size: usize) -> bool;
    fn ublox_msgs__msg__RxmEPH__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RxmEPH>);
    fn ublox_msgs__msg__RxmEPH__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RxmEPH>, out_seq: *mut rosidl_runtime_rs::Sequence<RxmEPH>) -> bool;
}

// Corresponds to ublox_msgs__msg__RxmEPH
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// RXM-EPH (0x02 0x31)
/// GPS Aiding Ephemeris Input/Output Message
///
/// This message is provided considered obsolete, please use AID-EPH instead!
/// - SF1D0 to SF3D7 is only sent if ephemeris is available for this SV. If not, 
///   the payload may be reduced to 8 Bytes, or all bytes are set to zero, 
///   indicating that this SV Number does not have valid ephemeris for the moment.
/// - SF1D0 to SF3D7 contain the 24 words following the Hand-Over Word ( HOW ) 
///   from the GPS navigation message, subframes 1 to 3. See IS-GPS-200 for a 
///   full description of the contents of the Subframes.
/// - In SF1D0 to SF3D7, the parity bits have been removed, and the 24 bits of 
///   data are located in Bits 0 to 23. Bits 24 to 31 shall be ignored.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RxmEPH {
    /// SV ID for which this ephemeris data is (Valid Range: 1 .. 32).
    pub svid: u32,

    /// Hand-Over Word of first Subframe. This is
    /// required if data is sent to the receiver.
    /// 0 indicates that no Ephemeris Data is following.
    pub how: u32,

    /// Start of optional block
    /// Subframe 1 Words 3..10 (SF1D0..SF1D7)
    pub sf1d: rosidl_runtime_rs::Sequence<u32>,

    /// Subframe 2 Words 3..10 (SF2D0..SF2D7)
    pub sf2d: rosidl_runtime_rs::Sequence<u32>,

    /// Subframe 3 Words 3..10 (SF3D0..SF3D7)
    pub sf3d: rosidl_runtime_rs::Sequence<u32>,

}

impl RxmEPH {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 49;

}


impl Default for RxmEPH {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__RxmEPH__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__RxmEPH__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RxmEPH {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmEPH__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmEPH__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmEPH__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RxmEPH {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RxmEPH where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/RxmEPH";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmEPH() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRAW() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__RxmRAW__init(msg: *mut RxmRAW) -> bool;
    fn ublox_msgs__msg__RxmRAW__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RxmRAW>, size: usize) -> bool;
    fn ublox_msgs__msg__RxmRAW__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RxmRAW>);
    fn ublox_msgs__msg__RxmRAW__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RxmRAW>, out_seq: *mut rosidl_runtime_rs::Sequence<RxmRAW>) -> bool;
}

// Corresponds to ublox_msgs__msg__RxmRAW
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// RXM-RAW (0x02 0x10)
/// Raw Measurement Data
///
/// Supported up to ublox 7 firmware. See RxmRAWX for ublox 8
/// This message contains all information needed to be able to generate a RINEX
/// observation file.
/// This message outputs pseudorange, doppler and carrier phase measurements for
/// GPS satellites once signals have been synchronised. No other GNSS types are
/// currently supported.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RxmRAW {
    /// Measurement time of week in receiver local time
    pub rcv_tow: i32,

    /// Measurement week number in receiver local time
    pub week: i16,

    /// # of satellites following
    pub num_sv: u8,

    /// Reserved
    pub reserved1: u8,

    /// numSV times
    pub sv: rosidl_runtime_rs::Sequence<super::super::msg::rmw::RxmRAWSV>,

}

impl RxmRAW {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 16;

}


impl Default for RxmRAW {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__RxmRAW__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__RxmRAW__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RxmRAW {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRAW__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRAW__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRAW__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RxmRAW {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RxmRAW where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/RxmRAW";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRAW() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRAWSV() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__RxmRAWSV__init(msg: *mut RxmRAWSV) -> bool;
    fn ublox_msgs__msg__RxmRAWSV__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RxmRAWSV>, size: usize) -> bool;
    fn ublox_msgs__msg__RxmRAWSV__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RxmRAWSV>);
    fn ublox_msgs__msg__RxmRAWSV__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RxmRAWSV>, out_seq: *mut rosidl_runtime_rs::Sequence<RxmRAWSV>) -> bool;
}

// Corresponds to ublox_msgs__msg__RxmRAWSV
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// see message RxmRAW

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RxmRAWSV {
    /// Carrier phase measurement
    pub cp_mes: f64,

    /// Pseudorange measurement
    pub pr_mes: f64,

    /// Doppler measurement
    pub do_mes: f32,

    /// Space Vehicle Number
    pub sv: u8,

    /// Nav Measurements Quality Indicator
    ///  >=4 : PR+DO OK
    ///  >=5 : PR+DO+CP OK
    ///  <6 : likely loss of carrier lock in previous
    ///       interval
    pub mes_qi: i8,

    /// Signal strength C/No.
    pub cno: i8,

    /// Loss of lock indicator (RINEX definition)
    pub lli: u8,

}



impl Default for RxmRAWSV {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__RxmRAWSV__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__RxmRAWSV__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RxmRAWSV {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRAWSV__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRAWSV__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRAWSV__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RxmRAWSV {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RxmRAWSV where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/RxmRAWSV";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRAWSV() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRAWXMeas() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__RxmRAWXMeas__init(msg: *mut RxmRAWXMeas) -> bool;
    fn ublox_msgs__msg__RxmRAWXMeas__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RxmRAWXMeas>, size: usize) -> bool;
    fn ublox_msgs__msg__RxmRAWXMeas__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RxmRAWXMeas>);
    fn ublox_msgs__msg__RxmRAWXMeas__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RxmRAWXMeas>, out_seq: *mut rosidl_runtime_rs::Sequence<RxmRAWXMeas>) -> bool;
}

// Corresponds to ublox_msgs__msg__RxmRAWXMeas
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// see message RxmRAWX

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RxmRAWXMeas {
    /// Pseudorange measurement. GLONASS inter frequency
    /// channel delays are compensated with an internal
    /// calibration table.
    pub pr_mes: f64,

    /// Carrier phase measurement. The carrier
    /// phase initial ambiguity is initialized using an
    /// approximate value to make the magnitude of
    /// the phase close to the pseudorange
    /// measurement. Clock resets are applied to both
    /// phase and code measurements in accordance
    /// with the RINEX specification.
    pub cp_mes: f64,

    /// Doppler measurement (positive sign for
    /// approaching satellites)
    pub do_mes: f32,

    /// GNSS identifier (see CfgGNSS for constants)
    pub gnss_id: u8,

    /// Satellite identifier (see Satellite Numbering)
    pub sv_id: u8,

    /// Reserved
    pub reserved0: u8,

    /// Only used for GLONASS: This is the frequency
    /// slot + 7 (range from 0 to 13)
    pub freq_id: u8,

    /// Carrier phase locktime counter
    /// (maximum 64500 ms)
    pub locktime: u16,

    /// Carrier-to-noise density ratio (signal strength)
    /// [dB-Hz]
    pub cno: i8,

    /// Estimated pseudorange measurement standard
    /// deviation
    pub pr_stdev: u8,

    /// Estimated carrier phase measurement standard
    /// deviation (note a raw value of 0x0F indicates the
    /// value is invalid)
    pub cp_stdev: u8,

    /// Estimated Doppler measurement standard deviation
    /// [Hz / 0.002*2^n]
    pub do_stdev: u8,

    /// Tracking status bitfield
    pub trk_stat: u8,

    /// Reserved
    pub reserved1: u8,

}

impl RxmRAWXMeas {
    /// Pseudorange valid
    pub const TRK_STAT_PR_VALID: u8 = 1;

    /// Carrier phase valid
    pub const TRK_STAT_CP_VALID: u8 = 2;

    /// Half cycle valid
    pub const TRK_STAT_HALF_CYC: u8 = 4;

    /// Half cycle subtracted from phase
    pub const TRK_STAT_SUB_HALF_CYC: u8 = 8;

}


impl Default for RxmRAWXMeas {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__RxmRAWXMeas__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__RxmRAWXMeas__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RxmRAWXMeas {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRAWXMeas__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRAWXMeas__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRAWXMeas__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RxmRAWXMeas {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RxmRAWXMeas where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/RxmRAWXMeas";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRAWXMeas() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRAWX() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__RxmRAWX__init(msg: *mut RxmRAWX) -> bool;
    fn ublox_msgs__msg__RxmRAWX__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RxmRAWX>, size: usize) -> bool;
    fn ublox_msgs__msg__RxmRAWX__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RxmRAWX>);
    fn ublox_msgs__msg__RxmRAWX__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RxmRAWX>, out_seq: *mut rosidl_runtime_rs::Sequence<RxmRAWX>) -> bool;
}

// Corresponds to ublox_msgs__msg__RxmRAWX
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// RXM-RAWX (0x02 0x15)
/// Multi-GNSS Raw Measurement Data
///
/// This message contains the information needed to be able to generate a RINEX 3
/// multi-GNSS observation file.
/// This message contains pseudorange, Doppler, carrier phase, phase lock and
/// signal quality information for GNSS satellites once signals have been
/// synchronized. This message supports all active GNSS.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RxmRAWX {
    /// Measurement time of week in receiver local time
    /// approximately aligned to the GPS time system. The
    /// receiver local time of week number and leap second
    /// information can be used to translate the time to other
    /// time systems. More information about the difference in
    /// time systems can be found in RINEX 3 documentation.
    /// For a receiver operating in GLONASS only mode, UTC
    /// time can be determined by subtracting the leapS field
    /// from GPS time regardless of whether the GPS leap
    /// seconds are valid.
    pub rcv_tow: f64,

    /// GPS week number in receiver local time.
    pub week: u16,

    /// GPS leap seconds (GPS-UTC).
    /// This field represents the receiver's best knowledge of
    /// the leap seconds offset. A flag is given in the
    /// recStat bitfield to indicate if the leap seconds
    /// are known.
    pub leap_s: i8,

    /// # of measurements to follow
    pub num_meas: u8,

    /// Receiver tracking status bitfield
    pub rec_stat: u8,

    /// Message version (0x01 for this version).
    pub version: u8,

    /// Reserved
    pub reserved1: [u8; 2],


    // This member is not documented.
    #[allow(missing_docs)]
    pub meas: rosidl_runtime_rs::Sequence<super::super::msg::rmw::RxmRAWXMeas>,

}

impl RxmRAWX {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 21;

    /// Leap seconds have been determined
    pub const REC_STAT_LEAP_SEC: u8 = 1;

    /// Clock reset applied. Typically the receiver
    /// clock is changed in increments of integer
    /// milliseconds.
    pub const REC_STAT_CLK_RESET: u8 = 2;

}


impl Default for RxmRAWX {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__RxmRAWX__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__RxmRAWX__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RxmRAWX {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRAWX__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRAWX__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRAWX__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RxmRAWX {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RxmRAWX where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/RxmRAWX";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRAWX() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRTCM() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__RxmRTCM__init(msg: *mut RxmRTCM) -> bool;
    fn ublox_msgs__msg__RxmRTCM__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RxmRTCM>, size: usize) -> bool;
    fn ublox_msgs__msg__RxmRTCM__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RxmRTCM>);
    fn ublox_msgs__msg__RxmRTCM__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RxmRTCM>, out_seq: *mut rosidl_runtime_rs::Sequence<RxmRTCM>) -> bool;
}

// Corresponds to ublox_msgs__msg__RxmRTCM
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// RXM-RTCM (0x02 0x32)
/// RTCM input status
///
/// Output upon processing of an RTCM input message
/// Supported on:
/// - u-blox 8 / u-blox M8 from protocol version 20.01 up to version 23.01

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RxmRTCM {
    /// Message version (0x02 for this version)
    pub version: u8,

    /// RTCM input status flags
    pub flags: u8,

    /// Reserved
    pub reserved0: [u8; 2],

    /// Reference station ID
    pub ref_station: u16,

    /// Message type
    pub msg_type: u16,

}

impl RxmRTCM {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 50;

    /// 0 when RTCM message received and passed CRC
    /// check, 1 when failed in which case refStation
    /// and msgType might be corrupted and misleading
    pub const FLAGS_CRC_FAILED: u8 = 1;

}


impl Default for RxmRTCM {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__RxmRTCM__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__RxmRTCM__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RxmRTCM {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRTCM__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRTCM__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmRTCM__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RxmRTCM {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RxmRTCM where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/RxmRTCM";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmRTCM() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmSFRB() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__RxmSFRB__init(msg: *mut RxmSFRB) -> bool;
    fn ublox_msgs__msg__RxmSFRB__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RxmSFRB>, size: usize) -> bool;
    fn ublox_msgs__msg__RxmSFRB__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RxmSFRB>);
    fn ublox_msgs__msg__RxmSFRB__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RxmSFRB>, out_seq: *mut rosidl_runtime_rs::Sequence<RxmSFRB>) -> bool;
}

// Corresponds to ublox_msgs__msg__RxmSFRB
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// RXM-SFRB (0x02 0x11)
/// Subframe Buffer
///
/// The content of one single subframe buffer
/// For GPS satellites, the 10 dwrd values contain the parity checked subframe 
/// data for 10 Words. Each dwrd has 24 Bits with valid data (Bits 23 to 0). The 
/// remaining 8 bits (31 to 24) have an undefined value. The direction within the 
/// Word is that the higher order bits are received from the SV first. Example: 
/// The Preamble can be found in dwrd[0], at bit position 23 down to 16. For more 
/// details on the data format please refer to the ICD-GPS-200C
/// Interface document.
/// For SBAS satellites, the 250 Bit message block can be found in dwrd[0] to 
/// dwrd[6] for the first 224 bits. The remaining 26 bits are in dwrd[7], whereas
/// Bits 25 and 24 are the last two data bits, and Bits 23 down to 0 are the
/// parity bits. For more information on SBAS data format, please refer to 
/// RTCA/DO-229C (MOPS), Appendix A.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RxmSFRB {
    /// Channel Number
    pub chn: u8,

    /// ID of Satellite transmitting Subframe
    pub svid: u8,

    /// Words of Data
    pub dwrd: [u32; 10],

}

impl RxmSFRB {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 17;

}


impl Default for RxmSFRB {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__RxmSFRB__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__RxmSFRB__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RxmSFRB {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmSFRB__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmSFRB__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmSFRB__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RxmSFRB {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RxmSFRB where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/RxmSFRB";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmSFRB() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmSFRBX() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__RxmSFRBX__init(msg: *mut RxmSFRBX) -> bool;
    fn ublox_msgs__msg__RxmSFRBX__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RxmSFRBX>, size: usize) -> bool;
    fn ublox_msgs__msg__RxmSFRBX__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RxmSFRBX>);
    fn ublox_msgs__msg__RxmSFRBX__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RxmSFRBX>, out_seq: *mut rosidl_runtime_rs::Sequence<RxmSFRBX>) -> bool;
}

// Corresponds to ublox_msgs__msg__RxmSFRBX
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// RXM-SFRB (0x02 0x13)
/// Subframe Buffer
///
/// This message reports a complete subframe of broadcast navigation data decoded
/// from a single signal. The number of data words reported in each message
/// depends on the nature of the signal. See the section on Broadcast Navigation
/// Data for further details.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RxmSFRBX {
    /// GNSS identifier (see Cfg GNSS for constants)
    pub gnss_id: u8,

    /// Satellite identifier within corresponding GNSS system
    pub sv_id: u8,

    /// Reserved
    pub reserved0: u8,

    /// Only used for GLONASS: This is the frequency
    /// slot + 7 (range from 0 to 13)
    pub freq_id: u8,

    /// The number of data words contained in this message (up
    /// to 10, for currently supported signals)
    pub num_words: u8,

    /// The tracking channel number the message was received
    /// on
    pub chn: u8,

    /// Message version, (0x02 for this version)
    pub version: u8,

    /// Reserved
    pub reserved1: u8,

    /// Start of repeated block (numWords times)
    /// The data words
    pub dwrd: rosidl_runtime_rs::Sequence<u32>,

}

impl RxmSFRBX {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 19;

}


impl Default for RxmSFRBX {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__RxmSFRBX__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__RxmSFRBX__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RxmSFRBX {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmSFRBX__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmSFRBX__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmSFRBX__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RxmSFRBX {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RxmSFRBX where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/RxmSFRBX";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmSFRBX() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmSVSI() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__RxmSVSI__init(msg: *mut RxmSVSI) -> bool;
    fn ublox_msgs__msg__RxmSVSI__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RxmSVSI>, size: usize) -> bool;
    fn ublox_msgs__msg__RxmSVSI__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RxmSVSI>);
    fn ublox_msgs__msg__RxmSVSI__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RxmSVSI>, out_seq: *mut rosidl_runtime_rs::Sequence<RxmSVSI>) -> bool;
}

// Corresponds to ublox_msgs__msg__RxmSVSI
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// RXM-SVSI (0x02 0x20)
/// SV Status Info
///
/// Status of the receiver manager knowledge about GPS Orbit Validity
///
/// This message has only been retained for backwards compatibility; users are
/// recommended to use the UBX-NAV-ORB message in preference.

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RxmSVSI {
    /// GPS time of week of the navigation epoch
    pub i_tow: i32,

    /// GPS week number of the navigation epoch
    pub week: i16,

    /// Number of visible satellites
    pub num_vis: u8,

    /// Number of per-SV data blocks following
    pub num_sv: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub sv: rosidl_runtime_rs::Sequence<super::super::msg::rmw::RxmSVSISV>,

}

impl RxmSVSI {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 32;

}


impl Default for RxmSVSI {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__RxmSVSI__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__RxmSVSI__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RxmSVSI {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmSVSI__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmSVSI__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmSVSI__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RxmSVSI {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RxmSVSI where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/RxmSVSI";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmSVSI() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmSVSISV() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__RxmSVSISV__init(msg: *mut RxmSVSISV) -> bool;
    fn ublox_msgs__msg__RxmSVSISV__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RxmSVSISV>, size: usize) -> bool;
    fn ublox_msgs__msg__RxmSVSISV__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RxmSVSISV>);
    fn ublox_msgs__msg__RxmSVSISV__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RxmSVSISV>, out_seq: *mut rosidl_runtime_rs::Sequence<RxmSVSISV>) -> bool;
}

// Corresponds to ublox_msgs__msg__RxmSVSISV
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// see message RxmSVSI

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RxmSVSISV {
    /// Satellite ID
    pub svid: u8,

    /// Information Flags
    pub sv_flag: u8,

    /// Azimuth
    pub azim: i16,

    /// Elevation
    pub elev: i8,

    /// Age of Almanac and Ephemeris
    pub age: u8,

}

impl RxmSVSISV {
    /// Figure of Merit (URA) range 0..15
    pub const FLAG_URA_MASK: u8 = 15;

    /// SV healthy flag
    pub const FLAG_HEALTHY: u8 = 16;

    /// Ephemeris valid
    pub const FLAG_EPH_VAL: u8 = 32;

    /// Almanac valid
    pub const FLAG_ALM_VAL: u8 = 64;

    /// SV not available
    pub const FLAG_NOT_AVAIL: u8 = 128;

    /// Age of ALM in days offset by 4
    /// i.e. the reference time may be in the future:
    /// ageOfAlm = (age & 0x0f) - 4
    pub const AGE_ALM_MASK: u8 = 15;

    /// Age of EPH in hours offset by 4.
    /// i.e. the reference time may be in the future:
    /// ageOfEph = ((age & 0xf0) >> 4) - 4
    pub const AGE_EPH_MASK: u8 = 240;

}


impl Default for RxmSVSISV {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__RxmSVSISV__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__RxmSVSISV__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RxmSVSISV {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmSVSISV__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmSVSISV__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__RxmSVSISV__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RxmSVSISV {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RxmSVSISV where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/RxmSVSISV";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__RxmSVSISV() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__TimTM2() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__TimTM2__init(msg: *mut TimTM2) -> bool;
    fn ublox_msgs__msg__TimTM2__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<TimTM2>, size: usize) -> bool;
    fn ublox_msgs__msg__TimTM2__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<TimTM2>);
    fn ublox_msgs__msg__TimTM2__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<TimTM2>, out_seq: *mut rosidl_runtime_rs::Sequence<TimTM2>) -> bool;
}

// Corresponds to ublox_msgs__msg__TimTM2
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// TIM-TM2 (0x0D, 0x03)
/// Time mark data
///
/// Description for details.
///
/// Supported on:
///  - u-blox 8 / u-blox M8 with protocol version 22 (only with Timing Products)

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct TimTM2 {
    /// Channel (i.e. EXTINT) upon which the pulse was measured
    pub ch: u8,

    /// Bitmask [newRisingEdge, time, utc, timeBase, , newFallingEdge, run, mode]
    pub flags: u8,

    /// rising edge count
    pub rising_edge_count: u16,

    /// week number of last rising edge
    pub wn_r: u16,

    /// week number of last falling edge
    pub wn_f: u16,

    /// Tow of rising edge
    pub tow_ms_r: u32,

    /// Millisecond Fraction of Tow of rising edge in nanoseconds
    pub tow_sub_ms_r: u32,

    /// tow of falling edge
    pub tow_ms_f: u32,

    /// millisecond fraction of tow of falling edge in nanoseconds
    pub tow_sub_ms_f: u32,

    /// Accuracy estimate
    pub acc_est: u32,

}

impl TimTM2 {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 13;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 3;

    /// single = 0, running = 1
    pub const FLAGS_MODE_RUNNING: u8 = 1;

    /// armed = 0, stopped = 1
    pub const FLAGS_RUN: u8 = 2;

    /// new falling edge detected
    pub const FLAGS_NEWFALLINGEDGE: u8 = 4;

    /// 0 = time base is receiver time, 1 = time base is GNSS Time (according to the configuration in CFG-TP5 for tpldx= 0)
    pub const FLAGS_TIMEBASE_GNSS: u8 = 8;

    /// Time Base is UTC (the variant according to the configuration in CFG-NAV5
    pub const FLAGS_TIMEBASE_UTC: u8 = 16;

    /// 0 = utc not available, 1 = utc available
    pub const FLAGS_UTC_AVAIL: u8 = 32;

    /// 0 = time is not valid, 1 time is valid
    pub const FLAGS_TIME_VALID: u8 = 64;

    /// new rising edge detected
    pub const FLAGS_NEWRISINGEDGE: u8 = 128;

}


impl Default for TimTM2 {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__TimTM2__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__TimTM2__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for TimTM2 {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__TimTM2__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__TimTM2__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__TimTM2__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for TimTM2 {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for TimTM2 where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/TimTM2";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__TimTM2() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__UpdSOSAck() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__UpdSOSAck__init(msg: *mut UpdSOSAck) -> bool;
    fn ublox_msgs__msg__UpdSOSAck__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UpdSOSAck>, size: usize) -> bool;
    fn ublox_msgs__msg__UpdSOSAck__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UpdSOSAck>);
    fn ublox_msgs__msg__UpdSOSAck__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UpdSOSAck>, out_seq: *mut rosidl_runtime_rs::Sequence<UpdSOSAck>) -> bool;
}

// Corresponds to ublox_msgs__msg__UpdSOSAck
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// UPD-SOS (0x09 0x14)
///
/// Backup File Creation Acknowledge / System Restored from Backup
///
/// Firmware Supported on:
/// u-blox 8 / u-blox M8 from protocol version 15 up to version 23.01

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UpdSOSAck {
    /// Command
    pub cmd: u8,

    /// Reserved
    pub reserved0: [u8; 3],

    /// Response:
    pub response: u8,

    /// Reserved
    pub reserved1: [u8; 3],

}

impl UpdSOSAck {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 9;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 20;

    /// Backup File Creation Acknowledge
    /// The message is sent from the device as
    /// confirmation of creation of a backup file
    /// in flash. The host can safely shut down the
    /// device after received this message.
    pub const CMD_BACKUP_CREATE_ACK: u8 = 2;

    /// System Restored from Backup
    /// The message is sent from the device to
    /// notify the host the BBR has been restored
    /// from a backup file in flash. The host
    /// should clear the backup file after
    /// receiving this message. If the UBX-UPD-SOS
    /// message is polled, this message will be
    /// present.
    pub const CMD_SYSTEM_RESTORED: u8 = 3;

    /// Not acknowledged
    pub const BACKUP_CREATE_NACK: u8 = 0;

    /// Acknowledged
    pub const BACKUP_CREATE_ACK: u8 = 1;

    /// Unknown
    pub const SYSTEM_RESTORED_RESPONSE_UNKNOWN: u8 = 0;

    /// Failed restoring from backup
    /// file
    pub const SYSTEM_RESTORED_RESPONSE_FAILED: u8 = 1;

    /// Restored from backup file
    pub const SYSTEM_RESTORED_RESPONSE_RESTORED: u8 = 2;

    /// Not restored (no backup)
    pub const SYSTEM_RESTORED_RESPONSE_NOT_RESTORED: u8 = 3;

}


impl Default for UpdSOSAck {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__UpdSOSAck__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__UpdSOSAck__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UpdSOSAck {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__UpdSOSAck__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__UpdSOSAck__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__UpdSOSAck__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UpdSOSAck {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UpdSOSAck where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/UpdSOSAck";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__UpdSOSAck() }
  }
}


#[link(name = "ublox_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__UpdSOS() -> *const std::ffi::c_void;
}

#[link(name = "ublox_msgs__rosidl_generator_c")]
extern "C" {
    fn ublox_msgs__msg__UpdSOS__init(msg: *mut UpdSOS) -> bool;
    fn ublox_msgs__msg__UpdSOS__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UpdSOS>, size: usize) -> bool;
    fn ublox_msgs__msg__UpdSOS__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UpdSOS>);
    fn ublox_msgs__msg__UpdSOS__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UpdSOS>, out_seq: *mut rosidl_runtime_rs::Sequence<UpdSOS>) -> bool;
}

// Corresponds to ublox_msgs__msg__UpdSOS
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// UPD-SOS (0x09 0x14)
///
/// Firmware Supported on:
/// u-blox 8 / u-blox M8 from protocol version 15 up to version 23.01

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UpdSOS {
    /// Command
    pub cmd: u8,

    /// Reserved
    pub reserved1: [u8; 3],

}

impl UpdSOS {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 9;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const MESSAGE_ID: u8 = 20;

    /// The host can send this message in order to save part of the BBR memory in a
    /// file in flash file system. The feature is designed in order to emulate the
    /// presence of the backup battery even if it is not present; the host can issue
    /// the save on shutdown command before switching off the device supply. It is
    /// recommended to issue a GNSS stop command before, in order to keep the BBR
    /// memory content consistent.
    /// Create Backup File in Flash
    pub const CMD_FLASH_BACKUP_CREATE: u8 = 0;

    /// The host can send this message in order to erase the backup file present in
    /// flash. It is recommended that the clear operation is issued after the host has
    /// received the notification that the memory has been restored after a reset.
    /// Alternatively the host can parse the startup string 'Restored data saved on
    /// shutdown' or poll the UBX-UPD-SOS message for getting the status.
    /// Clear Backup File in Flash
    pub const CMD_FLASH_BACKUP_CLEAR: u8 = 1;

}


impl Default for UpdSOS {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !ublox_msgs__msg__UpdSOS__init(&mut msg as *mut _) {
        panic!("Call to ublox_msgs__msg__UpdSOS__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UpdSOS {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__UpdSOS__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__UpdSOS__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { ublox_msgs__msg__UpdSOS__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UpdSOS {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UpdSOS where Self: Sized {
  const TYPE_NAME: &'static str = "ublox_msgs/msg/UpdSOS";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__ublox_msgs__msg__UpdSOS() }
  }
}


