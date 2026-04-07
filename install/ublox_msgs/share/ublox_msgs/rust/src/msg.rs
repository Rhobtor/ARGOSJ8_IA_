#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to ublox_msgs__msg__Ack
/// ACK (0x05)
/// Message Acknowledged / Not-Acknowledged
///
/// Output upon processing of an input message

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Ack::default())
  }
}

impl rosidl_runtime_rs::Message for Ack {
  type RmwMsg = super::msg::rmw::Ack;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        cls_id: msg.cls_id,
        msg_id: msg.msg_id,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      cls_id: msg.cls_id,
      msg_id: msg.msg_id,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      cls_id: msg.cls_id,
      msg_id: msg.msg_id,
    }
  }
}


// Corresponds to ublox_msgs__msg__AidALM
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct AidALM {
    /// SV ID for which the receiver shall return its
    /// Almanac Data (Valid Range: 1 .. 32 or 51, 56, 63).
    pub svid: u32,

    /// Issue Date of Almanach (GPS week number)
    pub week: u32,

    /// Start of optional block
    /// Almanac Words
    pub dwrd: Vec<u32>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AidALM::default())
  }
}

impl rosidl_runtime_rs::Message for AidALM {
  type RmwMsg = super::msg::rmw::AidALM;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        svid: msg.svid,
        week: msg.week,
        dwrd: msg.dwrd.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      svid: msg.svid,
      week: msg.week,
        dwrd: msg.dwrd.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      svid: msg.svid,
      week: msg.week,
      dwrd: msg.dwrd
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__AidEPH
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub sf1d: Vec<u32>,

    /// Subframe 2 Words 3..10 (SF2D0..SF2D7)
    pub sf2d: Vec<u32>,

    /// Subframe 3 Words 3..10 (SF3D0..SF3D7)
    pub sf3d: Vec<u32>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AidEPH::default())
  }
}

impl rosidl_runtime_rs::Message for AidEPH {
  type RmwMsg = super::msg::rmw::AidEPH;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        svid: msg.svid,
        how: msg.how,
        sf1d: msg.sf1d.into(),
        sf2d: msg.sf2d.into(),
        sf3d: msg.sf3d.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      svid: msg.svid,
      how: msg.how,
        sf1d: msg.sf1d.as_slice().into(),
        sf2d: msg.sf2d.as_slice().into(),
        sf3d: msg.sf3d.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      svid: msg.svid,
      how: msg.how,
      sf1d: msg.sf1d
          .into_iter()
          .collect(),
      sf2d: msg.sf2d
          .into_iter()
          .collect(),
      sf3d: msg.sf3d
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__AidHUI
/// AID-HUI (0x0B 0x02)
/// GPS Health, UTC and ionosphere parameters
///
/// All UBX-AID messages are deprecated; use UBX-MGA messages instead.
/// This message contains a health bit mask, UTC time and Klobuchar parameters. For more
/// information on these parameters, please see the ICD-GPS-200 documentation.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::AidHUI::default())
  }
}

impl rosidl_runtime_rs::Message for AidHUI {
  type RmwMsg = super::msg::rmw::AidHUI;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        health: msg.health,
        utc_a0: msg.utc_a0,
        utc_a1: msg.utc_a1,
        utc_tow: msg.utc_tow,
        utc_wnt: msg.utc_wnt,
        utc_ls: msg.utc_ls,
        utc_wnf: msg.utc_wnf,
        utc_dn: msg.utc_dn,
        utc_lsf: msg.utc_lsf,
        utc_spare: msg.utc_spare,
        klob_a0: msg.klob_a0,
        klob_a1: msg.klob_a1,
        klob_a2: msg.klob_a2,
        klob_a3: msg.klob_a3,
        klob_b0: msg.klob_b0,
        klob_b1: msg.klob_b1,
        klob_b2: msg.klob_b2,
        klob_b3: msg.klob_b3,
        flags: msg.flags,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      health: msg.health,
      utc_a0: msg.utc_a0,
      utc_a1: msg.utc_a1,
      utc_tow: msg.utc_tow,
      utc_wnt: msg.utc_wnt,
      utc_ls: msg.utc_ls,
      utc_wnf: msg.utc_wnf,
      utc_dn: msg.utc_dn,
      utc_lsf: msg.utc_lsf,
      utc_spare: msg.utc_spare,
      klob_a0: msg.klob_a0,
      klob_a1: msg.klob_a1,
      klob_a2: msg.klob_a2,
      klob_a3: msg.klob_a3,
      klob_b0: msg.klob_b0,
      klob_b1: msg.klob_b1,
      klob_b2: msg.klob_b2,
      klob_b3: msg.klob_b3,
      flags: msg.flags,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      health: msg.health,
      utc_a0: msg.utc_a0,
      utc_a1: msg.utc_a1,
      utc_tow: msg.utc_tow,
      utc_wnt: msg.utc_wnt,
      utc_ls: msg.utc_ls,
      utc_wnf: msg.utc_wnf,
      utc_dn: msg.utc_dn,
      utc_lsf: msg.utc_lsf,
      utc_spare: msg.utc_spare,
      klob_a0: msg.klob_a0,
      klob_a1: msg.klob_a1,
      klob_a2: msg.klob_a2,
      klob_a3: msg.klob_a3,
      klob_b0: msg.klob_b0,
      klob_b1: msg.klob_b1,
      klob_b2: msg.klob_b2,
      klob_b3: msg.klob_b3,
      flags: msg.flags,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgANT
/// CFG-ANT (0x06 0x13)
/// Antenna Control Settings

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgANT::default())
  }
}

impl rosidl_runtime_rs::Message for CfgANT {
  type RmwMsg = super::msg::rmw::CfgANT;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        flags: msg.flags,
        pins: msg.pins,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      flags: msg.flags,
      pins: msg.pins,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      flags: msg.flags,
      pins: msg.pins,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgCFG
/// CFG-CFG (0x06 0x09)
/// Clear, Save and Load configurations

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgCFG::default())
  }
}

impl rosidl_runtime_rs::Message for CfgCFG {
  type RmwMsg = super::msg::rmw::CfgCFG;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        clear_mask: msg.clear_mask,
        save_mask: msg.save_mask,
        load_mask: msg.load_mask,
        device_mask: msg.device_mask,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      clear_mask: msg.clear_mask,
      save_mask: msg.save_mask,
      load_mask: msg.load_mask,
      device_mask: msg.device_mask,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      clear_mask: msg.clear_mask,
      save_mask: msg.save_mask,
      load_mask: msg.load_mask,
      device_mask: msg.device_mask,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgDAT
/// CFG-DAT (0x06 0x06)
/// Set User-defined Datum
///
/// For more information see the description of Geodetic Systems and Frames.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgDAT::default())
  }
}

impl rosidl_runtime_rs::Message for CfgDAT {
  type RmwMsg = super::msg::rmw::CfgDAT;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        datum_num: msg.datum_num,
        datum_name: msg.datum_name,
        maj_a: msg.maj_a,
        flat: msg.flat,
        d_x: msg.d_x,
        d_y: msg.d_y,
        d_z: msg.d_z,
        rot_x: msg.rot_x,
        rot_y: msg.rot_y,
        rot_z: msg.rot_z,
        scale: msg.scale,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      datum_num: msg.datum_num,
        datum_name: msg.datum_name,
      maj_a: msg.maj_a,
      flat: msg.flat,
      d_x: msg.d_x,
      d_y: msg.d_y,
      d_z: msg.d_z,
      rot_x: msg.rot_x,
      rot_y: msg.rot_y,
      rot_z: msg.rot_z,
      scale: msg.scale,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      datum_num: msg.datum_num,
      datum_name: msg.datum_name,
      maj_a: msg.maj_a,
      flat: msg.flat,
      d_x: msg.d_x,
      d_y: msg.d_y,
      d_z: msg.d_z,
      rot_x: msg.rot_x,
      rot_y: msg.rot_y,
      rot_z: msg.rot_z,
      scale: msg.scale,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgDGNSS
/// CFG-DGNSS (0x06 0x70)
/// DGNSS configuration
///
/// This message allows the user to configure the DGNSS configuration of the
/// receiver.
/// Supported on:
///  - u-blox 8 / u-blox M8 from protocol version 20.01 up to version 23.01 (only
///    with High Precision GNSS products)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgDGNSS::default())
  }
}

impl rosidl_runtime_rs::Message for CfgDGNSS {
  type RmwMsg = super::msg::rmw::CfgDGNSS;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        dgnss_mode: msg.dgnss_mode,
        reserved0: msg.reserved0,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      dgnss_mode: msg.dgnss_mode,
        reserved0: msg.reserved0,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      dgnss_mode: msg.dgnss_mode,
      reserved0: msg.reserved0,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgGNSSBlock
/// see Cfg-GNSS message

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgGNSSBlock::default())
  }
}

impl rosidl_runtime_rs::Message for CfgGNSSBlock {
  type RmwMsg = super::msg::rmw::CfgGNSSBlock;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        gnss_id: msg.gnss_id,
        res_trk_ch: msg.res_trk_ch,
        max_trk_ch: msg.max_trk_ch,
        reserved1: msg.reserved1,
        flags: msg.flags,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      gnss_id: msg.gnss_id,
      res_trk_ch: msg.res_trk_ch,
      max_trk_ch: msg.max_trk_ch,
      reserved1: msg.reserved1,
      flags: msg.flags,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      gnss_id: msg.gnss_id,
      res_trk_ch: msg.res_trk_ch,
      max_trk_ch: msg.max_trk_ch,
      reserved1: msg.reserved1,
      flags: msg.flags,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgGNSS
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub blocks: Vec<super::msg::CfgGNSSBlock>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgGNSS::default())
  }
}

impl rosidl_runtime_rs::Message for CfgGNSS {
  type RmwMsg = super::msg::rmw::CfgGNSS;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        msg_ver: msg.msg_ver,
        num_trk_ch_hw: msg.num_trk_ch_hw,
        num_trk_ch_use: msg.num_trk_ch_use,
        num_config_blocks: msg.num_config_blocks,
        blocks: msg.blocks
          .into_iter()
          .map(|elem| super::msg::CfgGNSSBlock::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      msg_ver: msg.msg_ver,
      num_trk_ch_hw: msg.num_trk_ch_hw,
      num_trk_ch_use: msg.num_trk_ch_use,
      num_config_blocks: msg.num_config_blocks,
        blocks: msg.blocks
          .iter()
          .map(|elem| super::msg::CfgGNSSBlock::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      msg_ver: msg.msg_ver,
      num_trk_ch_hw: msg.num_trk_ch_hw,
      num_trk_ch_use: msg.num_trk_ch_use,
      num_config_blocks: msg.num_config_blocks,
      blocks: msg.blocks
          .into_iter()
          .map(super::msg::CfgGNSSBlock::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgHNR
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgHNR::default())
  }
}

impl rosidl_runtime_rs::Message for CfgHNR {
  type RmwMsg = super::msg::rmw::CfgHNR;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        high_nav_rate: msg.high_nav_rate,
        reserved0: msg.reserved0,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      high_nav_rate: msg.high_nav_rate,
        reserved0: msg.reserved0,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      high_nav_rate: msg.high_nav_rate,
      reserved0: msg.reserved0,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgINFBlock
/// See CfgINF message

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgINFBlock::default())
  }
}

impl rosidl_runtime_rs::Message for CfgINFBlock {
  type RmwMsg = super::msg::rmw::CfgINFBlock;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        protocol_id: msg.protocol_id,
        reserved1: msg.reserved1,
        inf_msg_mask: msg.inf_msg_mask,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      protocol_id: msg.protocol_id,
        reserved1: msg.reserved1,
        inf_msg_mask: msg.inf_msg_mask,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      protocol_id: msg.protocol_id,
      reserved1: msg.reserved1,
      inf_msg_mask: msg.inf_msg_mask,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgINF
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CfgINF {
    /// start of repeated block
    pub blocks: Vec<super::msg::CfgINFBlock>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgINF::default())
  }
}

impl rosidl_runtime_rs::Message for CfgINF {
  type RmwMsg = super::msg::rmw::CfgINF;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        blocks: msg.blocks
          .into_iter()
          .map(|elem| super::msg::CfgINFBlock::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        blocks: msg.blocks
          .iter()
          .map(|elem| super::msg::CfgINFBlock::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      blocks: msg.blocks
          .into_iter()
          .map(super::msg::CfgINFBlock::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgMSG
/// CFG-MSG (0x06 0x01)
/// Message Rate(s)
///
/// Set message rate for the current port

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgMSG::default())
  }
}

impl rosidl_runtime_rs::Message for CfgMSG {
  type RmwMsg = super::msg::rmw::CfgMSG;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        msg_class: msg.msg_class,
        msg_id: msg.msg_id,
        rate: msg.rate,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      msg_class: msg.msg_class,
      msg_id: msg.msg_id,
      rate: msg.rate,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      msg_class: msg.msg_class,
      msg_id: msg.msg_id,
      rate: msg.rate,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgNAV5
/// CFG-NAV5 (0x06 0x24)
/// Navigation Engine Settings

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgNAV5::default())
  }
}

impl rosidl_runtime_rs::Message for CfgNAV5 {
  type RmwMsg = super::msg::rmw::CfgNAV5;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        mask: msg.mask,
        dyn_model: msg.dyn_model,
        fix_mode: msg.fix_mode,
        fixed_alt: msg.fixed_alt,
        fixed_alt_var: msg.fixed_alt_var,
        min_elev: msg.min_elev,
        dr_limit: msg.dr_limit,
        p_dop: msg.p_dop,
        t_dop: msg.t_dop,
        p_acc: msg.p_acc,
        t_acc: msg.t_acc,
        static_hold_thresh: msg.static_hold_thresh,
        dgnss_time_out: msg.dgnss_time_out,
        cno_thresh_num_svs: msg.cno_thresh_num_svs,
        cno_thresh: msg.cno_thresh,
        reserved1: msg.reserved1,
        static_hold_max_dist: msg.static_hold_max_dist,
        utc_standard: msg.utc_standard,
        reserved2: msg.reserved2,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      mask: msg.mask,
      dyn_model: msg.dyn_model,
      fix_mode: msg.fix_mode,
      fixed_alt: msg.fixed_alt,
      fixed_alt_var: msg.fixed_alt_var,
      min_elev: msg.min_elev,
      dr_limit: msg.dr_limit,
      p_dop: msg.p_dop,
      t_dop: msg.t_dop,
      p_acc: msg.p_acc,
      t_acc: msg.t_acc,
      static_hold_thresh: msg.static_hold_thresh,
      dgnss_time_out: msg.dgnss_time_out,
      cno_thresh_num_svs: msg.cno_thresh_num_svs,
      cno_thresh: msg.cno_thresh,
        reserved1: msg.reserved1,
      static_hold_max_dist: msg.static_hold_max_dist,
      utc_standard: msg.utc_standard,
        reserved2: msg.reserved2,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      mask: msg.mask,
      dyn_model: msg.dyn_model,
      fix_mode: msg.fix_mode,
      fixed_alt: msg.fixed_alt,
      fixed_alt_var: msg.fixed_alt_var,
      min_elev: msg.min_elev,
      dr_limit: msg.dr_limit,
      p_dop: msg.p_dop,
      t_dop: msg.t_dop,
      p_acc: msg.p_acc,
      t_acc: msg.t_acc,
      static_hold_thresh: msg.static_hold_thresh,
      dgnss_time_out: msg.dgnss_time_out,
      cno_thresh_num_svs: msg.cno_thresh_num_svs,
      cno_thresh: msg.cno_thresh,
      reserved1: msg.reserved1,
      static_hold_max_dist: msg.static_hold_max_dist,
      utc_standard: msg.utc_standard,
      reserved2: msg.reserved2,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgNAVX5
/// CFG-NAVX5 (0x06 0x23)
/// Navigation Engine Expert Settings
///
/// Warning: Refer to u-blox protocol spec before changing these settings.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgNAVX5::default())
  }
}

impl rosidl_runtime_rs::Message for CfgNAVX5 {
  type RmwMsg = super::msg::rmw::CfgNAVX5;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        version: msg.version,
        mask1: msg.mask1,
        mask2: msg.mask2,
        reserved1: msg.reserved1,
        min_svs: msg.min_svs,
        max_svs: msg.max_svs,
        min_cno: msg.min_cno,
        reserved2: msg.reserved2,
        ini_fix3d: msg.ini_fix3d,
        reserved3: msg.reserved3,
        ack_aiding: msg.ack_aiding,
        wkn_rollover: msg.wkn_rollover,
        sig_atten_comp_mode: msg.sig_atten_comp_mode,
        reserved4: msg.reserved4,
        use_ppp: msg.use_ppp,
        aop_cfg: msg.aop_cfg,
        reserved5: msg.reserved5,
        aop_orb_max_err: msg.aop_orb_max_err,
        reserved6: msg.reserved6,
        use_adr: msg.use_adr,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      version: msg.version,
      mask1: msg.mask1,
      mask2: msg.mask2,
        reserved1: msg.reserved1,
      min_svs: msg.min_svs,
      max_svs: msg.max_svs,
      min_cno: msg.min_cno,
      reserved2: msg.reserved2,
      ini_fix3d: msg.ini_fix3d,
        reserved3: msg.reserved3,
      ack_aiding: msg.ack_aiding,
      wkn_rollover: msg.wkn_rollover,
      sig_atten_comp_mode: msg.sig_atten_comp_mode,
        reserved4: msg.reserved4,
      use_ppp: msg.use_ppp,
      aop_cfg: msg.aop_cfg,
        reserved5: msg.reserved5,
      aop_orb_max_err: msg.aop_orb_max_err,
        reserved6: msg.reserved6,
      use_adr: msg.use_adr,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      version: msg.version,
      mask1: msg.mask1,
      mask2: msg.mask2,
      reserved1: msg.reserved1,
      min_svs: msg.min_svs,
      max_svs: msg.max_svs,
      min_cno: msg.min_cno,
      reserved2: msg.reserved2,
      ini_fix3d: msg.ini_fix3d,
      reserved3: msg.reserved3,
      ack_aiding: msg.ack_aiding,
      wkn_rollover: msg.wkn_rollover,
      sig_atten_comp_mode: msg.sig_atten_comp_mode,
      reserved4: msg.reserved4,
      use_ppp: msg.use_ppp,
      aop_cfg: msg.aop_cfg,
      reserved5: msg.reserved5,
      aop_orb_max_err: msg.aop_orb_max_err,
      reserved6: msg.reserved6,
      use_adr: msg.use_adr,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgNMEA6
/// CFG-NMEA (0x06 0x17)
/// NMEA protocol configuration
///
/// Set/Get the NMEA protocol configuration. See section NMEA Protocol
/// Configuration for a detailed description of the configuration effects on
/// NMEA output
///
/// Supported on u-blox 6 from firmware version 6.00 up to version 7.03.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgNMEA6::default())
  }
}

impl rosidl_runtime_rs::Message for CfgNMEA6 {
  type RmwMsg = super::msg::rmw::CfgNMEA6;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        filter: msg.filter,
        version: msg.version,
        num_sv: msg.num_sv,
        flags: msg.flags,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      filter: msg.filter,
      version: msg.version,
      num_sv: msg.num_sv,
      flags: msg.flags,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      filter: msg.filter,
      version: msg.version,
      num_sv: msg.num_sv,
      flags: msg.flags,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgNMEA7
/// CFG-NMEA (0x06 0x17)
/// NMEA protocol configuration V0
///
/// Set/Get the NMEA protocol configuration. See section NMEA Protocol
/// Configuration for a detailed description of the configuration effects on
/// NMEA output
///
/// Supported on: u-blox 7 firmware version 1.00

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgNMEA7::default())
  }
}

impl rosidl_runtime_rs::Message for CfgNMEA7 {
  type RmwMsg = super::msg::rmw::CfgNMEA7;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        filter: msg.filter,
        nmea_version: msg.nmea_version,
        num_sv: msg.num_sv,
        flags: msg.flags,
        gnss_to_filter: msg.gnss_to_filter,
        sv_numbering: msg.sv_numbering,
        main_talker_id: msg.main_talker_id,
        gsv_talker_id: msg.gsv_talker_id,
        reserved: msg.reserved,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      filter: msg.filter,
      nmea_version: msg.nmea_version,
      num_sv: msg.num_sv,
      flags: msg.flags,
      gnss_to_filter: msg.gnss_to_filter,
      sv_numbering: msg.sv_numbering,
      main_talker_id: msg.main_talker_id,
      gsv_talker_id: msg.gsv_talker_id,
      reserved: msg.reserved,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      filter: msg.filter,
      nmea_version: msg.nmea_version,
      num_sv: msg.num_sv,
      flags: msg.flags,
      gnss_to_filter: msg.gnss_to_filter,
      sv_numbering: msg.sv_numbering,
      main_talker_id: msg.main_talker_id,
      gsv_talker_id: msg.gsv_talker_id,
      reserved: msg.reserved,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgNMEA
/// CFG-NMEA (0x06 0x17)
/// NMEA protocol configuration V1
///
/// Set/Get the NMEA protocol configuration. See section NMEA Protocol
/// Configuration for a detailed description of the configuration effects on
/// NMEA output

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgNMEA::default())
  }
}

impl rosidl_runtime_rs::Message for CfgNMEA {
  type RmwMsg = super::msg::rmw::CfgNMEA;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        filter: msg.filter,
        nmea_version: msg.nmea_version,
        num_sv: msg.num_sv,
        flags: msg.flags,
        gnss_to_filter: msg.gnss_to_filter,
        sv_numbering: msg.sv_numbering,
        main_talker_id: msg.main_talker_id,
        gsv_talker_id: msg.gsv_talker_id,
        version: msg.version,
        bds_talker_id: msg.bds_talker_id,
        reserved1: msg.reserved1,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      filter: msg.filter,
      nmea_version: msg.nmea_version,
      num_sv: msg.num_sv,
      flags: msg.flags,
      gnss_to_filter: msg.gnss_to_filter,
      sv_numbering: msg.sv_numbering,
      main_talker_id: msg.main_talker_id,
      gsv_talker_id: msg.gsv_talker_id,
      version: msg.version,
        bds_talker_id: msg.bds_talker_id,
        reserved1: msg.reserved1,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      filter: msg.filter,
      nmea_version: msg.nmea_version,
      num_sv: msg.num_sv,
      flags: msg.flags,
      gnss_to_filter: msg.gnss_to_filter,
      sv_numbering: msg.sv_numbering,
      main_talker_id: msg.main_talker_id,
      gsv_talker_id: msg.gsv_talker_id,
      version: msg.version,
      bds_talker_id: msg.bds_talker_id,
      reserved1: msg.reserved1,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgPRT
/// CFG-PRT (0x06 0x00)
/// Port Configuration for DDC, UART, USB, SPI
///
/// Several configurations can be concatenated to one input message. In this case
/// the payload length can be a multiple of the normal length (see the other
/// versions of CFG-PRT). Output messages from the module contain only one
/// configuration unit.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgPRT::default())
  }
}

impl rosidl_runtime_rs::Message for CfgPRT {
  type RmwMsg = super::msg::rmw::CfgPRT;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        port_id: msg.port_id,
        reserved0: msg.reserved0,
        tx_ready: msg.tx_ready,
        mode: msg.mode,
        baud_rate: msg.baud_rate,
        in_proto_mask: msg.in_proto_mask,
        out_proto_mask: msg.out_proto_mask,
        flags: msg.flags,
        reserved1: msg.reserved1,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      port_id: msg.port_id,
      reserved0: msg.reserved0,
      tx_ready: msg.tx_ready,
      mode: msg.mode,
      baud_rate: msg.baud_rate,
      in_proto_mask: msg.in_proto_mask,
      out_proto_mask: msg.out_proto_mask,
      flags: msg.flags,
      reserved1: msg.reserved1,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      port_id: msg.port_id,
      reserved0: msg.reserved0,
      tx_ready: msg.tx_ready,
      mode: msg.mode,
      baud_rate: msg.baud_rate,
      in_proto_mask: msg.in_proto_mask,
      out_proto_mask: msg.out_proto_mask,
      flags: msg.flags,
      reserved1: msg.reserved1,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgRATE
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgRATE::default())
  }
}

impl rosidl_runtime_rs::Message for CfgRATE {
  type RmwMsg = super::msg::rmw::CfgRATE;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        meas_rate: msg.meas_rate,
        nav_rate: msg.nav_rate,
        time_ref: msg.time_ref,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      meas_rate: msg.meas_rate,
      nav_rate: msg.nav_rate,
      time_ref: msg.time_ref,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      meas_rate: msg.meas_rate,
      nav_rate: msg.nav_rate,
      time_ref: msg.time_ref,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgRST
/// CFG-RST (0x06 0x04)
/// Reset Receiver / Clear Backup Data Structures
///
/// Don't expect this message to be acknowledged by the receiver.
///  - Newer FW version won't acknowledge this message at all.
///  - Older FW version will acknowledge this message but the acknowledge may not
///    be sent completely before the receiver is reset.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgRST::default())
  }
}

impl rosidl_runtime_rs::Message for CfgRST {
  type RmwMsg = super::msg::rmw::CfgRST;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        nav_bbr_mask: msg.nav_bbr_mask,
        reset_mode: msg.reset_mode,
        reserved1: msg.reserved1,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      nav_bbr_mask: msg.nav_bbr_mask,
      reset_mode: msg.reset_mode,
      reserved1: msg.reserved1,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      nav_bbr_mask: msg.nav_bbr_mask,
      reset_mode: msg.reset_mode,
      reserved1: msg.reserved1,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgSBAS
/// CFG-SBAS (0x06 0x16)
/// SBAS Configuration
///
/// This message configures the SBAS receiver subsystem (i.e. WAAS, EGNOS, MSAS).
/// See the SBAS Configuration Settings Description for a detailed description of
/// how these settings affect receiver operation

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgSBAS::default())
  }
}

impl rosidl_runtime_rs::Message for CfgSBAS {
  type RmwMsg = super::msg::rmw::CfgSBAS;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        mode: msg.mode,
        usage: msg.usage,
        max_sbas: msg.max_sbas,
        scanmode2: msg.scanmode2,
        scanmode1: msg.scanmode1,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      mode: msg.mode,
      usage: msg.usage,
      max_sbas: msg.max_sbas,
      scanmode2: msg.scanmode2,
      scanmode1: msg.scanmode1,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      mode: msg.mode,
      usage: msg.usage,
      max_sbas: msg.max_sbas,
      scanmode2: msg.scanmode2,
      scanmode1: msg.scanmode1,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgTMODE3
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgTMODE3::default())
  }
}

impl rosidl_runtime_rs::Message for CfgTMODE3 {
  type RmwMsg = super::msg::rmw::CfgTMODE3;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        version: msg.version,
        reserved1: msg.reserved1,
        flags: msg.flags,
        ecef_x_or_lat: msg.ecef_x_or_lat,
        ecef_y_or_lon: msg.ecef_y_or_lon,
        ecef_z_or_alt: msg.ecef_z_or_alt,
        ecef_x_or_lat_hp: msg.ecef_x_or_lat_hp,
        ecef_y_or_lon_hp: msg.ecef_y_or_lon_hp,
        ecef_z_or_alt_hp: msg.ecef_z_or_alt_hp,
        reserved2: msg.reserved2,
        fixed_pos_acc: msg.fixed_pos_acc,
        svin_min_dur: msg.svin_min_dur,
        svin_acc_limit: msg.svin_acc_limit,
        reserved3: msg.reserved3,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      version: msg.version,
      reserved1: msg.reserved1,
      flags: msg.flags,
      ecef_x_or_lat: msg.ecef_x_or_lat,
      ecef_y_or_lon: msg.ecef_y_or_lon,
      ecef_z_or_alt: msg.ecef_z_or_alt,
      ecef_x_or_lat_hp: msg.ecef_x_or_lat_hp,
      ecef_y_or_lon_hp: msg.ecef_y_or_lon_hp,
      ecef_z_or_alt_hp: msg.ecef_z_or_alt_hp,
      reserved2: msg.reserved2,
      fixed_pos_acc: msg.fixed_pos_acc,
      svin_min_dur: msg.svin_min_dur,
      svin_acc_limit: msg.svin_acc_limit,
        reserved3: msg.reserved3,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      version: msg.version,
      reserved1: msg.reserved1,
      flags: msg.flags,
      ecef_x_or_lat: msg.ecef_x_or_lat,
      ecef_y_or_lon: msg.ecef_y_or_lon,
      ecef_z_or_alt: msg.ecef_z_or_alt,
      ecef_x_or_lat_hp: msg.ecef_x_or_lat_hp,
      ecef_y_or_lon_hp: msg.ecef_y_or_lon_hp,
      ecef_z_or_alt_hp: msg.ecef_z_or_alt_hp,
      reserved2: msg.reserved2,
      fixed_pos_acc: msg.fixed_pos_acc,
      svin_min_dur: msg.svin_min_dur,
      svin_acc_limit: msg.svin_acc_limit,
      reserved3: msg.reserved3,
    }
  }
}


// Corresponds to ublox_msgs__msg__CfgUSB
/// UBX-CFG-USB (0x06 0x1B)
/// USB Configuration

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::CfgUSB::default())
  }
}

impl rosidl_runtime_rs::Message for CfgUSB {
  type RmwMsg = super::msg::rmw::CfgUSB;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        vendor_id: msg.vendor_id,
        product_id: msg.product_id,
        reserved1: msg.reserved1,
        reserved2: msg.reserved2,
        power_consumption: msg.power_consumption,
        flags: msg.flags,
        vendor_string: msg.vendor_string,
        product_string: msg.product_string,
        serial_number: msg.serial_number,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      vendor_id: msg.vendor_id,
      product_id: msg.product_id,
        reserved1: msg.reserved1,
        reserved2: msg.reserved2,
      power_consumption: msg.power_consumption,
      flags: msg.flags,
        vendor_string: msg.vendor_string,
        product_string: msg.product_string,
        serial_number: msg.serial_number,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      vendor_id: msg.vendor_id,
      product_id: msg.product_id,
      reserved1: msg.reserved1,
      reserved2: msg.reserved2,
      power_consumption: msg.power_consumption,
      flags: msg.flags,
      vendor_string: msg.vendor_string,
      product_string: msg.product_string,
      serial_number: msg.serial_number,
    }
  }
}


// Corresponds to ublox_msgs__msg__EsfINS
/// ESF-INS (0x10 0x15)
/// Vehicle dynamics information
///
/// This message outputs information about vehicle dynamics computed by the
/// Inertial Navigation System (INS) during ESF-based navigation.
/// For ADR products, the output dynamics information (angular rates and
/// accelerations) is expressed with respect to the vehicle-frame.
/// For UDR products, the output dynamics information (angular rates and
/// accelerations) is expressed with respect to the body-frame.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::EsfINS::default())
  }
}

impl rosidl_runtime_rs::Message for EsfINS {
  type RmwMsg = super::msg::rmw::EsfINS;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        bitfield0: msg.bitfield0,
        reserved1: msg.reserved1,
        i_tow: msg.i_tow,
        x_ang_rate: msg.x_ang_rate,
        y_ang_rate: msg.y_ang_rate,
        z_ang_rate: msg.z_ang_rate,
        x_accel: msg.x_accel,
        y_accel: msg.y_accel,
        z_accel: msg.z_accel,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      bitfield0: msg.bitfield0,
        reserved1: msg.reserved1,
      i_tow: msg.i_tow,
      x_ang_rate: msg.x_ang_rate,
      y_ang_rate: msg.y_ang_rate,
      z_ang_rate: msg.z_ang_rate,
      x_accel: msg.x_accel,
      y_accel: msg.y_accel,
      z_accel: msg.z_accel,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      bitfield0: msg.bitfield0,
      reserved1: msg.reserved1,
      i_tow: msg.i_tow,
      x_ang_rate: msg.x_ang_rate,
      y_ang_rate: msg.y_ang_rate,
      z_ang_rate: msg.z_ang_rate,
      x_accel: msg.x_accel,
      y_accel: msg.y_accel,
      z_accel: msg.z_accel,
    }
  }
}


// Corresponds to ublox_msgs__msg__EsfMEAS
/// ESF-MEAS (0x10 0x02)
/// External Sensor Fusion Measurements
///
/// Possible data types for the data field are described in the ESF Measurement
/// Data section

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub data: Vec<u32>,

    /// End of repeated block
    /// Start of optional block (size is either 0 or 1)
    /// Receiver local time calibrated.
    /// This field must not be supplied when
    /// calibTtagValid is set to 0
    pub calib_t_tag: Vec<u32>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::EsfMEAS::default())
  }
}

impl rosidl_runtime_rs::Message for EsfMEAS {
  type RmwMsg = super::msg::rmw::EsfMEAS;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        time_tag: msg.time_tag,
        flags: msg.flags,
        id: msg.id,
        data: msg.data.into(),
        calib_t_tag: msg.calib_t_tag.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      time_tag: msg.time_tag,
      flags: msg.flags,
      id: msg.id,
        data: msg.data.as_slice().into(),
        calib_t_tag: msg.calib_t_tag.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      time_tag: msg.time_tag,
      flags: msg.flags,
      id: msg.id,
      data: msg.data
          .into_iter()
          .collect(),
      calib_t_tag: msg.calib_t_tag
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__EsfRAWBlock
/// See ESF-RAW

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::EsfRAWBlock::default())
  }
}

impl rosidl_runtime_rs::Message for EsfRAWBlock {
  type RmwMsg = super::msg::rmw::EsfRAWBlock;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        data: msg.data,
        s_t_tag: msg.s_t_tag,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      data: msg.data,
      s_t_tag: msg.s_t_tag,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      data: msg.data,
      s_t_tag: msg.s_t_tag,
    }
  }
}


// Corresponds to ublox_msgs__msg__EsfRAW
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct EsfRAW {
    /// Reserved
    pub reserved0: [u8; 4],


    // This member is not documented.
    #[allow(missing_docs)]
    pub blocks: Vec<super::msg::EsfRAWBlock>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::EsfRAW::default())
  }
}

impl rosidl_runtime_rs::Message for EsfRAW {
  type RmwMsg = super::msg::rmw::EsfRAW;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        reserved0: msg.reserved0,
        blocks: msg.blocks
          .into_iter()
          .map(|elem| super::msg::EsfRAWBlock::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        reserved0: msg.reserved0,
        blocks: msg.blocks
          .iter()
          .map(|elem| super::msg::EsfRAWBlock::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      reserved0: msg.reserved0,
      blocks: msg.blocks
          .into_iter()
          .map(super::msg::EsfRAWBlock::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__EsfSTATUS
/// ESF-STATUS (0x10 0x10)
/// External Sensor Fusion (ESF) status information
///
/// Supported on UDR/ADR products

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub sens: Vec<super::msg::EsfSTATUSSens>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::EsfSTATUS::default())
  }
}

impl rosidl_runtime_rs::Message for EsfSTATUS {
  type RmwMsg = super::msg::rmw::EsfSTATUS;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        version: msg.version,
        reserved1: msg.reserved1,
        fusion_mode: msg.fusion_mode,
        reserved2: msg.reserved2,
        num_sens: msg.num_sens,
        sens: msg.sens
          .into_iter()
          .map(|elem| super::msg::EsfSTATUSSens::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      version: msg.version,
        reserved1: msg.reserved1,
      fusion_mode: msg.fusion_mode,
        reserved2: msg.reserved2,
      num_sens: msg.num_sens,
        sens: msg.sens
          .iter()
          .map(|elem| super::msg::EsfSTATUSSens::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      version: msg.version,
      reserved1: msg.reserved1,
      fusion_mode: msg.fusion_mode,
      reserved2: msg.reserved2,
      num_sens: msg.num_sens,
      sens: msg.sens
          .into_iter()
          .map(super::msg::EsfSTATUSSens::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__EsfSTATUSSens
/// See Esf-STATUS

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::EsfSTATUSSens::default())
  }
}

impl rosidl_runtime_rs::Message for EsfSTATUSSens {
  type RmwMsg = super::msg::rmw::EsfSTATUSSens;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        sens_status1: msg.sens_status1,
        sens_status2: msg.sens_status2,
        freq: msg.freq,
        faults: msg.faults,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      sens_status1: msg.sens_status1,
      sens_status2: msg.sens_status2,
      freq: msg.freq,
      faults: msg.faults,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      sens_status1: msg.sens_status1,
      sens_status2: msg.sens_status2,
      freq: msg.freq,
      faults: msg.faults,
    }
  }
}


// Corresponds to ublox_msgs__msg__HnrPVT
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::HnrPVT::default())
  }
}

impl rosidl_runtime_rs::Message for HnrPVT {
  type RmwMsg = super::msg::rmw::HnrPVT;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        year: msg.year,
        month: msg.month,
        day: msg.day,
        hour: msg.hour,
        min: msg.min,
        sec: msg.sec,
        valid: msg.valid,
        nano: msg.nano,
        gps_fix: msg.gps_fix,
        flags: msg.flags,
        reserved0: msg.reserved0,
        lon: msg.lon,
        lat: msg.lat,
        height: msg.height,
        h_msl: msg.h_msl,
        g_speed: msg.g_speed,
        speed: msg.speed,
        head_mot: msg.head_mot,
        head_veh: msg.head_veh,
        h_acc: msg.h_acc,
        v_acc: msg.v_acc,
        s_acc: msg.s_acc,
        head_acc: msg.head_acc,
        reserved1: msg.reserved1,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      year: msg.year,
      month: msg.month,
      day: msg.day,
      hour: msg.hour,
      min: msg.min,
      sec: msg.sec,
      valid: msg.valid,
      nano: msg.nano,
      gps_fix: msg.gps_fix,
      flags: msg.flags,
        reserved0: msg.reserved0,
      lon: msg.lon,
      lat: msg.lat,
      height: msg.height,
      h_msl: msg.h_msl,
      g_speed: msg.g_speed,
      speed: msg.speed,
      head_mot: msg.head_mot,
      head_veh: msg.head_veh,
      h_acc: msg.h_acc,
      v_acc: msg.v_acc,
      s_acc: msg.s_acc,
      head_acc: msg.head_acc,
        reserved1: msg.reserved1,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      year: msg.year,
      month: msg.month,
      day: msg.day,
      hour: msg.hour,
      min: msg.min,
      sec: msg.sec,
      valid: msg.valid,
      nano: msg.nano,
      gps_fix: msg.gps_fix,
      flags: msg.flags,
      reserved0: msg.reserved0,
      lon: msg.lon,
      lat: msg.lat,
      height: msg.height,
      h_msl: msg.h_msl,
      g_speed: msg.g_speed,
      speed: msg.speed,
      head_mot: msg.head_mot,
      head_veh: msg.head_veh,
      h_acc: msg.h_acc,
      v_acc: msg.v_acc,
      s_acc: msg.s_acc,
      head_acc: msg.head_acc,
      reserved1: msg.reserved1,
    }
  }
}


// Corresponds to ublox_msgs__msg__Inf
/// UBX-INF (0x04, 0x00...0x04)
/// ASCII output
///
/// This message has a variable length payload, representing an ASCII string.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Inf {

    // This member is not documented.
    #[allow(missing_docs)]
    pub str: Vec<u8>,

}

impl Inf {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLASS_ID: u8 = 4;

}


impl Default for Inf {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::Inf::default())
  }
}

impl rosidl_runtime_rs::Message for Inf {
  type RmwMsg = super::msg::rmw::Inf;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        str: msg.str.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        str: msg.str.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      str: msg.str
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__MgaGAL
/// MGA-GAL (0x13 0x02)
/// Galileo Ephemeris Assistance
///
/// This message allows the delivery of Galileo ephemeris assistance to a
/// receiver. See the description of AssistNow Online for details.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MgaGAL::default())
  }
}

impl rosidl_runtime_rs::Message for MgaGAL {
  type RmwMsg = super::msg::rmw::MgaGAL;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        type_: msg.type_,
        version: msg.version,
        svid: msg.svid,
        reserved0: msg.reserved0,
        iod_nav: msg.iod_nav,
        delta_n: msg.delta_n,
        m0: msg.m0,
        e: msg.e,
        sqrt_a: msg.sqrt_a,
        omega0: msg.omega0,
        i0: msg.i0,
        omega: msg.omega,
        omega_dot: msg.omega_dot,
        i_dot: msg.i_dot,
        cuc: msg.cuc,
        cus: msg.cus,
        crc: msg.crc,
        crs: msg.crs,
        cic: msg.cic,
        cis: msg.cis,
        toe: msg.toe,
        af0: msg.af0,
        af1: msg.af1,
        af2: msg.af2,
        sisaindex_e1_e5b: msg.sisaindex_e1_e5b,
        toc: msg.toc,
        bgd_e1_e5b: msg.bgd_e1_e5b,
        reserved1: msg.reserved1,
        health_e1b: msg.health_e1b,
        data_validity_e1b: msg.data_validity_e1b,
        health_e5b: msg.health_e5b,
        data_validity_e5b: msg.data_validity_e5b,
        reserved2: msg.reserved2,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      type_: msg.type_,
      version: msg.version,
      svid: msg.svid,
      reserved0: msg.reserved0,
      iod_nav: msg.iod_nav,
      delta_n: msg.delta_n,
      m0: msg.m0,
      e: msg.e,
      sqrt_a: msg.sqrt_a,
      omega0: msg.omega0,
      i0: msg.i0,
      omega: msg.omega,
      omega_dot: msg.omega_dot,
      i_dot: msg.i_dot,
      cuc: msg.cuc,
      cus: msg.cus,
      crc: msg.crc,
      crs: msg.crs,
      cic: msg.cic,
      cis: msg.cis,
      toe: msg.toe,
      af0: msg.af0,
      af1: msg.af1,
      af2: msg.af2,
      sisaindex_e1_e5b: msg.sisaindex_e1_e5b,
      toc: msg.toc,
      bgd_e1_e5b: msg.bgd_e1_e5b,
        reserved1: msg.reserved1,
      health_e1b: msg.health_e1b,
      data_validity_e1b: msg.data_validity_e1b,
      health_e5b: msg.health_e5b,
      data_validity_e5b: msg.data_validity_e5b,
        reserved2: msg.reserved2,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      type_: msg.type_,
      version: msg.version,
      svid: msg.svid,
      reserved0: msg.reserved0,
      iod_nav: msg.iod_nav,
      delta_n: msg.delta_n,
      m0: msg.m0,
      e: msg.e,
      sqrt_a: msg.sqrt_a,
      omega0: msg.omega0,
      i0: msg.i0,
      omega: msg.omega,
      omega_dot: msg.omega_dot,
      i_dot: msg.i_dot,
      cuc: msg.cuc,
      cus: msg.cus,
      crc: msg.crc,
      crs: msg.crs,
      cic: msg.cic,
      cis: msg.cis,
      toe: msg.toe,
      af0: msg.af0,
      af1: msg.af1,
      af2: msg.af2,
      sisaindex_e1_e5b: msg.sisaindex_e1_e5b,
      toc: msg.toc,
      bgd_e1_e5b: msg.bgd_e1_e5b,
      reserved1: msg.reserved1,
      health_e1b: msg.health_e1b,
      data_validity_e1b: msg.data_validity_e1b,
      health_e5b: msg.health_e5b,
      data_validity_e5b: msg.data_validity_e5b,
      reserved2: msg.reserved2,
    }
  }
}


// Corresponds to ublox_msgs__msg__MonGNSS
/// MON-GNSS (0x0A 0x28)
/// Information message major GNSS selection
///
/// This message reports major GNSS selection. Augmentation systems are not
/// reported.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MonGNSS::default())
  }
}

impl rosidl_runtime_rs::Message for MonGNSS {
  type RmwMsg = super::msg::rmw::MonGNSS;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        version: msg.version,
        supported: msg.supported,
        default_gnss: msg.default_gnss,
        enabled: msg.enabled,
        simultaneous: msg.simultaneous,
        reserved1: msg.reserved1,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      version: msg.version,
      supported: msg.supported,
      default_gnss: msg.default_gnss,
      enabled: msg.enabled,
      simultaneous: msg.simultaneous,
        reserved1: msg.reserved1,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      version: msg.version,
      supported: msg.supported,
      default_gnss: msg.default_gnss,
      enabled: msg.enabled,
      simultaneous: msg.simultaneous,
      reserved1: msg.reserved1,
    }
  }
}


// Corresponds to ublox_msgs__msg__MonHW6
/// MON-HW (0x0A 0x09)
/// Hardware Status
/// Firmware 6
///
/// Status of different aspect of the hardware, such as Antenna, PIO/Peripheral
/// Pins, Noise Level, Automatic Gain Control (AGC)
///
/// WARNING: this message is a different length than the MonHW message for
/// firmware version 7 & 8

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MonHW6::default())
  }
}

impl rosidl_runtime_rs::Message for MonHW6 {
  type RmwMsg = super::msg::rmw::MonHW6;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pin_sel: msg.pin_sel,
        pin_bank: msg.pin_bank,
        pin_dir: msg.pin_dir,
        pin_val: msg.pin_val,
        noise_per_ms: msg.noise_per_ms,
        agc_cnt: msg.agc_cnt,
        a_status: msg.a_status,
        a_power: msg.a_power,
        flags: msg.flags,
        reserved0: msg.reserved0,
        used_mask: msg.used_mask,
        vp: msg.vp,
        jam_ind: msg.jam_ind,
        reserved1: msg.reserved1,
        pin_irq: msg.pin_irq,
        pull_h: msg.pull_h,
        pull_l: msg.pull_l,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      pin_sel: msg.pin_sel,
      pin_bank: msg.pin_bank,
      pin_dir: msg.pin_dir,
      pin_val: msg.pin_val,
      noise_per_ms: msg.noise_per_ms,
      agc_cnt: msg.agc_cnt,
      a_status: msg.a_status,
      a_power: msg.a_power,
      flags: msg.flags,
      reserved0: msg.reserved0,
      used_mask: msg.used_mask,
        vp: msg.vp,
      jam_ind: msg.jam_ind,
        reserved1: msg.reserved1,
      pin_irq: msg.pin_irq,
      pull_h: msg.pull_h,
      pull_l: msg.pull_l,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pin_sel: msg.pin_sel,
      pin_bank: msg.pin_bank,
      pin_dir: msg.pin_dir,
      pin_val: msg.pin_val,
      noise_per_ms: msg.noise_per_ms,
      agc_cnt: msg.agc_cnt,
      a_status: msg.a_status,
      a_power: msg.a_power,
      flags: msg.flags,
      reserved0: msg.reserved0,
      used_mask: msg.used_mask,
      vp: msg.vp,
      jam_ind: msg.jam_ind,
      reserved1: msg.reserved1,
      pin_irq: msg.pin_irq,
      pull_h: msg.pull_h,
      pull_l: msg.pull_l,
    }
  }
}


// Corresponds to ublox_msgs__msg__MonHW
/// MON-HW (0x0A 0x09)
/// Hardware Status
///
/// Status of different aspect of the hardware, such as Antenna, PIO/Peripheral
/// Pins, Noise Level, Automatic Gain Control (AGC)
///
/// WARNING: this message is a different length than the MonHW message for
/// firmware version 6

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MonHW::default())
  }
}

impl rosidl_runtime_rs::Message for MonHW {
  type RmwMsg = super::msg::rmw::MonHW;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pin_sel: msg.pin_sel,
        pin_bank: msg.pin_bank,
        pin_dir: msg.pin_dir,
        pin_val: msg.pin_val,
        noise_per_ms: msg.noise_per_ms,
        agc_cnt: msg.agc_cnt,
        a_status: msg.a_status,
        a_power: msg.a_power,
        flags: msg.flags,
        reserved0: msg.reserved0,
        used_mask: msg.used_mask,
        vp: msg.vp,
        jam_ind: msg.jam_ind,
        reserved1: msg.reserved1,
        pin_irq: msg.pin_irq,
        pull_h: msg.pull_h,
        pull_l: msg.pull_l,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      pin_sel: msg.pin_sel,
      pin_bank: msg.pin_bank,
      pin_dir: msg.pin_dir,
      pin_val: msg.pin_val,
      noise_per_ms: msg.noise_per_ms,
      agc_cnt: msg.agc_cnt,
      a_status: msg.a_status,
      a_power: msg.a_power,
      flags: msg.flags,
      reserved0: msg.reserved0,
      used_mask: msg.used_mask,
        vp: msg.vp,
      jam_ind: msg.jam_ind,
        reserved1: msg.reserved1,
      pin_irq: msg.pin_irq,
      pull_h: msg.pull_h,
      pull_l: msg.pull_l,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pin_sel: msg.pin_sel,
      pin_bank: msg.pin_bank,
      pin_dir: msg.pin_dir,
      pin_val: msg.pin_val,
      noise_per_ms: msg.noise_per_ms,
      agc_cnt: msg.agc_cnt,
      a_status: msg.a_status,
      a_power: msg.a_power,
      flags: msg.flags,
      reserved0: msg.reserved0,
      used_mask: msg.used_mask,
      vp: msg.vp,
      jam_ind: msg.jam_ind,
      reserved1: msg.reserved1,
      pin_irq: msg.pin_irq,
      pull_h: msg.pull_h,
      pull_l: msg.pull_l,
    }
  }
}


// Corresponds to ublox_msgs__msg__MonVERExtension
/// see MonVER message

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MonVERExtension {

    // This member is not documented.
    #[allow(missing_docs)]
    pub field: [u8; 30],

}



impl Default for MonVERExtension {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MonVERExtension::default())
  }
}

impl rosidl_runtime_rs::Message for MonVERExtension {
  type RmwMsg = super::msg::rmw::MonVERExtension;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        field: msg.field,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        field: msg.field,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      field: msg.field,
    }
  }
}


// Corresponds to ublox_msgs__msg__MonVER
/// MON-VER (0x0A 0x04)
///
/// Receiver/Software Version
/// Returned when the version is polled.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MonVER {
    /// Zero-terminated software version string.
    pub sw_version: [u8; 30],

    /// Zero-terminated hardware version string.
    pub hw_version: [u8; 10],

    /// Start of repeated block (N times)
    pub extension: Vec<super::msg::MonVERExtension>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MonVER::default())
  }
}

impl rosidl_runtime_rs::Message for MonVER {
  type RmwMsg = super::msg::rmw::MonVER;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        sw_version: msg.sw_version,
        hw_version: msg.hw_version,
        extension: msg.extension
          .into_iter()
          .map(|elem| super::msg::MonVERExtension::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        sw_version: msg.sw_version,
        hw_version: msg.hw_version,
        extension: msg.extension
          .iter()
          .map(|elem| super::msg::MonVERExtension::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      sw_version: msg.sw_version,
      hw_version: msg.hw_version,
      extension: msg.extension
          .into_iter()
          .map(super::msg::MonVERExtension::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__NavATT
/// NAV-ATT (0x01 0x05)
/// Attitude Solution
///
/// This message outputs the attitude solution as roll, pitch and heading angles.
/// Supported on ADR and UDR products.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavATT::default())
  }
}

impl rosidl_runtime_rs::Message for NavATT {
  type RmwMsg = super::msg::rmw::NavATT;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        version: msg.version,
        reserved0: msg.reserved0,
        roll: msg.roll,
        pitch: msg.pitch,
        heading: msg.heading,
        acc_roll: msg.acc_roll,
        acc_pitch: msg.acc_pitch,
        acc_heading: msg.acc_heading,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      version: msg.version,
        reserved0: msg.reserved0,
      roll: msg.roll,
      pitch: msg.pitch,
      heading: msg.heading,
      acc_roll: msg.acc_roll,
      acc_pitch: msg.acc_pitch,
      acc_heading: msg.acc_heading,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      version: msg.version,
      reserved0: msg.reserved0,
      roll: msg.roll,
      pitch: msg.pitch,
      heading: msg.heading,
      acc_roll: msg.acc_roll,
      acc_pitch: msg.acc_pitch,
      acc_heading: msg.acc_heading,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavCLOCK
/// NAV-CLOCK (0x01 0x22)
/// Clock Solution

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavCLOCK::default())
  }
}

impl rosidl_runtime_rs::Message for NavCLOCK {
  type RmwMsg = super::msg::rmw::NavCLOCK;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        clk_b: msg.clk_b,
        clk_d: msg.clk_d,
        t_acc: msg.t_acc,
        f_acc: msg.f_acc,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      clk_b: msg.clk_b,
      clk_d: msg.clk_d,
      t_acc: msg.t_acc,
      f_acc: msg.f_acc,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      clk_b: msg.clk_b,
      clk_d: msg.clk_d,
      t_acc: msg.t_acc,
      f_acc: msg.f_acc,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavCOV
/// NAV-COV (0x01 0x36)
/// Covariance matrices for position and velocity solutions

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavCOV::default())
  }
}

impl rosidl_runtime_rs::Message for NavCOV {
  type RmwMsg = super::msg::rmw::NavCOV;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        version: msg.version,
        pos_cov_valid: msg.pos_cov_valid,
        vel_cov_valid: msg.vel_cov_valid,
        reserved_0: msg.reserved_0,
        pos_cov_nn: msg.pos_cov_nn,
        pos_cov_ne: msg.pos_cov_ne,
        pos_cov_nd: msg.pos_cov_nd,
        pos_cov_ee: msg.pos_cov_ee,
        pos_cov_ed: msg.pos_cov_ed,
        pos_cov_dd: msg.pos_cov_dd,
        vel_cov_nn: msg.vel_cov_nn,
        vel_cov_ne: msg.vel_cov_ne,
        vel_cov_nd: msg.vel_cov_nd,
        vel_cov_ee: msg.vel_cov_ee,
        vel_cov_ed: msg.vel_cov_ed,
        vel_cov_dd: msg.vel_cov_dd,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      version: msg.version,
      pos_cov_valid: msg.pos_cov_valid,
      vel_cov_valid: msg.vel_cov_valid,
        reserved_0: msg.reserved_0,
      pos_cov_nn: msg.pos_cov_nn,
      pos_cov_ne: msg.pos_cov_ne,
      pos_cov_nd: msg.pos_cov_nd,
      pos_cov_ee: msg.pos_cov_ee,
      pos_cov_ed: msg.pos_cov_ed,
      pos_cov_dd: msg.pos_cov_dd,
      vel_cov_nn: msg.vel_cov_nn,
      vel_cov_ne: msg.vel_cov_ne,
      vel_cov_nd: msg.vel_cov_nd,
      vel_cov_ee: msg.vel_cov_ee,
      vel_cov_ed: msg.vel_cov_ed,
      vel_cov_dd: msg.vel_cov_dd,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      version: msg.version,
      pos_cov_valid: msg.pos_cov_valid,
      vel_cov_valid: msg.vel_cov_valid,
      reserved_0: msg.reserved_0,
      pos_cov_nn: msg.pos_cov_nn,
      pos_cov_ne: msg.pos_cov_ne,
      pos_cov_nd: msg.pos_cov_nd,
      pos_cov_ee: msg.pos_cov_ee,
      pos_cov_ed: msg.pos_cov_ed,
      pos_cov_dd: msg.pos_cov_dd,
      vel_cov_nn: msg.vel_cov_nn,
      vel_cov_ne: msg.vel_cov_ne,
      vel_cov_nd: msg.vel_cov_nd,
      vel_cov_ee: msg.vel_cov_ee,
      vel_cov_ed: msg.vel_cov_ed,
      vel_cov_dd: msg.vel_cov_dd,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavDGPS
/// NAV-DGPS (0x01 0x31)
/// DGPS Data Used for NAV
///
/// This message outputs the Correction data as it has been applied to the current
/// NAV Solution. See also the notes on the RTCM protocol.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub sv: Vec<super::msg::NavDGPSSV>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavDGPS::default())
  }
}

impl rosidl_runtime_rs::Message for NavDGPS {
  type RmwMsg = super::msg::rmw::NavDGPS;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        age: msg.age,
        base_id: msg.base_id,
        base_health: msg.base_health,
        num_ch: msg.num_ch,
        status: msg.status,
        reserved1: msg.reserved1,
        sv: msg.sv
          .into_iter()
          .map(|elem| super::msg::NavDGPSSV::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      age: msg.age,
      base_id: msg.base_id,
      base_health: msg.base_health,
      num_ch: msg.num_ch,
      status: msg.status,
      reserved1: msg.reserved1,
        sv: msg.sv
          .iter()
          .map(|elem| super::msg::NavDGPSSV::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      age: msg.age,
      base_id: msg.base_id,
      base_health: msg.base_health,
      num_ch: msg.num_ch,
      status: msg.status,
      reserved1: msg.reserved1,
      sv: msg.sv
          .into_iter()
          .map(super::msg::NavDGPSSV::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__NavDGPSSV
/// see message NavDGPS

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavDGPSSV::default())
  }
}

impl rosidl_runtime_rs::Message for NavDGPSSV {
  type RmwMsg = super::msg::rmw::NavDGPSSV;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        svid: msg.svid,
        flags: msg.flags,
        age_c: msg.age_c,
        prc: msg.prc,
        prrc: msg.prrc,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      svid: msg.svid,
      flags: msg.flags,
      age_c: msg.age_c,
      prc: msg.prc,
      prrc: msg.prrc,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      svid: msg.svid,
      flags: msg.flags,
      age_c: msg.age_c,
      prc: msg.prc,
      prrc: msg.prrc,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavDOP
/// NAV-DOP (0x01 0x04)
/// Dilution of precision
///
/// - DOP values are dimensionless.
/// - All DOP values are scaled by a factor of 100. If the unit transmits a value
///   of e.g. 156, the DOP value is 1.56.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavDOP::default())
  }
}

impl rosidl_runtime_rs::Message for NavDOP {
  type RmwMsg = super::msg::rmw::NavDOP;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        g_dop: msg.g_dop,
        p_dop: msg.p_dop,
        t_dop: msg.t_dop,
        v_dop: msg.v_dop,
        h_dop: msg.h_dop,
        n_dop: msg.n_dop,
        e_dop: msg.e_dop,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      g_dop: msg.g_dop,
      p_dop: msg.p_dop,
      t_dop: msg.t_dop,
      v_dop: msg.v_dop,
      h_dop: msg.h_dop,
      n_dop: msg.n_dop,
      e_dop: msg.e_dop,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      g_dop: msg.g_dop,
      p_dop: msg.p_dop,
      t_dop: msg.t_dop,
      v_dop: msg.v_dop,
      h_dop: msg.h_dop,
      n_dop: msg.n_dop,
      e_dop: msg.e_dop,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavPOSECEF
/// NAV-POSECEF (0x01 0x01)
/// Position Solution in ECEF
///
/// See important comments concerning validity of position given in section
/// Navigation Output Filters.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavPOSECEF::default())
  }
}

impl rosidl_runtime_rs::Message for NavPOSECEF {
  type RmwMsg = super::msg::rmw::NavPOSECEF;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        ecef_x: msg.ecef_x,
        ecef_y: msg.ecef_y,
        ecef_z: msg.ecef_z,
        p_acc: msg.p_acc,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      ecef_x: msg.ecef_x,
      ecef_y: msg.ecef_y,
      ecef_z: msg.ecef_z,
      p_acc: msg.p_acc,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      ecef_x: msg.ecef_x,
      ecef_y: msg.ecef_y,
      ecef_z: msg.ecef_z,
      p_acc: msg.p_acc,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavPOSLLH
/// NAV-POSLLH (0x01 0x02)
/// Geodetic Position Solution
///
/// See important comments concerning validity of position given in section
/// Navigation Output Filters.
/// This message outputs the Geodetic position in the currently selected
/// Ellipsoid. The default is the WGS84 Ellipsoid, but can be changed with the
/// message CFG-DAT.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavPOSLLH::default())
  }
}

impl rosidl_runtime_rs::Message for NavPOSLLH {
  type RmwMsg = super::msg::rmw::NavPOSLLH;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        lon: msg.lon,
        lat: msg.lat,
        height: msg.height,
        h_msl: msg.h_msl,
        h_acc: msg.h_acc,
        v_acc: msg.v_acc,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      lon: msg.lon,
      lat: msg.lat,
      height: msg.height,
      h_msl: msg.h_msl,
      h_acc: msg.h_acc,
      v_acc: msg.v_acc,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      lon: msg.lon,
      lat: msg.lat,
      height: msg.height,
      h_msl: msg.h_msl,
      h_acc: msg.h_acc,
      v_acc: msg.v_acc,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavPVT7
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavPVT7::default())
  }
}

impl rosidl_runtime_rs::Message for NavPVT7 {
  type RmwMsg = super::msg::rmw::NavPVT7;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        year: msg.year,
        month: msg.month,
        day: msg.day,
        hour: msg.hour,
        min: msg.min,
        sec: msg.sec,
        valid: msg.valid,
        t_acc: msg.t_acc,
        nano: msg.nano,
        fix_type: msg.fix_type,
        flags: msg.flags,
        flags2: msg.flags2,
        num_sv: msg.num_sv,
        lon: msg.lon,
        lat: msg.lat,
        height: msg.height,
        h_msl: msg.h_msl,
        h_acc: msg.h_acc,
        v_acc: msg.v_acc,
        vel_n: msg.vel_n,
        vel_e: msg.vel_e,
        vel_d: msg.vel_d,
        g_speed: msg.g_speed,
        heading: msg.heading,
        s_acc: msg.s_acc,
        head_acc: msg.head_acc,
        p_dop: msg.p_dop,
        reserved1: msg.reserved1,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      year: msg.year,
      month: msg.month,
      day: msg.day,
      hour: msg.hour,
      min: msg.min,
      sec: msg.sec,
      valid: msg.valid,
      t_acc: msg.t_acc,
      nano: msg.nano,
      fix_type: msg.fix_type,
      flags: msg.flags,
      flags2: msg.flags2,
      num_sv: msg.num_sv,
      lon: msg.lon,
      lat: msg.lat,
      height: msg.height,
      h_msl: msg.h_msl,
      h_acc: msg.h_acc,
      v_acc: msg.v_acc,
      vel_n: msg.vel_n,
      vel_e: msg.vel_e,
      vel_d: msg.vel_d,
      g_speed: msg.g_speed,
      heading: msg.heading,
      s_acc: msg.s_acc,
      head_acc: msg.head_acc,
      p_dop: msg.p_dop,
        reserved1: msg.reserved1,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      year: msg.year,
      month: msg.month,
      day: msg.day,
      hour: msg.hour,
      min: msg.min,
      sec: msg.sec,
      valid: msg.valid,
      t_acc: msg.t_acc,
      nano: msg.nano,
      fix_type: msg.fix_type,
      flags: msg.flags,
      flags2: msg.flags2,
      num_sv: msg.num_sv,
      lon: msg.lon,
      lat: msg.lat,
      height: msg.height,
      h_msl: msg.h_msl,
      h_acc: msg.h_acc,
      v_acc: msg.v_acc,
      vel_n: msg.vel_n,
      vel_e: msg.vel_e,
      vel_d: msg.vel_d,
      g_speed: msg.g_speed,
      heading: msg.heading,
      s_acc: msg.s_acc,
      head_acc: msg.head_acc,
      p_dop: msg.p_dop,
      reserved1: msg.reserved1,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavPVT
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavPVT::default())
  }
}

impl rosidl_runtime_rs::Message for NavPVT {
  type RmwMsg = super::msg::rmw::NavPVT;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        year: msg.year,
        month: msg.month,
        day: msg.day,
        hour: msg.hour,
        min: msg.min,
        sec: msg.sec,
        valid: msg.valid,
        t_acc: msg.t_acc,
        nano: msg.nano,
        fix_type: msg.fix_type,
        flags: msg.flags,
        flags2: msg.flags2,
        num_sv: msg.num_sv,
        lon: msg.lon,
        lat: msg.lat,
        height: msg.height,
        h_msl: msg.h_msl,
        h_acc: msg.h_acc,
        v_acc: msg.v_acc,
        vel_n: msg.vel_n,
        vel_e: msg.vel_e,
        vel_d: msg.vel_d,
        g_speed: msg.g_speed,
        heading: msg.heading,
        s_acc: msg.s_acc,
        head_acc: msg.head_acc,
        p_dop: msg.p_dop,
        reserved1: msg.reserved1,
        head_veh: msg.head_veh,
        mag_dec: msg.mag_dec,
        mag_acc: msg.mag_acc,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      year: msg.year,
      month: msg.month,
      day: msg.day,
      hour: msg.hour,
      min: msg.min,
      sec: msg.sec,
      valid: msg.valid,
      t_acc: msg.t_acc,
      nano: msg.nano,
      fix_type: msg.fix_type,
      flags: msg.flags,
      flags2: msg.flags2,
      num_sv: msg.num_sv,
      lon: msg.lon,
      lat: msg.lat,
      height: msg.height,
      h_msl: msg.h_msl,
      h_acc: msg.h_acc,
      v_acc: msg.v_acc,
      vel_n: msg.vel_n,
      vel_e: msg.vel_e,
      vel_d: msg.vel_d,
      g_speed: msg.g_speed,
      heading: msg.heading,
      s_acc: msg.s_acc,
      head_acc: msg.head_acc,
      p_dop: msg.p_dop,
        reserved1: msg.reserved1,
      head_veh: msg.head_veh,
      mag_dec: msg.mag_dec,
      mag_acc: msg.mag_acc,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      year: msg.year,
      month: msg.month,
      day: msg.day,
      hour: msg.hour,
      min: msg.min,
      sec: msg.sec,
      valid: msg.valid,
      t_acc: msg.t_acc,
      nano: msg.nano,
      fix_type: msg.fix_type,
      flags: msg.flags,
      flags2: msg.flags2,
      num_sv: msg.num_sv,
      lon: msg.lon,
      lat: msg.lat,
      height: msg.height,
      h_msl: msg.h_msl,
      h_acc: msg.h_acc,
      v_acc: msg.v_acc,
      vel_n: msg.vel_n,
      vel_e: msg.vel_e,
      vel_d: msg.vel_d,
      g_speed: msg.g_speed,
      heading: msg.heading,
      s_acc: msg.s_acc,
      head_acc: msg.head_acc,
      p_dop: msg.p_dop,
      reserved1: msg.reserved1,
      head_veh: msg.head_veh,
      mag_dec: msg.mag_dec,
      mag_acc: msg.mag_acc,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavRELPOSNED9
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavRELPOSNED9::default())
  }
}

impl rosidl_runtime_rs::Message for NavRELPOSNED9 {
  type RmwMsg = super::msg::rmw::NavRELPOSNED9;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        version: msg.version,
        reserved1: msg.reserved1,
        ref_station_id: msg.ref_station_id,
        i_tow: msg.i_tow,
        rel_pos_n: msg.rel_pos_n,
        rel_pos_e: msg.rel_pos_e,
        rel_pos_d: msg.rel_pos_d,
        rel_pos_length: msg.rel_pos_length,
        rel_pos_heading: msg.rel_pos_heading,
        reserved2: msg.reserved2,
        rel_pos_hpn: msg.rel_pos_hpn,
        rel_pos_hpe: msg.rel_pos_hpe,
        rel_pos_hpd: msg.rel_pos_hpd,
        rel_pos_hp_length: msg.rel_pos_hp_length,
        acc_n: msg.acc_n,
        acc_e: msg.acc_e,
        acc_d: msg.acc_d,
        acc_length: msg.acc_length,
        acc_heading: msg.acc_heading,
        reserved3: msg.reserved3,
        flags: msg.flags,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      version: msg.version,
      reserved1: msg.reserved1,
      ref_station_id: msg.ref_station_id,
      i_tow: msg.i_tow,
      rel_pos_n: msg.rel_pos_n,
      rel_pos_e: msg.rel_pos_e,
      rel_pos_d: msg.rel_pos_d,
      rel_pos_length: msg.rel_pos_length,
      rel_pos_heading: msg.rel_pos_heading,
        reserved2: msg.reserved2,
      rel_pos_hpn: msg.rel_pos_hpn,
      rel_pos_hpe: msg.rel_pos_hpe,
      rel_pos_hpd: msg.rel_pos_hpd,
      rel_pos_hp_length: msg.rel_pos_hp_length,
      acc_n: msg.acc_n,
      acc_e: msg.acc_e,
      acc_d: msg.acc_d,
      acc_length: msg.acc_length,
      acc_heading: msg.acc_heading,
        reserved3: msg.reserved3,
      flags: msg.flags,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      version: msg.version,
      reserved1: msg.reserved1,
      ref_station_id: msg.ref_station_id,
      i_tow: msg.i_tow,
      rel_pos_n: msg.rel_pos_n,
      rel_pos_e: msg.rel_pos_e,
      rel_pos_d: msg.rel_pos_d,
      rel_pos_length: msg.rel_pos_length,
      rel_pos_heading: msg.rel_pos_heading,
      reserved2: msg.reserved2,
      rel_pos_hpn: msg.rel_pos_hpn,
      rel_pos_hpe: msg.rel_pos_hpe,
      rel_pos_hpd: msg.rel_pos_hpd,
      rel_pos_hp_length: msg.rel_pos_hp_length,
      acc_n: msg.acc_n,
      acc_e: msg.acc_e,
      acc_d: msg.acc_d,
      acc_length: msg.acc_length,
      acc_heading: msg.acc_heading,
      reserved3: msg.reserved3,
      flags: msg.flags,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavRELPOSNED
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavRELPOSNED::default())
  }
}

impl rosidl_runtime_rs::Message for NavRELPOSNED {
  type RmwMsg = super::msg::rmw::NavRELPOSNED;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        version: msg.version,
        reserved0: msg.reserved0,
        ref_station_id: msg.ref_station_id,
        i_tow: msg.i_tow,
        rel_pos_n: msg.rel_pos_n,
        rel_pos_e: msg.rel_pos_e,
        rel_pos_d: msg.rel_pos_d,
        rel_pos_hpn: msg.rel_pos_hpn,
        rel_pos_hpe: msg.rel_pos_hpe,
        rel_pos_hpd: msg.rel_pos_hpd,
        reserved1: msg.reserved1,
        acc_n: msg.acc_n,
        acc_e: msg.acc_e,
        acc_d: msg.acc_d,
        flags: msg.flags,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      version: msg.version,
      reserved0: msg.reserved0,
      ref_station_id: msg.ref_station_id,
      i_tow: msg.i_tow,
      rel_pos_n: msg.rel_pos_n,
      rel_pos_e: msg.rel_pos_e,
      rel_pos_d: msg.rel_pos_d,
      rel_pos_hpn: msg.rel_pos_hpn,
      rel_pos_hpe: msg.rel_pos_hpe,
      rel_pos_hpd: msg.rel_pos_hpd,
      reserved1: msg.reserved1,
      acc_n: msg.acc_n,
      acc_e: msg.acc_e,
      acc_d: msg.acc_d,
      flags: msg.flags,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      version: msg.version,
      reserved0: msg.reserved0,
      ref_station_id: msg.ref_station_id,
      i_tow: msg.i_tow,
      rel_pos_n: msg.rel_pos_n,
      rel_pos_e: msg.rel_pos_e,
      rel_pos_d: msg.rel_pos_d,
      rel_pos_hpn: msg.rel_pos_hpn,
      rel_pos_hpe: msg.rel_pos_hpe,
      rel_pos_hpd: msg.rel_pos_hpd,
      reserved1: msg.reserved1,
      acc_n: msg.acc_n,
      acc_e: msg.acc_e,
      acc_d: msg.acc_d,
      flags: msg.flags,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavSAT
/// NAV-SAT (0x01 0x35)
/// Satellite Information
///
/// This message displays information about SVs which are either known to be
/// visible or currently tracked by the receiver.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub sv: Vec<super::msg::NavSATSV>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavSAT::default())
  }
}

impl rosidl_runtime_rs::Message for NavSAT {
  type RmwMsg = super::msg::rmw::NavSAT;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        version: msg.version,
        num_svs: msg.num_svs,
        reserved0: msg.reserved0,
        sv: msg.sv
          .into_iter()
          .map(|elem| super::msg::NavSATSV::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      version: msg.version,
      num_svs: msg.num_svs,
        reserved0: msg.reserved0,
        sv: msg.sv
          .iter()
          .map(|elem| super::msg::NavSATSV::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      version: msg.version,
      num_svs: msg.num_svs,
      reserved0: msg.reserved0,
      sv: msg.sv
          .into_iter()
          .map(super::msg::NavSATSV::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__NavSATSV
/// see NAV-SAT message

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavSATSV::default())
  }
}

impl rosidl_runtime_rs::Message for NavSATSV {
  type RmwMsg = super::msg::rmw::NavSATSV;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        gnss_id: msg.gnss_id,
        sv_id: msg.sv_id,
        cno: msg.cno,
        elev: msg.elev,
        azim: msg.azim,
        pr_res: msg.pr_res,
        flags: msg.flags,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      gnss_id: msg.gnss_id,
      sv_id: msg.sv_id,
      cno: msg.cno,
      elev: msg.elev,
      azim: msg.azim,
      pr_res: msg.pr_res,
      flags: msg.flags,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      gnss_id: msg.gnss_id,
      sv_id: msg.sv_id,
      cno: msg.cno,
      elev: msg.elev,
      azim: msg.azim,
      pr_res: msg.pr_res,
      flags: msg.flags,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavSBAS
/// NAV-SBAS (0x01 0x32)
/// SBAS Status Data
///
/// This message outputs the status of the SBAS sub system

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub sv: Vec<super::msg::NavSBASSV>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavSBAS::default())
  }
}

impl rosidl_runtime_rs::Message for NavSBAS {
  type RmwMsg = super::msg::rmw::NavSBAS;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        geo: msg.geo,
        mode: msg.mode,
        sys: msg.sys,
        service: msg.service,
        cnt: msg.cnt,
        reserved0: msg.reserved0,
        sv: msg.sv
          .into_iter()
          .map(|elem| super::msg::NavSBASSV::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      geo: msg.geo,
      mode: msg.mode,
      sys: msg.sys,
      service: msg.service,
      cnt: msg.cnt,
        reserved0: msg.reserved0,
        sv: msg.sv
          .iter()
          .map(|elem| super::msg::NavSBASSV::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      geo: msg.geo,
      mode: msg.mode,
      sys: msg.sys,
      service: msg.service,
      cnt: msg.cnt,
      reserved0: msg.reserved0,
      sv: msg.sv
          .into_iter()
          .map(super::msg::NavSBASSV::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__NavSBASSV
/// see message NavSBAS

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavSBASSV::default())
  }
}

impl rosidl_runtime_rs::Message for NavSBASSV {
  type RmwMsg = super::msg::rmw::NavSBASSV;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        svid: msg.svid,
        flags: msg.flags,
        udre: msg.udre,
        sv_sys: msg.sv_sys,
        sv_service: msg.sv_service,
        reserved1: msg.reserved1,
        prc: msg.prc,
        reserved2: msg.reserved2,
        ic: msg.ic,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      svid: msg.svid,
      flags: msg.flags,
      udre: msg.udre,
      sv_sys: msg.sv_sys,
      sv_service: msg.sv_service,
      reserved1: msg.reserved1,
      prc: msg.prc,
      reserved2: msg.reserved2,
      ic: msg.ic,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      svid: msg.svid,
      flags: msg.flags,
      udre: msg.udre,
      sv_sys: msg.sv_sys,
      sv_service: msg.sv_service,
      reserved1: msg.reserved1,
      prc: msg.prc,
      reserved2: msg.reserved2,
      ic: msg.ic,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavSOL
/// NAV-SOL (0x01 0x06)
/// Navigation Solution Information
///
/// This message combines Position, velocity and time solution in ECEF, including
/// accuracy figures
/// This message has only been retained for backwards compatibility; users are
/// recommended to use the UBX-NAV-PVT message in preference.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavSOL::default())
  }
}

impl rosidl_runtime_rs::Message for NavSOL {
  type RmwMsg = super::msg::rmw::NavSOL;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        f_tow: msg.f_tow,
        week: msg.week,
        gps_fix: msg.gps_fix,
        flags: msg.flags,
        ecef_x: msg.ecef_x,
        ecef_y: msg.ecef_y,
        ecef_z: msg.ecef_z,
        p_acc: msg.p_acc,
        ecef_vx: msg.ecef_vx,
        ecef_vy: msg.ecef_vy,
        ecef_vz: msg.ecef_vz,
        s_acc: msg.s_acc,
        p_dop: msg.p_dop,
        reserved1: msg.reserved1,
        num_sv: msg.num_sv,
        reserved2: msg.reserved2,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      f_tow: msg.f_tow,
      week: msg.week,
      gps_fix: msg.gps_fix,
      flags: msg.flags,
      ecef_x: msg.ecef_x,
      ecef_y: msg.ecef_y,
      ecef_z: msg.ecef_z,
      p_acc: msg.p_acc,
      ecef_vx: msg.ecef_vx,
      ecef_vy: msg.ecef_vy,
      ecef_vz: msg.ecef_vz,
      s_acc: msg.s_acc,
      p_dop: msg.p_dop,
      reserved1: msg.reserved1,
      num_sv: msg.num_sv,
      reserved2: msg.reserved2,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      f_tow: msg.f_tow,
      week: msg.week,
      gps_fix: msg.gps_fix,
      flags: msg.flags,
      ecef_x: msg.ecef_x,
      ecef_y: msg.ecef_y,
      ecef_z: msg.ecef_z,
      p_acc: msg.p_acc,
      ecef_vx: msg.ecef_vx,
      ecef_vy: msg.ecef_vy,
      ecef_vz: msg.ecef_vz,
      s_acc: msg.s_acc,
      p_dop: msg.p_dop,
      reserved1: msg.reserved1,
      num_sv: msg.num_sv,
      reserved2: msg.reserved2,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavSTATUS
/// NAV-STATUS (0x01 0x03)
/// Receiver Navigation Status
///
/// See important comments concerning validity of position and velocity given in
/// section Navigation Output Filters.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavSTATUS::default())
  }
}

impl rosidl_runtime_rs::Message for NavSTATUS {
  type RmwMsg = super::msg::rmw::NavSTATUS;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        gps_fix: msg.gps_fix,
        flags: msg.flags,
        fix_stat: msg.fix_stat,
        flags2: msg.flags2,
        ttff: msg.ttff,
        msss: msg.msss,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      gps_fix: msg.gps_fix,
      flags: msg.flags,
      fix_stat: msg.fix_stat,
      flags2: msg.flags2,
      ttff: msg.ttff,
      msss: msg.msss,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      gps_fix: msg.gps_fix,
      flags: msg.flags,
      fix_stat: msg.fix_stat,
      flags2: msg.flags2,
      ttff: msg.ttff,
      msss: msg.msss,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavSVINFO
/// NAV-SVINFO (0x01 0x30)
/// Space Vehicle Information

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub sv: Vec<super::msg::NavSVINFOSV>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavSVINFO::default())
  }
}

impl rosidl_runtime_rs::Message for NavSVINFO {
  type RmwMsg = super::msg::rmw::NavSVINFO;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        num_ch: msg.num_ch,
        global_flags: msg.global_flags,
        reserved2: msg.reserved2,
        sv: msg.sv
          .into_iter()
          .map(|elem| super::msg::NavSVINFOSV::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      num_ch: msg.num_ch,
      global_flags: msg.global_flags,
      reserved2: msg.reserved2,
        sv: msg.sv
          .iter()
          .map(|elem| super::msg::NavSVINFOSV::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      num_ch: msg.num_ch,
      global_flags: msg.global_flags,
      reserved2: msg.reserved2,
      sv: msg.sv
          .into_iter()
          .map(super::msg::NavSVINFOSV::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__NavSVINFOSV
/// see message NavSVINFO

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavSVINFOSV::default())
  }
}

impl rosidl_runtime_rs::Message for NavSVINFOSV {
  type RmwMsg = super::msg::rmw::NavSVINFOSV;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        chn: msg.chn,
        svid: msg.svid,
        flags: msg.flags,
        quality: msg.quality,
        cno: msg.cno,
        elev: msg.elev,
        azim: msg.azim,
        pr_res: msg.pr_res,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      chn: msg.chn,
      svid: msg.svid,
      flags: msg.flags,
      quality: msg.quality,
      cno: msg.cno,
      elev: msg.elev,
      azim: msg.azim,
      pr_res: msg.pr_res,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      chn: msg.chn,
      svid: msg.svid,
      flags: msg.flags,
      quality: msg.quality,
      cno: msg.cno,
      elev: msg.elev,
      azim: msg.azim,
      pr_res: msg.pr_res,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavSVIN
/// NAV-SVIN (0x01 0x3B)
/// Survey-in data
///
/// This message contains information about survey-in parameters.
/// Supported on:
///  - u-blox 8 / u-blox M8 with protocol version 20 (only with High Precision
///    GNSS products)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavSVIN::default())
  }
}

impl rosidl_runtime_rs::Message for NavSVIN {
  type RmwMsg = super::msg::rmw::NavSVIN;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        version: msg.version,
        reserved0: msg.reserved0,
        i_tow: msg.i_tow,
        dur: msg.dur,
        mean_x: msg.mean_x,
        mean_y: msg.mean_y,
        mean_z: msg.mean_z,
        mean_xhp: msg.mean_xhp,
        mean_yhp: msg.mean_yhp,
        mean_zhp: msg.mean_zhp,
        reserved1: msg.reserved1,
        mean_acc: msg.mean_acc,
        obs: msg.obs,
        valid: msg.valid,
        active: msg.active,
        reserved3: msg.reserved3,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      version: msg.version,
        reserved0: msg.reserved0,
      i_tow: msg.i_tow,
      dur: msg.dur,
      mean_x: msg.mean_x,
      mean_y: msg.mean_y,
      mean_z: msg.mean_z,
      mean_xhp: msg.mean_xhp,
      mean_yhp: msg.mean_yhp,
      mean_zhp: msg.mean_zhp,
      reserved1: msg.reserved1,
      mean_acc: msg.mean_acc,
      obs: msg.obs,
      valid: msg.valid,
      active: msg.active,
        reserved3: msg.reserved3,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      version: msg.version,
      reserved0: msg.reserved0,
      i_tow: msg.i_tow,
      dur: msg.dur,
      mean_x: msg.mean_x,
      mean_y: msg.mean_y,
      mean_z: msg.mean_z,
      mean_xhp: msg.mean_xhp,
      mean_yhp: msg.mean_yhp,
      mean_zhp: msg.mean_zhp,
      reserved1: msg.reserved1,
      mean_acc: msg.mean_acc,
      obs: msg.obs,
      valid: msg.valid,
      active: msg.active,
      reserved3: msg.reserved3,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavTIMEGPS
/// NAV-TIMEGPS (0x01 0x20)
/// GPS Time Solution

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavTIMEGPS::default())
  }
}

impl rosidl_runtime_rs::Message for NavTIMEGPS {
  type RmwMsg = super::msg::rmw::NavTIMEGPS;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        f_tow: msg.f_tow,
        week: msg.week,
        leap_s: msg.leap_s,
        valid: msg.valid,
        t_acc: msg.t_acc,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      f_tow: msg.f_tow,
      week: msg.week,
      leap_s: msg.leap_s,
      valid: msg.valid,
      t_acc: msg.t_acc,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      f_tow: msg.f_tow,
      week: msg.week,
      leap_s: msg.leap_s,
      valid: msg.valid,
      t_acc: msg.t_acc,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavTIMEUTC
/// NAV-TIMEUTC (0x01 0x21)
/// UTC Time Solution

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavTIMEUTC::default())
  }
}

impl rosidl_runtime_rs::Message for NavTIMEUTC {
  type RmwMsg = super::msg::rmw::NavTIMEUTC;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        t_acc: msg.t_acc,
        nano: msg.nano,
        year: msg.year,
        month: msg.month,
        day: msg.day,
        hour: msg.hour,
        min: msg.min,
        sec: msg.sec,
        valid: msg.valid,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      t_acc: msg.t_acc,
      nano: msg.nano,
      year: msg.year,
      month: msg.month,
      day: msg.day,
      hour: msg.hour,
      min: msg.min,
      sec: msg.sec,
      valid: msg.valid,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      t_acc: msg.t_acc,
      nano: msg.nano,
      year: msg.year,
      month: msg.month,
      day: msg.day,
      hour: msg.hour,
      min: msg.min,
      sec: msg.sec,
      valid: msg.valid,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavVELECEF
/// NAV-VELECEF (0x01 0x11)
/// Velocity Solution in ECEF
///
/// See important comments concerning validity of velocity given in section
/// Navigation Output Filters.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavVELECEF::default())
  }
}

impl rosidl_runtime_rs::Message for NavVELECEF {
  type RmwMsg = super::msg::rmw::NavVELECEF;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        ecef_vx: msg.ecef_vx,
        ecef_vy: msg.ecef_vy,
        ecef_vz: msg.ecef_vz,
        s_acc: msg.s_acc,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      ecef_vx: msg.ecef_vx,
      ecef_vy: msg.ecef_vy,
      ecef_vz: msg.ecef_vz,
      s_acc: msg.s_acc,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      ecef_vx: msg.ecef_vx,
      ecef_vy: msg.ecef_vy,
      ecef_vz: msg.ecef_vz,
      s_acc: msg.s_acc,
    }
  }
}


// Corresponds to ublox_msgs__msg__NavVELNED
/// NAV-VELNED (0x01 0x12)
/// Velocity Solution in NED
///
/// See important comments concerning validity of velocity given in section
/// Navigation Output Filters.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::NavVELNED::default())
  }
}

impl rosidl_runtime_rs::Message for NavVELNED {
  type RmwMsg = super::msg::rmw::NavVELNED;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        vel_n: msg.vel_n,
        vel_e: msg.vel_e,
        vel_d: msg.vel_d,
        speed: msg.speed,
        g_speed: msg.g_speed,
        heading: msg.heading,
        s_acc: msg.s_acc,
        c_acc: msg.c_acc,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      vel_n: msg.vel_n,
      vel_e: msg.vel_e,
      vel_d: msg.vel_d,
      speed: msg.speed,
      g_speed: msg.g_speed,
      heading: msg.heading,
      s_acc: msg.s_acc,
      c_acc: msg.c_acc,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      vel_n: msg.vel_n,
      vel_e: msg.vel_e,
      vel_d: msg.vel_d,
      speed: msg.speed,
      g_speed: msg.g_speed,
      heading: msg.heading,
      s_acc: msg.s_acc,
      c_acc: msg.c_acc,
    }
  }
}


// Corresponds to ublox_msgs__msg__RxmALM
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RxmALM {
    /// SV ID for which this Almanach Data is
    /// (Valid Range: 1 .. 32 or 51, 56, 63).
    pub svid: u32,

    /// Issue Date of Almanach (GPS week number)
    pub week: u32,

    /// Start of optional block
    /// Almanach Words
    pub dwrd: Vec<u32>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RxmALM::default())
  }
}

impl rosidl_runtime_rs::Message for RxmALM {
  type RmwMsg = super::msg::rmw::RxmALM;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        svid: msg.svid,
        week: msg.week,
        dwrd: msg.dwrd.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      svid: msg.svid,
      week: msg.week,
        dwrd: msg.dwrd.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      svid: msg.svid,
      week: msg.week,
      dwrd: msg.dwrd
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__RxmEPH
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub sf1d: Vec<u32>,

    /// Subframe 2 Words 3..10 (SF2D0..SF2D7)
    pub sf2d: Vec<u32>,

    /// Subframe 3 Words 3..10 (SF3D0..SF3D7)
    pub sf3d: Vec<u32>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RxmEPH::default())
  }
}

impl rosidl_runtime_rs::Message for RxmEPH {
  type RmwMsg = super::msg::rmw::RxmEPH;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        svid: msg.svid,
        how: msg.how,
        sf1d: msg.sf1d.into(),
        sf2d: msg.sf2d.into(),
        sf3d: msg.sf3d.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      svid: msg.svid,
      how: msg.how,
        sf1d: msg.sf1d.as_slice().into(),
        sf2d: msg.sf2d.as_slice().into(),
        sf3d: msg.sf3d.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      svid: msg.svid,
      how: msg.how,
      sf1d: msg.sf1d
          .into_iter()
          .collect(),
      sf2d: msg.sf2d
          .into_iter()
          .collect(),
      sf3d: msg.sf3d
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__RxmRAW
/// RXM-RAW (0x02 0x10)
/// Raw Measurement Data
///
/// Supported up to ublox 7 firmware. See RxmRAWX for ublox 8
/// This message contains all information needed to be able to generate a RINEX
/// observation file.
/// This message outputs pseudorange, doppler and carrier phase measurements for
/// GPS satellites once signals have been synchronised. No other GNSS types are
/// currently supported.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub sv: Vec<super::msg::RxmRAWSV>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RxmRAW::default())
  }
}

impl rosidl_runtime_rs::Message for RxmRAW {
  type RmwMsg = super::msg::rmw::RxmRAW;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        rcv_tow: msg.rcv_tow,
        week: msg.week,
        num_sv: msg.num_sv,
        reserved1: msg.reserved1,
        sv: msg.sv
          .into_iter()
          .map(|elem| super::msg::RxmRAWSV::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      rcv_tow: msg.rcv_tow,
      week: msg.week,
      num_sv: msg.num_sv,
      reserved1: msg.reserved1,
        sv: msg.sv
          .iter()
          .map(|elem| super::msg::RxmRAWSV::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      rcv_tow: msg.rcv_tow,
      week: msg.week,
      num_sv: msg.num_sv,
      reserved1: msg.reserved1,
      sv: msg.sv
          .into_iter()
          .map(super::msg::RxmRAWSV::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__RxmRAWSV
/// see message RxmRAW

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RxmRAWSV::default())
  }
}

impl rosidl_runtime_rs::Message for RxmRAWSV {
  type RmwMsg = super::msg::rmw::RxmRAWSV;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        cp_mes: msg.cp_mes,
        pr_mes: msg.pr_mes,
        do_mes: msg.do_mes,
        sv: msg.sv,
        mes_qi: msg.mes_qi,
        cno: msg.cno,
        lli: msg.lli,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      cp_mes: msg.cp_mes,
      pr_mes: msg.pr_mes,
      do_mes: msg.do_mes,
      sv: msg.sv,
      mes_qi: msg.mes_qi,
      cno: msg.cno,
      lli: msg.lli,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      cp_mes: msg.cp_mes,
      pr_mes: msg.pr_mes,
      do_mes: msg.do_mes,
      sv: msg.sv,
      mes_qi: msg.mes_qi,
      cno: msg.cno,
      lli: msg.lli,
    }
  }
}


// Corresponds to ublox_msgs__msg__RxmRAWXMeas
/// see message RxmRAWX

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RxmRAWXMeas::default())
  }
}

impl rosidl_runtime_rs::Message for RxmRAWXMeas {
  type RmwMsg = super::msg::rmw::RxmRAWXMeas;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        pr_mes: msg.pr_mes,
        cp_mes: msg.cp_mes,
        do_mes: msg.do_mes,
        gnss_id: msg.gnss_id,
        sv_id: msg.sv_id,
        reserved0: msg.reserved0,
        freq_id: msg.freq_id,
        locktime: msg.locktime,
        cno: msg.cno,
        pr_stdev: msg.pr_stdev,
        cp_stdev: msg.cp_stdev,
        do_stdev: msg.do_stdev,
        trk_stat: msg.trk_stat,
        reserved1: msg.reserved1,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      pr_mes: msg.pr_mes,
      cp_mes: msg.cp_mes,
      do_mes: msg.do_mes,
      gnss_id: msg.gnss_id,
      sv_id: msg.sv_id,
      reserved0: msg.reserved0,
      freq_id: msg.freq_id,
      locktime: msg.locktime,
      cno: msg.cno,
      pr_stdev: msg.pr_stdev,
      cp_stdev: msg.cp_stdev,
      do_stdev: msg.do_stdev,
      trk_stat: msg.trk_stat,
      reserved1: msg.reserved1,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      pr_mes: msg.pr_mes,
      cp_mes: msg.cp_mes,
      do_mes: msg.do_mes,
      gnss_id: msg.gnss_id,
      sv_id: msg.sv_id,
      reserved0: msg.reserved0,
      freq_id: msg.freq_id,
      locktime: msg.locktime,
      cno: msg.cno,
      pr_stdev: msg.pr_stdev,
      cp_stdev: msg.cp_stdev,
      do_stdev: msg.do_stdev,
      trk_stat: msg.trk_stat,
      reserved1: msg.reserved1,
    }
  }
}


// Corresponds to ublox_msgs__msg__RxmRAWX
/// RXM-RAWX (0x02 0x15)
/// Multi-GNSS Raw Measurement Data
///
/// This message contains the information needed to be able to generate a RINEX 3
/// multi-GNSS observation file.
/// This message contains pseudorange, Doppler, carrier phase, phase lock and
/// signal quality information for GNSS satellites once signals have been
/// synchronized. This message supports all active GNSS.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub meas: Vec<super::msg::RxmRAWXMeas>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RxmRAWX::default())
  }
}

impl rosidl_runtime_rs::Message for RxmRAWX {
  type RmwMsg = super::msg::rmw::RxmRAWX;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        rcv_tow: msg.rcv_tow,
        week: msg.week,
        leap_s: msg.leap_s,
        num_meas: msg.num_meas,
        rec_stat: msg.rec_stat,
        version: msg.version,
        reserved1: msg.reserved1,
        meas: msg.meas
          .into_iter()
          .map(|elem| super::msg::RxmRAWXMeas::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      rcv_tow: msg.rcv_tow,
      week: msg.week,
      leap_s: msg.leap_s,
      num_meas: msg.num_meas,
      rec_stat: msg.rec_stat,
      version: msg.version,
        reserved1: msg.reserved1,
        meas: msg.meas
          .iter()
          .map(|elem| super::msg::RxmRAWXMeas::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      rcv_tow: msg.rcv_tow,
      week: msg.week,
      leap_s: msg.leap_s,
      num_meas: msg.num_meas,
      rec_stat: msg.rec_stat,
      version: msg.version,
      reserved1: msg.reserved1,
      meas: msg.meas
          .into_iter()
          .map(super::msg::RxmRAWXMeas::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__RxmRTCM
/// RXM-RTCM (0x02 0x32)
/// RTCM input status
///
/// Output upon processing of an RTCM input message
/// Supported on:
/// - u-blox 8 / u-blox M8 from protocol version 20.01 up to version 23.01

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RxmRTCM::default())
  }
}

impl rosidl_runtime_rs::Message for RxmRTCM {
  type RmwMsg = super::msg::rmw::RxmRTCM;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        version: msg.version,
        flags: msg.flags,
        reserved0: msg.reserved0,
        ref_station: msg.ref_station,
        msg_type: msg.msg_type,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      version: msg.version,
      flags: msg.flags,
        reserved0: msg.reserved0,
      ref_station: msg.ref_station,
      msg_type: msg.msg_type,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      version: msg.version,
      flags: msg.flags,
      reserved0: msg.reserved0,
      ref_station: msg.ref_station,
      msg_type: msg.msg_type,
    }
  }
}


// Corresponds to ublox_msgs__msg__RxmSFRB
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

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RxmSFRB::default())
  }
}

impl rosidl_runtime_rs::Message for RxmSFRB {
  type RmwMsg = super::msg::rmw::RxmSFRB;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        chn: msg.chn,
        svid: msg.svid,
        dwrd: msg.dwrd,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      chn: msg.chn,
      svid: msg.svid,
        dwrd: msg.dwrd,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      chn: msg.chn,
      svid: msg.svid,
      dwrd: msg.dwrd,
    }
  }
}


// Corresponds to ublox_msgs__msg__RxmSFRBX
/// RXM-SFRB (0x02 0x13)
/// Subframe Buffer
///
/// This message reports a complete subframe of broadcast navigation data decoded
/// from a single signal. The number of data words reported in each message
/// depends on the nature of the signal. See the section on Broadcast Navigation
/// Data for further details.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub dwrd: Vec<u32>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RxmSFRBX::default())
  }
}

impl rosidl_runtime_rs::Message for RxmSFRBX {
  type RmwMsg = super::msg::rmw::RxmSFRBX;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        gnss_id: msg.gnss_id,
        sv_id: msg.sv_id,
        reserved0: msg.reserved0,
        freq_id: msg.freq_id,
        num_words: msg.num_words,
        chn: msg.chn,
        version: msg.version,
        reserved1: msg.reserved1,
        dwrd: msg.dwrd.into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      gnss_id: msg.gnss_id,
      sv_id: msg.sv_id,
      reserved0: msg.reserved0,
      freq_id: msg.freq_id,
      num_words: msg.num_words,
      chn: msg.chn,
      version: msg.version,
      reserved1: msg.reserved1,
        dwrd: msg.dwrd.as_slice().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      gnss_id: msg.gnss_id,
      sv_id: msg.sv_id,
      reserved0: msg.reserved0,
      freq_id: msg.freq_id,
      num_words: msg.num_words,
      chn: msg.chn,
      version: msg.version,
      reserved1: msg.reserved1,
      dwrd: msg.dwrd
          .into_iter()
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__RxmSVSI
/// RXM-SVSI (0x02 0x20)
/// SV Status Info
///
/// Status of the receiver manager knowledge about GPS Orbit Validity
///
/// This message has only been retained for backwards compatibility; users are
/// recommended to use the UBX-NAV-ORB message in preference.

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    pub sv: Vec<super::msg::RxmSVSISV>,

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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RxmSVSI::default())
  }
}

impl rosidl_runtime_rs::Message for RxmSVSI {
  type RmwMsg = super::msg::rmw::RxmSVSI;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        i_tow: msg.i_tow,
        week: msg.week,
        num_vis: msg.num_vis,
        num_sv: msg.num_sv,
        sv: msg.sv
          .into_iter()
          .map(|elem| super::msg::RxmSVSISV::into_rmw_message(std::borrow::Cow::Owned(elem)).into_owned())
          .collect(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      i_tow: msg.i_tow,
      week: msg.week,
      num_vis: msg.num_vis,
      num_sv: msg.num_sv,
        sv: msg.sv
          .iter()
          .map(|elem| super::msg::RxmSVSISV::into_rmw_message(std::borrow::Cow::Borrowed(elem)).into_owned())
          .collect(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      i_tow: msg.i_tow,
      week: msg.week,
      num_vis: msg.num_vis,
      num_sv: msg.num_sv,
      sv: msg.sv
          .into_iter()
          .map(super::msg::RxmSVSISV::from_rmw_message)
          .collect(),
    }
  }
}


// Corresponds to ublox_msgs__msg__RxmSVSISV
/// see message RxmSVSI

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RxmSVSISV::default())
  }
}

impl rosidl_runtime_rs::Message for RxmSVSISV {
  type RmwMsg = super::msg::rmw::RxmSVSISV;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        svid: msg.svid,
        sv_flag: msg.sv_flag,
        azim: msg.azim,
        elev: msg.elev,
        age: msg.age,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      svid: msg.svid,
      sv_flag: msg.sv_flag,
      azim: msg.azim,
      elev: msg.elev,
      age: msg.age,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      svid: msg.svid,
      sv_flag: msg.sv_flag,
      azim: msg.azim,
      elev: msg.elev,
      age: msg.age,
    }
  }
}


// Corresponds to ublox_msgs__msg__TimTM2
/// TIM-TM2 (0x0D, 0x03)
/// Time mark data
///
/// Description for details.
///
/// Supported on:
///  - u-blox 8 / u-blox M8 with protocol version 22 (only with Timing Products)

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::TimTM2::default())
  }
}

impl rosidl_runtime_rs::Message for TimTM2 {
  type RmwMsg = super::msg::rmw::TimTM2;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ch: msg.ch,
        flags: msg.flags,
        rising_edge_count: msg.rising_edge_count,
        wn_r: msg.wn_r,
        wn_f: msg.wn_f,
        tow_ms_r: msg.tow_ms_r,
        tow_sub_ms_r: msg.tow_sub_ms_r,
        tow_ms_f: msg.tow_ms_f,
        tow_sub_ms_f: msg.tow_sub_ms_f,
        acc_est: msg.acc_est,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ch: msg.ch,
      flags: msg.flags,
      rising_edge_count: msg.rising_edge_count,
      wn_r: msg.wn_r,
      wn_f: msg.wn_f,
      tow_ms_r: msg.tow_ms_r,
      tow_sub_ms_r: msg.tow_sub_ms_r,
      tow_ms_f: msg.tow_ms_f,
      tow_sub_ms_f: msg.tow_sub_ms_f,
      acc_est: msg.acc_est,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ch: msg.ch,
      flags: msg.flags,
      rising_edge_count: msg.rising_edge_count,
      wn_r: msg.wn_r,
      wn_f: msg.wn_f,
      tow_ms_r: msg.tow_ms_r,
      tow_sub_ms_r: msg.tow_sub_ms_r,
      tow_ms_f: msg.tow_ms_f,
      tow_sub_ms_f: msg.tow_sub_ms_f,
      acc_est: msg.acc_est,
    }
  }
}


// Corresponds to ublox_msgs__msg__UpdSOSAck
/// UPD-SOS (0x09 0x14)
///
/// Backup File Creation Acknowledge / System Restored from Backup
///
/// Firmware Supported on:
/// u-blox 8 / u-blox M8 from protocol version 15 up to version 23.01

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::UpdSOSAck::default())
  }
}

impl rosidl_runtime_rs::Message for UpdSOSAck {
  type RmwMsg = super::msg::rmw::UpdSOSAck;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        cmd: msg.cmd,
        reserved0: msg.reserved0,
        response: msg.response,
        reserved1: msg.reserved1,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      cmd: msg.cmd,
        reserved0: msg.reserved0,
      response: msg.response,
        reserved1: msg.reserved1,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      cmd: msg.cmd,
      reserved0: msg.reserved0,
      response: msg.response,
      reserved1: msg.reserved1,
    }
  }
}


// Corresponds to ublox_msgs__msg__UpdSOS
/// UPD-SOS (0x09 0x14)
///
/// Firmware Supported on:
/// u-blox 8 / u-blox M8 from protocol version 15 up to version 23.01

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::UpdSOS::default())
  }
}

impl rosidl_runtime_rs::Message for UpdSOS {
  type RmwMsg = super::msg::rmw::UpdSOS;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        cmd: msg.cmd,
        reserved1: msg.reserved1,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      cmd: msg.cmd,
        reserved1: msg.reserved1,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      cmd: msg.cmd,
      reserved1: msg.reserved1,
    }
  }
}


