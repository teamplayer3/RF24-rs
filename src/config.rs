//! Different structs and values for configuration of the chip.
//!
//! To construct a representation of your config, see [`NrfConfig`].
//!
//! # Default values
//! All these options have a default value:
//!
//! * `addr_width`:             address width of 5 bytes.
//! * `ack_payloads_enabled`:   false: acknowledgement payloads are disabled by default.
//! * `auto_retry`:             enabled, will wait 1586µs on ack, and will retry 15 times.
//! * `channel`:                channel 76.
//! * `crc_encoding_scheme`:    encoding scheme with 2 bytes.
//! * `data_rate`:              1Mbps.
//! * `payload_size`:           static payload size of [`MAX_PAYLOAD_SIZE`] bytes.
//! * `pa_level`:               min amplification level.
//!
use core::marker::PhantomData;
use core::ops::Deref;

use crate::nrf24::IntoBuf;
use crate::MAX_PAYLOAD_SIZE;
use crate::{register_acces::Register, status::Status};
#[cfg(feature = "micro-fmt")]
use ufmt::{uDebug, uWrite, uwrite, Formatter};

const MAX_CHANNEL: u8 = 125;

/// Configuration builder struct for NRF chip.
///
/// Always created with the `default()` method and modified through
/// the builder pattern.
///
/// # Example: default
/// ```rust
/// use nrf24::Nrf24l01;
/// use nrf24::config::NrfConfig;
///
/// let config = NrfConfig::default();
///
/// let mut chip = Nrf24l01::new(spi, ce, ncs, delay, config)?;
/// ```
///
/// # Example: custom configuration
/// ```rust
/// use nrf24::Nrf24l01;
/// use nrf24::config::{PALevel, DataRate, NrfConfig, PayloadSize};
///
/// let config = NrfConfig::default()
///     .payload_size(PayloadSize::Dynamic) // set dynamic payload size
///     .channel(7)
///     .addr_width(3),
///     .data_rate(DataRate::R2Mbps)
///     .pa_level(PALevel::Max)
///     .crc_encoding_scheme(None) // disable crc
///     .ack_payloads_enabled(true)
///     .auto_retry((15, 15));
///
/// let mut chip = Nrf24l01::new(spi, ce, ncs, delay, config)?;
/// ```
#[derive(Copy, Debug, Clone)]
pub struct NrfConfig<const N: usize> {
    pub(crate) pipe_config: PipeConfig<N>,
    pub(crate) channel: u8,
    pub(crate) addr_width: AddressWidth,
    pub(crate) data_rate: DataRate,
    pub(crate) pa_level: PALevel,
    pub(crate) crc_encoding_scheme: Option<EncodingScheme>,
    pub(crate) ack_payloads_enabled: bool,
    pub(crate) auto_retry: AutoRetransmission,
    pub(crate) auto_ack_enabled: bool,
}

impl<const N: usize> NrfConfig<N> {
    /// Set Payload Size
    /// A value of 0 means dynamic payloads will be enabled.
    /// Values greater than [`MAX_PAYLOAD_SIZE`] will be floored.
    pub fn rx_pipe_config<A: Into<PipeConfig<N>>>(mut self, pipe_config: A) -> Self {
        self.pipe_config = pipe_config.into();
        self
    }
    /// Set RF channel
    /// Must be a number in [0..125], values outside will be clipped
    pub fn channel(mut self, channel: u8) -> Self {
        self.channel = core::cmp::min(channel, MAX_CHANNEL);
        self
    }
    /// Set the Address Width
    /// If using a number, it must be in [3..5], values outside will be clipped
    pub fn addr_width<T: Into<AddressWidth>>(mut self, addr_width: T) -> Self {
        self.addr_width = addr_width.into();
        self
    }
    /// Set the Data Rate
    pub fn data_rate(mut self, data_rate: DataRate) -> Self {
        self.data_rate = data_rate;
        self
    }
    /// Set the Power Amplification Level
    pub fn pa_level(mut self, pa_level: PALevel) -> Self {
        self.pa_level = pa_level;
        self
    }
    /// Set the Cyclic Redundancy Check Encodign Scheme
    /// None will disable the CRC.
    pub fn crc_encoding_scheme(mut self, crc_encoding_scheme: Option<EncodingScheme>) -> Self {
        self.crc_encoding_scheme = crc_encoding_scheme;
        self
    }
    /// Configure if auto acknowledgements are enabled
    pub fn ack_payloads_enabled(mut self, ack_payloads_enabled: bool) -> Self {
        self.ack_payloads_enabled = ack_payloads_enabled;
        self
    }
    /// Set the automatic retransmission config
    pub fn auto_retry<T: Into<AutoRetransmission>>(mut self, auto_retry: T) -> Self {
        self.auto_retry = auto_retry.into();
        self
    }

    /// Configure if auto acknowledgement enabled
    pub fn auto_ack(mut self, auto_ack_enabled: bool) -> Self {
        self.auto_ack_enabled = auto_ack_enabled;
        self
    }
}

impl Default for NrfConfig<5> {
    fn default() -> Self {
        Self {
            channel: 76,
            pipe_config: PipeConfigBuilder::<Static, 5>::default().inner,
            addr_width: AddressWidth::default(),
            crc_encoding_scheme: Some(EncodingScheme::R2Bytes),
            pa_level: PALevel::default(),
            data_rate: DataRate::default(),
            ack_payloads_enabled: false,
            auto_retry: AutoRetransmission::default(),
            auto_ack_enabled: false,
        }
    }
}

#[cfg(feature = "micro-fmt")]
impl<const N: usize> uDebug for NrfConfig<N> {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        f.debug_struct("nRF configuration")?
            .field("channel", &self.channel)?
            .field("rx_pipe_config", &self.pipe_config)?
            .field("power amplification level", &self.pa_level)?
            .field("data rate", &self.data_rate)?
            .field("auto retransmission", &self.auto_retry)?
            .field(
                "acknowledgement payloads enabled",
                &self.ack_payloads_enabled,
            )?
            .field("address width", &self.addr_width)?
            .field("crc encoding scheme", &self.crc_encoding_scheme)?
            .finish()
    }
}

/// A trait used by the `PipeConfigBuilder` struct to determine if it should build a config with a static or dynamic payload configuration.
pub trait PayloadSizedType {}

/// The static variant of the payload type.
#[derive(Debug, Clone, Copy)]
pub struct Static;
/// The static variant of the payload type.
#[derive(Debug, Clone, Copy)]
pub struct Dynamic;

impl PayloadSizedType for Static {}
impl PayloadSizedType for Dynamic {}

/// A builder to build the rx pipe configuration in a convenient way.
#[derive(Debug, Clone, Copy)]
pub struct PipeConfigBuilder<T, const N: usize> {
    inner: PipeConfig<N>,
    payload_size_type: PhantomData<T>,
}

/// Configuration of the receive pipes p0-p5.
#[derive(Debug, Clone, Copy)]
pub struct PipeConfig<const N: usize> {
    pub(crate) pipes: PipesPayloadSize<N>,
}

impl<const N: usize> PipeConfig<N> {
    /// Start building a pipe config with a static payload. This means, for every enabled pipe a static payload must be specified.
    pub fn static_payload() -> PipeConfigBuilder<Static, N> {
        PipeConfigBuilder::static_payload()
    }

    /// Start building a pipe config resulting in a dynamic payload setup. This means, the module determines dynamically the payload size of a packet.
    pub fn dynamic_payload() -> PipeConfigBuilder<Dynamic, N> {
        PipeConfigBuilder::dynamic_payload()
    }
}

impl<T: PayloadSizedType, const N: usize> From<PipeConfigBuilder<T, N>> for PipeConfig<N> {
    fn from(p: PipeConfigBuilder<T, N>) -> Self {
        p.inner
    }
}

/// A enum to differentiate between the static and dynamic payload config.
#[derive(Debug, Clone, Copy)]
pub enum PipesPayloadSize<const N: usize> {
    /// Dynamic payload config variant.
    Dynamic(PipesDynamicSized<N>),
    /// Static payload config variant.
    Static(PipesStaticSized<N>),
}

#[cfg(feature = "micro-fmt")]
impl<const N: usize> uDebug for PipesPayloadSize<N> {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        match self {
            PipesPayloadSize::Dynamic(e) => f
                .debug_struct("dynamic payloads")?
                .field("pipes", &e.0[0])?
                .finish(),
            PipesPayloadSize::Static(e) => f
                .debug_struct("dynamic payloads")?
                .field("pip0", &e.0[0])?
                .finish(),
        }
    }
}

/// A enum to configure the pipe either as enabled or disabled.
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum PipeState<T> {
    /// The enabled variant. Holds more config infos for the pipe.
    Enable(T),
    /// The disabled variant.
    Disable,
}

#[cfg(feature = "micro-fmt")]
impl<T: uDebug> uDebug for PipeState<T> {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        match self {
            PipeState::Enable(p) => uwrite!(f, "Enabled Pipe {:?}", p),
            PipeState::Disable => uwrite!(f, "Disabled Pipe"),
        }
    }
}

/// A enum to differentiate between long pipe addresses and short ones.
/// In general the nrf module has two pipes with a address len of max. 5 bytes, which is configurable.
/// The other pipes can be configured with one byte. They share the first bytes of the long addresses.
///
///         Byte4 Byte3 Byte2 Byte1 Byte0
/// PIPE 0  0xE7  0xD3  0xF0  0x35  0x77
/// PIPE 1  0xC2  0xC2  0xC2  0xC2  0xC2
/// PIPE 2  0xC2  0xC2  -""-  -""-  0xC3
/// PIPE 3  -""-  -""-  -""-  -""-  0xC4
/// PIPE 4  -""-  -""-  -""-  -""-  0xC5
/// PIPE 5  -""-  -""-  -""-  -""-  0xC6
///
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub enum PipeAddress<const N: usize> {
    /// A long with N bytes configurable address.
    Long([u8; N]),
    /// Address based on one Byte.
    Short(u8),
}

impl<const N: usize> IntoBuf<u8> for PipeAddress<N> {
    fn into_buf(&self) -> &[u8] {
        match self {
            PipeAddress::Long(a) => a.as_slice(),
            PipeAddress::Short(a) => core::slice::from_ref(a),
        }
    }
}

impl<const N: usize> From<&str> for PipeAddress<N> {
    fn from(s: &str) -> Self {
        let mut buffer = [0u8; N];
        buffer.copy_from_slice(s[0..N].as_bytes());
        Self::Long(buffer)
    }
}

impl<const N: usize> From<&[u8]> for PipeAddress<N> {
    fn from(s: &[u8]) -> Self {
        let mut buffer = [0u8; N];
        buffer.copy_from_slice(&s[0..N]);
        Self::Long(buffer)
    }
}

#[cfg(feature = "micro-fmt")]
impl<const N: usize> uDebug for PipeAddress<N> {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        match self {
            PipeAddress::Long(a) => uwrite!(f, "Pipe Address: {:?}", a.as_slice()),
            PipeAddress::Short(a) => uwrite!(f, "Pipe Address: {}", a),
        }
    }
}

/// Contains the pipe config for dynamic sized payloads.
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub struct PipesDynamicSized<const N: usize>(pub(crate) [PipeState<(bool, PipeAddress<N>)>; 6]);

struct PipesDynamicSizedIterator<'a, const N: usize> {
    index: u8,
    pipes: &'a PipesDynamicSized<N>,
}

impl<'a, const N: usize> Iterator for PipesDynamicSizedIterator<'a, N> {
    type Item = &'a PipeState<(bool, PipeAddress<N>)>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index > 5 {
            return None;
        }
        let pipe = &self.pipes.0[0];
        self.index += 1;
        Some(pipe)
    }
}

/// Contains the pipe config for static sized payloads.
#[derive(PartialEq, Eq, Copy, Clone, Debug)]
pub struct PipesStaticSized<const N: usize>(
    pub(crate) [PipeState<(StaticPayloadSize, bool, PipeAddress<N>)>; 6],
);

macro_rules! pipe_config_static_payload_long_addr_enable {
    ( $func_name:ident, $x:expr ) => {
        /// Enables the nth pipe with a static payload configuration.
        pub fn $func_name<T: Into<StaticPayloadSize>>(
            mut self,
            payload_size: T,
            enable_auto_ack: bool,
            pipe_addr: &[u8],
        ) -> Self {
            match self.inner.pipes {
                PipesPayloadSize::Static(ref mut s) => {
                    s.0[$x] = PipeState::Enable((
                        payload_size.into(),
                        enable_auto_ack,
                        PipeAddress::from(pipe_addr),
                    ))
                }
                _ => unreachable!(),
            }
            self
        }
    };
}

macro_rules! pipe_config_static_payload_short_addr_enable {
    ( $func_name:ident, $x:expr ) => {
        /// Enables the nth pipe with a static payload configuration.
        pub fn $func_name<T: Into<StaticPayloadSize>>(
            mut self,
            payload_size: T,
            enable_auto_ack: bool,
            pipe_addr: u8,
        ) -> Self {
            match self.inner.pipes {
                PipesPayloadSize::Static(ref mut s) => {
                    s.0[$x] = PipeState::Enable((
                        payload_size.into(),
                        enable_auto_ack,
                        PipeAddress::Short(pipe_addr),
                    ))
                }
                _ => unreachable!(),
            }
            self
        }
    };
}

macro_rules! pipe_config_dynamic_payload_long_addr_enable {
    ( $func_name:ident, $x:expr ) => {
        /// Enables the nth pipe with a dynamic payload configuration.
        pub fn $func_name(mut self, enable_auto_ack: bool, pipe_addr: &[u8]) -> Self {
            match self.inner.pipes {
                PipesPayloadSize::Dynamic(ref mut s) => {
                    s.0[$x] = PipeState::Enable((enable_auto_ack, PipeAddress::from(pipe_addr)))
                }
                _ => unreachable!(),
            }
            self
        }
    };
}

macro_rules! pipe_config_dynamic_payload_short_addr_enable {
    ( $func_name:ident, $x:expr ) => {
        /// Enables the nth pipe with a dynamic payload configuration.
        pub fn $func_name(mut self, enable_auto_ack: bool, pipe_addr: u8) -> Self {
            match self.inner.pipes {
                PipesPayloadSize::Dynamic(ref mut s) => {
                    s.0[$x] = PipeState::Enable((enable_auto_ack, PipeAddress::Short(pipe_addr)))
                }
                _ => unreachable!(),
            }
            self
        }
    };
}

impl<const N: usize> PipeConfigBuilder<Static, N> {
    /// Creates a new pipe builder with a static payload configuration.
    pub fn static_payload() -> Self {
        Self {
            inner: PipeConfig {
                pipes: PipesPayloadSize::Static(PipesStaticSized([PipeState::Disable; 6])),
            },
            payload_size_type: PhantomData::default(),
        }
    }

    pipe_config_static_payload_long_addr_enable!(enable_pipe0, 0);
    pipe_config_static_payload_long_addr_enable!(enable_pipe1, 1);
    pipe_config_static_payload_short_addr_enable!(enable_pipe2, 2);
    pipe_config_static_payload_short_addr_enable!(enable_pipe3, 3);
    pipe_config_static_payload_short_addr_enable!(enable_pipe4, 4);
    pipe_config_static_payload_short_addr_enable!(enable_pipe5, 5);
}

impl<const N: usize> PipeConfigBuilder<Dynamic, N> {
    /// Creates a new pipe builder with a dynamic payload configuration.
    pub fn dynamic_payload() -> Self {
        Self {
            inner: PipeConfig {
                pipes: PipesPayloadSize::Dynamic(PipesDynamicSized([PipeState::Disable; 6])),
            },
            payload_size_type: PhantomData::default(),
        }
    }

    pipe_config_dynamic_payload_long_addr_enable!(enable_pipe0, 0);
    pipe_config_dynamic_payload_long_addr_enable!(enable_pipe1, 1);
    pipe_config_dynamic_payload_short_addr_enable!(enable_pipe2, 2);
    pipe_config_dynamic_payload_short_addr_enable!(enable_pipe3, 3);
    pipe_config_dynamic_payload_short_addr_enable!(enable_pipe4, 4);
    pipe_config_dynamic_payload_short_addr_enable!(enable_pipe5, 5);
}

#[cfg(feature = "micro-fmt")]
impl<const N: usize> uDebug for PipesDynamicSized<N> {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        f.debug_struct("dynamic sized pipes")?
            .field("pipe0", &self.0[0])?
            .field("pipe1", &self.0[1])?
            .field("pipe2", &self.0[2])?
            .field("pipe3", &self.0[3])?
            .field("pipe4", &self.0[4])?
            .field("pipe5", &self.0[5])?
            .finish()
    }
}

#[cfg(feature = "micro-fmt")]
impl<const N: usize> uDebug for PipesStaticSized<N> {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        f.debug_struct("static sized pipes")?
            .field("pipe0", &self.0[0])?
            .field("pipe1", &self.0[1])?
            .field("pipe2", &self.0[2])?
            .field("pipe3", &self.0[3])?
            .field("pipe4", &self.0[4])?
            .field("pipe5", &self.0[5])?
            .finish()
    }
}

impl<const N: usize> PipeConfig<N> {
    pub(crate) fn has_ack_enabled_pipe(&self) -> bool {
        match self.pipes {
            PipesPayloadSize::Dynamic(s) => match s.0.iter().find(|x| match x {
                PipeState::Enable(e) => e.0,
                _ => false,
            }) {
                Some(_) => return true,
                None => return false,
            },
            PipesPayloadSize::Static(s) => match s.0.iter().find(|x| match x {
                PipeState::Enable(e) => e.1,
                _ => false,
            }) {
                Some(_) => return true,
                None => return false,
            },
        }
    }
}

impl Default for PipeConfigBuilder<Static, 5> {
    fn default() -> Self {
        Self {
            inner: PipeConfig {
                pipes: PipesPayloadSize::Static(PipesStaticSized([
                    PipeState::Enable((
                        StaticPayloadSize::default(),
                        false,
                        PipeAddress::Long([0x10, 0x10, 0x10, 0x10, 0x10]),
                    )),
                    PipeState::Enable((
                        StaticPayloadSize::default(),
                        false,
                        PipeAddress::Long([0x10, 0x10, 0x10, 0x10, 0x10]),
                    )),
                    PipeState::Disable,
                    PipeState::Disable,
                    PipeState::Disable,
                    PipeState::Disable,
                ])),
            },
            payload_size_type: PhantomData::default(),
        }
    }
}

impl Default for PipeConfigBuilder<Dynamic, 5> {
    fn default() -> Self {
        Self {
            inner: PipeConfig {
                pipes: PipesPayloadSize::Dynamic(PipesDynamicSized([
                    PipeState::Enable((false, PipeAddress::Long([0x10, 0x10, 0x10, 0x10, 0x10]))),
                    PipeState::Enable((false, PipeAddress::Long([0x10, 0x10, 0x10, 0x10, 0x10]))),
                    PipeState::Disable,
                    PipeState::Disable,
                    PipeState::Disable,
                    PipeState::Disable,
                ])),
            },
            payload_size_type: PhantomData::default(),
        }
    }
}

#[cfg(feature = "micro-fmt")]
impl<const N: usize> uDebug for PipeConfig<N> {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        f.debug_struct("receive pipe configuration")?
            .field("pipes", &self.pipes)?
            .field("address_width", &N)?
            .finish()
    }
}

/// Different RF power levels. The higher the level the bigger range, but the more the current
/// consumption.
///
/// Defaults to Min.
#[derive(PartialEq, Eq, Copy, Clone)]
pub enum PALevel {
    /// -18 dBm, 7 mA current consumption.
    Min = 0b0000_0000,
    /// -12 dBm, 7.5 mA current consumption.
    Low = 0b0000_0010,
    /// -6 dBm, 9.0 mA current consumption.
    High = 0b0000_0100,
    /// -0 dBm, 11.3 mA current consumption.
    Max = 0b0000_0110,
}

impl PALevel {
    pub(crate) fn bitmask() -> u8 {
        0b0000_0110
    }
    pub(crate) fn level(&self) -> u8 {
        *self as u8
    }
}

impl Default for PALevel {
    fn default() -> Self {
        PALevel::Min
    }
}

impl From<u8> for PALevel {
    fn from(t: u8) -> Self {
        match t & Self::bitmask() {
            0b0000_0000 => Self::Min,
            0b0000_0010 => Self::Low,
            0b0000_0100 => Self::High,
            0b0000_0110 => Self::Max,
            _ => unreachable!(),
        }
    }
}
impl core::fmt::Debug for PALevel {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            PALevel::Min => f.write_str("min (-18 dBm)"),
            PALevel::Low => f.write_str("low (-12 dBm)"),
            PALevel::High => f.write_str("high (-6 dBm)"),
            PALevel::Max => f.write_str("max (0 dBm)"),
        }
    }
}

#[cfg(feature = "micro-fmt")]
impl uDebug for PALevel {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        match *self {
            PALevel::Min => f.write_str("min (-18 dBm)"),
            PALevel::Low => f.write_str("low (-12 dBm)"),
            PALevel::High => f.write_str("high (-6 dBm)"),
            PALevel::Max => f.write_str("max (0 dBm)"),
        }
    }
}

#[cfg(feature = "de-fmt")]
impl defmt::Format for PALevel {
    fn format(&self, fmt: defmt::Formatter) {
        use defmt::write;
        match *self {
            PALevel::Min => write!(fmt, "min (-18 dBm)"),
            PALevel::Low => write!(fmt, "low (-12 dBm)"),
            PALevel::High => write!(fmt, "high (-6 dBm)"),
            PALevel::Max => write!(fmt, "max (0 dBm)"),
        }
    }
}

/// Represents a static payload size.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct StaticPayloadSize(u8);

impl StaticPayloadSize {
    pub(crate) fn truncate(self) -> Self {
        Self(core::cmp::min(self.0, MAX_PAYLOAD_SIZE))
    }
}

impl Default for StaticPayloadSize {
    fn default() -> Self {
        Self(MAX_PAYLOAD_SIZE)
    }
}

impl From<u8> for StaticPayloadSize {
    fn from(size: u8) -> Self {
        Self(core::cmp::min(size, MAX_PAYLOAD_SIZE))
    }
}

impl Deref for StaticPayloadSize {
    type Target = u8;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[cfg(feature = "micro-fmt")]
impl uDebug for StaticPayloadSize {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        uwrite!(f, "{:?} byte static payloads", self.0)
    }
}

/// Enum representing the payload size.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PayloadSize {
    /// The chip will dynamically set the payload size, depending on the message size.
    Dynamic,
    /// Static payload size. Maximum value of 127.
    Static(u8),
}

impl PayloadSize {
    /// Truncates the payload size to be max [`MAX_PAYLOAD_SIZE`].
    pub(crate) fn truncate(self) -> Self {
        match self {
            Self::Dynamic => Self::Dynamic,
            Self::Static(n) => Self::Static(core::cmp::min(n, MAX_PAYLOAD_SIZE)),
        }
    }
}

impl Default for PayloadSize {
    fn default() -> Self {
        Self::Static(MAX_PAYLOAD_SIZE)
    }
}

impl From<u8> for PayloadSize {
    fn from(size: u8) -> Self {
        match size {
            0 => Self::Dynamic,
            n => Self::Static(core::cmp::min(n, MAX_PAYLOAD_SIZE)),
        }
    }
}

impl From<StaticPayloadSize> for PayloadSize {
    fn from(s: StaticPayloadSize) -> Self {
        Self::Static(s.0)
    }
}

#[cfg(feature = "micro-fmt")]
impl uDebug for PayloadSize {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        match *self {
            Self::Dynamic => f.write_str("dynamic payloads"),
            Self::Static(n) => uwrite!(f, "{:?} byte static payloads", n),
        }
    }
}

#[cfg(feature = "de-fmt")]
impl defmt::Format for PayloadSize {
    fn format(&self, fmt: defmt::Formatter) {
        use defmt::write;
        match *self {
            Self::Dynamic => write!(fmt, "dynamic payloads"),
            Self::Static(n) => write!(fmt, "{:?} byte static payloads", n),
        }
    }
}

/// Configured speed at which data will be sent.
///
/// Defaults to 2Mpbs.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum DataRate {
    /// 1 Mbps
    R1Mbps = 0b0000_0000,
    /// 2 Mbps
    R2Mbps = 0b0000_1000,
}

impl DataRate {
    pub(crate) fn bitmask() -> u8 {
        0b0000_1000
    }
    pub(crate) fn rate(&self) -> u8 {
        *self as u8
    }
}

impl Default for DataRate {
    fn default() -> Self {
        DataRate::R1Mbps
    }
}

impl From<u8> for DataRate {
    fn from(t: u8) -> Self {
        match t & Self::bitmask() {
            0b0000_0000 => Self::R1Mbps,
            0b0000_1000 => Self::R2Mbps,
            _ => unreachable!(),
        }
    }
}

#[cfg(feature = "micro-fmt")]
impl uDebug for DataRate {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        match *self {
            DataRate::R1Mbps => f.write_str("1 Mbps"),
            DataRate::R2Mbps => f.write_str("2 Mbps"),
        }
    }
}

#[cfg(feature = "de-fmt")]
impl defmt::Format for DataRate {
    fn format(&self, fmt: defmt::Formatter) {
        use defmt::write;
        match *self {
            DataRate::R1Mbps => write!(fmt, "1 Mbps"),
            DataRate::R2Mbps => write!(fmt, "2 Mbps"),
        }
    }
}

/// Cyclic Redundancy Check encoding scheme.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum EncodingScheme {
    /// 1 byte
    R1Byte = 0,
    /// 2 bytes
    R2Bytes = 1,
}

impl EncodingScheme {
    fn bitmask() -> u8 {
        0b0000_0100
    }

    pub(crate) fn scheme(&self) -> u8 {
        *self as u8
    }
}

impl From<u8> for EncodingScheme {
    fn from(t: u8) -> Self {
        match t & Self::bitmask() {
            0b0000_0000 => Self::R1Byte,
            0b0000_0100 => Self::R2Bytes,
            _ => unreachable!(),
        }
    }
}

#[cfg(feature = "micro-fmt")]
impl uDebug for EncodingScheme {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        match *self {
            Self::R1Byte => f.write_str("1 byte"),
            Self::R2Bytes => f.write_str("2 bytes"),
        }
    }
}

#[cfg(feature = "de-fmt")]
impl defmt::Format for EncodingScheme {
    fn format(&self, fmt: defmt::Formatter) {
        use defmt::write;
        match *self {
            Self::R1Byte => write!(fmt, "1 byte"),
            Self::R2Bytes => write!(fmt, "2 bytes"),
        }
    }
}

/// Address width for the reading and writing pipes.
#[derive(PartialEq, Eq, Copy, Clone)]
pub enum AddressWidth {
    /// 3 bytes
    R3Bytes = 1,
    /// 4 bytes
    R4Bytes = 2,
    /// 5 bytes
    R5Bytes = 3,
}

impl AddressWidth {
    pub(crate) fn value(&self) -> u8 {
        *self as u8
    }
    pub(crate) fn from_register(t: u8) -> Self {
        match t & 0b11 {
            0b01 => Self::R3Bytes,
            0b10 => Self::R4Bytes,
            0b11 => Self::R5Bytes,
            _ => unreachable!(),
        }
    }
}
impl Default for AddressWidth {
    fn default() -> Self {
        Self::R5Bytes
    }
}

impl From<u8> for AddressWidth {
    // from literal value
    fn from(t: u8) -> Self {
        match t {
            0..=3 => Self::R3Bytes,
            4 => Self::R4Bytes,
            5..=u8::MAX => Self::R5Bytes,
        }
    }
}

impl core::fmt::Debug for AddressWidth {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            Self::R3Bytes => f.write_str("3 bytes"),
            Self::R4Bytes => f.write_str("4 bytes"),
            Self::R5Bytes => f.write_str("5 bytes"),
        }
    }
}

#[cfg(feature = "micro-fmt")]
impl uDebug for AddressWidth {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        match *self {
            Self::R3Bytes => f.write_str("3 bytes"),
            Self::R4Bytes => f.write_str("4 bytes"),
            Self::R5Bytes => f.write_str("5 bytes"),
        }
    }
}

#[cfg(feature = "de-fmt")]
impl defmt::Format for AddressWidth {
    fn format(&self, fmt: defmt::Formatter) {
        use defmt::write;
        match *self {
            Self::R3Bytes => write!(fmt, "3 bytes"),
            Self::R4Bytes => write!(fmt, "4 bytes"),
            Self::R5Bytes => write!(fmt, "5 bytes"),
        }
    }
}

/// Configuration of automatic retransmission consisting of a retransmit delay
/// and a retransmission count.
///
/// The delay before a retransmit is initiated, is calculated according to the following formula:
/// > ((**delay** + 1) * 250) + 86 µs
///
/// # Default
///
/// * Auto retransmission delay has a default value of 5, which means `1586 µs`.
/// * The chip will try to resend a failed message 15 times by default.
#[derive(PartialEq, Eq, Copy, Clone)]
pub struct AutoRetransmission {
    delay: u8,
    count: u8,
}

impl Default for AutoRetransmission {
    fn default() -> Self {
        Self {
            delay: 5,
            count: 15,
        }
    }
}

impl AutoRetransmission {
    pub(crate) fn from_register(reg: u8) -> Self {
        Self {
            delay: reg >> 4,
            count: reg & 0b0000_1111,
        }
    }
    /// The auto retransmit delay value.
    /// Values can be between 0 and 15.
    /// The delay before a retransmit is initiated, is calculated according to the following formula:
    /// > ((**delay** + 1) * 250) + 86 µs
    pub fn raw_delay(&self) -> u8 {
        self.delay
    }

    /// Returns the delay between auto retransmissions in ms.
    pub fn delay(&self) -> u32 {
        ((self.delay as u32 + 1) * 250) + 86
    }
    /// The number of times there will be an auto retransmission.
    /// Guarantueed to be a value between 0 and 15.
    pub fn count(&self) -> u8 {
        self.count
    }
}

impl From<(u8, u8)> for AutoRetransmission {
    fn from((d, c): (u8, u8)) -> Self {
        Self {
            delay: core::cmp::min(d, 15),
            count: core::cmp::min(c, 15),
        }
    }
}

impl core::fmt::Debug for AutoRetransmission {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AutoRetransmission")
            .field("raw delay value", &self.raw_delay())
            .field("delay (µs)", &self.delay())
            .field("count", &self.count())
            .finish()
    }
}

#[cfg(feature = "micro-fmt")]
impl uDebug for AutoRetransmission {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        f.debug_struct("AutoRetransmission")?
            .field("raw delay value", &self.raw_delay())?
            .field("delay (µs)", &self.delay())?
            .field("count", &self.count())?
            .finish()
    }
}

#[cfg(feature = "de-fmt")]
impl defmt::Format for AutoRetransmission {
    fn format(&self, fmt: defmt::Formatter) {
        use defmt::write;
        write!(
            fmt,
            "AutoRetransmission {{ raw delay value: {} , delay (µs): {}, count: {} }}",
            &self.raw_delay(),
            &self.delay(),
            &self.count()
        );
    }
}

/// Representation of the different data pipes through which data can be received.
///
/// An nRF24L01 configured as primary RX (PRX) will be able to receive data trough 6 different data
/// pipes.
/// One data pipe will have a unique address but share the same frequency channel.
/// This means that up to 6 different nRF24L01 configured as primary TX (PTX) can communicate with
/// one nRF24L01 configured as PRX, and the nRF24L01 configured as PRX will be able to distinguish
/// between them.
///
/// The default assumed data pipe is 0.
///
/// Data pipe 0 has a unique 40 bit configurable address. Each of data pipe 1-5 has an 8 bit unique
/// address and shares the 32 most significant address bits.
///
/// # Notes
/// In the PTX device data pipe 0 is used to received the acknowledgement, and therefore the
/// receive address for data pipe 0 has to be equal to the transmit address to be able to receive
/// the acknowledgement.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
#[repr(u8)]
pub enum DataPipe {
    /// Data pipe 0.
    /// Default pipe with a 40 bit configurable address.
    /// This pipe is used in TX mode when auto acknowledgement is enabled. On this channel the ACK
    /// messages are received.
    DP0 = 0,
    /// Data pipe 1.
    DP1 = 1,
    /// Data pipe 2.
    DP2 = 2,
    /// Data pipe 3.
    DP3 = 3,
    /// Data pipe 4.
    DP4 = 4,
    /// Data pipe 5.
    DP5 = 5,
}

impl DataPipe {
    pub(crate) fn pipe(&self) -> u8 {
        *self as u8
    }

    pub(crate) fn addr_len_register(&self) -> Register {
        match *self {
            DataPipe::DP0 => Register::RX_PW_P0,
            DataPipe::DP1 => Register::RX_PW_P1,
            DataPipe::DP2 => Register::RX_PW_P2,
            DataPipe::DP3 => Register::RX_PW_P3,
            DataPipe::DP4 => Register::RX_PW_P4,
            DataPipe::DP5 => Register::RX_PW_P5,
        }
    }
}

impl Default for DataPipe {
    fn default() -> Self {
        DataPipe::DP0
    }
}

impl From<u8> for DataPipe {
    fn from(t: u8) -> Self {
        match t {
            0 => DataPipe::DP0,
            1 => DataPipe::DP1,
            2 => DataPipe::DP2,
            3 => DataPipe::DP3,
            4 => DataPipe::DP4,
            5 => DataPipe::DP5,
            _ => DataPipe::DP0,
        }
    }
}

impl Into<Register> for DataPipe {
    fn into(self) -> Register {
        match self {
            DataPipe::DP0 => Register::RX_ADDR_P0,
            DataPipe::DP1 => Register::RX_ADDR_P1,
            DataPipe::DP2 => Register::RX_ADDR_P2,
            DataPipe::DP3 => Register::RX_ADDR_P3,
            DataPipe::DP4 => Register::RX_ADDR_P4,
            DataPipe::DP5 => Register::RX_ADDR_P5,
        }
    }
}

#[cfg(feature = "micro-fmt")]
impl uDebug for DataPipe {
    fn fmt<W: ?Sized>(&self, f: &mut Formatter<'_, W>) -> core::result::Result<(), W::Error>
    where
        W: uWrite,
    {
        match *self {
            DataPipe::DP0 => f.write_str("data pipe 0"),
            DataPipe::DP1 => f.write_str("data pipe 1"),
            DataPipe::DP2 => f.write_str("data pipe 2"),
            DataPipe::DP3 => f.write_str("data pipe 3"),
            DataPipe::DP4 => f.write_str("data pipe 4"),
            DataPipe::DP5 => f.write_str("data pipe 5"),
        }
    }
}

#[cfg(feature = "de-fmt")]
impl defmt::Format for DataPipe {
    fn format(&self, fmt: defmt::Formatter) {
        use defmt::write;
        match *self {
            DataPipe::DP0 => write!(fmt, "data pipe 0"),
            DataPipe::DP1 => write!(fmt, "data pipe 1"),
            DataPipe::DP2 => write!(fmt, "data pipe 2"),
            DataPipe::DP3 => write!(fmt, "data pipe 3"),
            DataPipe::DP4 => write!(fmt, "data pipe 4"),
            DataPipe::DP5 => write!(fmt, "data pipe 5"),
        }
    }
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "de-fmt", derive(defmt::Format))]
pub(crate) enum Mode {
    TransmissionMode,
    ReceiverMode,
}

/// A struct to collect all debug infos of the nrf module.
#[derive(Copy, Clone)]
pub struct DebugInfo {
    pub(crate) channel: u8,
    pub(crate) data_rate: DataRate,
    pub(crate) pa_level: PALevel,
    pub(crate) crc_encoding_scheme: Option<EncodingScheme>,
    pub(crate) payload_size: PayloadSize,
    pub(crate) retry_setup: AutoRetransmission,
    pub(crate) mode: Mode,
    pub(crate) addr_width: AddressWidth,
    pub(crate) tx_addr: [u8; 5],
    pub(crate) rx0_addr: [u8; 5],
    pub(crate) rx1_addr: [u8; 5],
    pub(crate) auto_ack: u8,
    pub(crate) open_read_pipes: u8,
    pub(crate) awaits_ack: bool,
    pub(crate) status: Status,
}

impl core::fmt::Debug for DebugInfo {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Nrf24l01")
            .field("channel", &self.channel)
            .field("frequency", &(self.channel as u16 + 2400))
            .field("data_rate", &self.data_rate)
            .field("pa_level", &self.pa_level)
            .field("crc_encoding_scheme", &self.crc_encoding_scheme)
            .field("payload_size", &self.payload_size)
            .field("retry_setup", &self.retry_setup)
            .field("mode", &self.mode)
            .field("address_width", &self.addr_width)
            .field("awaits_ack", &self.awaits_ack)
            .field("tx_address", &core::str::from_utf8(&self.tx_addr).unwrap())
            .field("rx0_address", &format_args!("{:x?}", &self.rx0_addr))
            .field("rx1_address", &format_args!("{:x?}", &self.rx1_addr))
            .field("auto_ack_channels", &format_args!("{:06b}", self.auto_ack))
            .field(
                "enabled_rx_addresses",
                &format_args!("{:06b}", self.open_read_pipes),
            )
            .field("status", &self.status)
            .finish()
    }
}

#[cfg(feature = "de-fmt")]
impl defmt::Format for DebugInfo {
    fn format(&self, fmt: defmt::Formatter) {
        use defmt::write;

        write!(
            fmt,
            "Nrf24l01 {{ channel: {}, frequency: {}, data_rate: {}, pa_level: {}, crc_encoding_scheme: {}, payload_size: {}, retry_setup: {}, mode: {}, address_width: {}, awaits_ack: {}, tx_address: {:x}, rx0_address: {:x}, rx1_address: {:x}, status: {} auto_ack_channels: {:06b} }}",
            &self.channel,
            &(self.channel as u16 + 2400),
            &self.data_rate,
            &self.pa_level,
            &self.crc_encoding_scheme,
            &self.payload_size,
            &self.retry_setup,
            &self.mode,
            &self.addr_width,
            &self.awaits_ack,
            &self.tx_addr,
            &self.rx0_addr,
            &self.rx1_addr,
            &self.status,
            &self.auto_ack
        );
    }
}
