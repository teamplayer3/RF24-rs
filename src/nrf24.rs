//! nRF24 implementations.

use crate::config::{AddressWidth, DataPipe, DataRate, EncodingScheme, NrfConfig, PALevel};
use crate::error::Error;
use crate::hal::blocking::{
    delay::DelayMs,
    delay::DelayUs,
    spi::{Transfer, Write},
};

use crate::hal::digital::v2::OutputPin;
use crate::register_acces::{Instruction, Register};
use crate::status::Status;
use crate::MAX_PAYLOAD_SIZE;
use core::fmt;

/// The nRF24L01 driver type. This struct encapsulates all functionality.
///
/// # Example
/// ```
/// use nrf24::{Nrf24l01, MAX_PAYLOAD_SIZE};
///
/// // Initialized and started up chip
/// let nrf24 = Nrf24l01::new(spi, ce, ncs, &mut delay, NrfConfig::default()).unwrap();
///
///
/// ```
///
pub struct Nrf24l01<SPI, CE, NCS> {
    spi: SPI,
    // SPI Chip Select Pin, active low
    ncs: NCS,
    // Chip Enable Pin
    ce: CE,
    // Config Register
    config_reg: u8,
    // Payload size
    payload_size: PayloadSize,
    // Transmission buffer
    tx_buf: [u8; MAX_PAYLOAD_SIZE as usize + 1],
}

enum PayloadSize {
    Dynamic,
    Static(u8),
}

type Result<T, E, F> = core::result::Result<T, Error<E, F>>;

impl<SPI, CE, NCS, SPIErr, PinErr> Nrf24l01<SPI, CE, NCS>
where
    SPI: Transfer<u8, Error = SPIErr> + Write<u8, Error = SPIErr>,
    NCS: OutputPin<Error = PinErr>,
    CE: OutputPin<Error = PinErr>,
{
    const MAX_ADDR_WIDTH: usize = 5;
    const CORRECT_CONFIG: u8 = 0b00001110;
    const STATUS_RESET: u8 = 0b01110000;

    /// Creates a new nrf24l01 driver with given config.
    /// Stars up the device, so calling `power_up` isn't necessary.
    ///
    /// # Example
    /// ```
    /// // Initialize all pins required
    /// let dp = Peripherals::take()::unwrap();
    /// let mut portd = dp.PORTD.split();
    /// let ce = portd.pd3.into_output(&mut portd.ddr); // Chip Enable
    ///
    /// let mut portb = dp.PORTB.split();
    /// let ncs = portb.pb2.into_output(&mut portb.ddr); // Chip Select (active low)
    /// let mosi = portb.pb3.into_output(&mut portb.ddr); // Master Out Slave In Pin
    /// let miso = portb.pb4.into_pull_up_input(&mut portb.ddr); // Master In Slave Out Pin
    /// let sclk = portb.pb5.into_output(&mut portb.ddr); // Clock
    ///
    /// // Now we initialize SPI settings to create an SPI instance
    /// let settings = spi::Settings {
    ///     data_order: DataOrder::MostSignificantFirst,
    ///     clock: SerialClockRate::OscfOver4,
    ///     // The required SPI mode for communication with the nrf chip is specified in
    ///     // this crate
    ///     mode: nrf24_rs::MODE,
    /// };
    /// let spi = spi::Spi::new(dp.SPI, sclk, mosi, miso, settings);
    ///
    /// let mut delay = hal::delay::Delay::<clock::MHz16>::new();
    ///
    /// // Construct a new instance of the chip with a default configuration
    /// // This will initialize the module and start it up
    /// let nrf24 = nrf24_rs::Nrf24l01::new(spi, ce, ncs, &mut delay, NrfConfig::default())?;
    ///
    /// ```
    pub fn new<D>(
        spi: SPI,
        ce: CE,
        ncs: NCS,
        delay: &mut D,
        config: NrfConfig,
    ) -> Result<Self, SPIErr, PinErr>
    where
        D: DelayMs<u8>,
    {
        let mut chip = Nrf24l01 {
            spi,
            ncs,
            ce,
            config_reg: 0,
            payload_size: PayloadSize::Static(0),
            tx_buf: [0; MAX_PAYLOAD_SIZE as usize + 1],
        };

        // Set the output pins to the correct levels
        chip.ce.set_low().map_err(Error::Pin)?;
        chip.ncs.set_high().map_err(Error::Pin)?;

        // Must allow the radio time to settle else configuration bits will not necessarily stick.
        // This is actually only required following power up but some settling time also appears to
        // be required after resets too. For full coverage, we'll always assume the worst.
        // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
        // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
        delay.delay_ms(5);

        // Set retries
        chip.set_retries(config.auto_retry.delay(), config.auto_retry.count())?;
        // Set rf
        chip.setup_rf(config.data_rate, config.pa_level)?;
        // Set payload size
        chip.set_payload_size(config.payload_size)?;
        // Set address length
        chip.set_address_width(config.addr_width)?;
        // Reset status
        chip.reset_status()?;
        // This channel should be universally safe and not bleed over into adjacent spectrum.
        chip.set_channel(config.channel)?;
        // flush buffers
        chip.flush_rx()?;
        chip.flush_tx()?;

        // clear CONFIG register, Enable PTX, Power Up & 16-bit CRC
        if let Some(encoding_scheme) = config.crc_encoding_scheme {
            chip.enable_crc(encoding_scheme)?;
        }

        chip.config_reg = chip.read_register(Register::CONFIG)?;

        chip.power_up(delay)?;

        if chip.config_reg != Self::CORRECT_CONFIG {
            Err(Error::CommunicationError(chip.config_reg))
        } else {
            Ok(chip)
        }
    }

    /// Power up now.
    ///
    /// # Examples
    /// ```rust
    /// chip.power_up(&mut delay)?;
    /// ```
    pub fn power_up<D>(&mut self, delay: &mut D) -> Result<(), SPIErr, PinErr>
    where
        D: DelayMs<u8>,
    {
        // if not powered up, power up and wait for the radio to initialize
        if !self.is_powered_up() {
            // update the stored config register
            self.config_reg |= 1 << 1;
            self.write_register(Register::CONFIG, self.config_reg)?;

            delay.delay_ms(5);
        }
        Ok(())
    }

    /// Checks if the chip is connected to the SPI bus.
    pub fn is_connected(&mut self) -> Result<bool, SPIErr, PinErr> {
        let setup = self.read_register(Register::SETUP_AW)?;
        if setup >= 1 && setup <= 3 {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Opens a reading pipe.
    ///
    /// Call this before calling [start_listening()](#method.start_listening).
    pub fn open_reading_pipe(&mut self, mut addr: &[u8]) -> Result<(), SPIErr, PinErr> {
        if addr.len() > Self::MAX_ADDR_WIDTH {
            addr = &addr[0..Self::MAX_ADDR_WIDTH];
        }
        self.write_register(Register::RX_ADDR_P0, addr)?;

        // Enable RX Addr 0
        let old_reg = self.read_register(Register::EN_RXADDR)?;
        self.write_register(Register::EN_RXADDR, old_reg | 1)?;

        Ok(())
    }

    /// Opens a writing pipe.
    ///
    /// Must be called before writing data.
    pub fn open_writing_pipe(&mut self, mut addr: &[u8]) -> Result<(), SPIErr, PinErr> {
        if addr.len() > Self::MAX_ADDR_WIDTH {
            addr = &addr[0..Self::MAX_ADDR_WIDTH];
        }
        // We need to open Reading Pipe 0 with the same address name
        // because ACK messages will be recieved on this channel
        self.write_register(Register::RX_ADDR_P0, addr)?;
        // Open writing pipe
        self.write_register(Register::TX_ADDR, addr)?;

        Ok(())
    }

    /// Starts listening on the pipes that are opened for reading.
    /// Used in Receiver Mode.
    ///
    /// Make sure [open_reading_pipe()](#method.open_reading_pipe) is called first.
    ///
    /// TODO: Use the type system to make start and stop listening by RAII and Drop
    pub fn start_listening(&mut self) -> Result<(), SPIErr, PinErr> {
        // Enable RX listening flag
        self.config_reg |= 1;
        self.write_register(Register::CONFIG, self.config_reg)?;
        // Flush interrupts
        self.reset_status()?;

        self.ce.set_high().map_err(Error::Pin)?;

        Ok(())
    }

    /// Stop listening.
    ///
    /// TODO: Use the type system to make start and stop listening by RAII and Drop
    pub fn stop_listening(&mut self) -> Result<(), SPIErr, PinErr> {
        self.ce.set_low().map_err(Error::Pin)?;

        self.config_reg &= !0b1;
        self.write_register(Register::CONFIG, self.config_reg)?;

        Ok(())
    }

    /// Check if there are any bytes available to be read.
    pub fn data_available(&mut self) -> Result<bool, SPIErr, PinErr> {
        Ok(self.data_available_on_pipe()?.is_some())
    }

    /// Returns the data pipe where the data is available and `None` if no data available.
    pub fn data_available_on_pipe(&mut self) -> Result<Option<DataPipe>, SPIErr, PinErr> {
        Ok(self.status()?.data_pipe_available())
    }

    /// Read the available payload.
    ///
    /// Returns the number of bytes read into the buffer.
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, SPIErr, PinErr> {
        let len = if let PayloadSize::Static(n) = self.payload_size {
            n as usize
        } else {
            core::cmp::min(buf.len(), MAX_PAYLOAD_SIZE as usize)
        };

        // Use tx buffer to copy the values into
        // First byte will be the opcode
        self.tx_buf[0] = Instruction::RRX.opcode();
        // Write to spi
        self.ncs.set_low().map_err(Error::Pin)?;
        let r = self
            .spi
            .transfer(&mut self.tx_buf[..=len])
            .map_err(Error::Spi)?;
        self.ncs.set_high().map_err(Error::Pin)?;

        // Transfer the data read to buf.
        // Skip first byte because it contains the command.
        // Make both slices are the same length, otherwise `copy_from_slice` panics.
        buf[..len].copy_from_slice(&r[1..=len]);
        Ok(len)
    }

    /// Writes data on the opened channel
    ///
    /// Will clear all interrupt flags after write.
    /// Returns an error when max retries has been reached.
    pub fn write<D: DelayUs<u8>>(
        &mut self,
        delay: &mut D,
        buf: &[u8],
    ) -> Result<(), SPIErr, PinErr> {
        let send_count = if let PayloadSize::Static(n) = self.payload_size {
            let n = n as usize;
            // we have to send `n` bytes
            let len = core::cmp::min(buf.len(), n);
            self.tx_buf[1..=len].copy_from_slice(&buf[..len]);
            if len < MAX_PAYLOAD_SIZE as usize {
                self.tx_buf[len + 1..=n].fill(0);
            }
            // now our tx_buf is guarantueed to have `n` bytes filled
            n
        } else {
            // In dynamic payload mode, max payload_size is the limit
            core::cmp::min(buf.len(), MAX_PAYLOAD_SIZE as usize)
        };

        // Add instruction to buffer
        self.tx_buf[0] = Instruction::WTX.opcode();
        // Write to spi
        self.ncs.set_low().map_err(Error::Pin)?;
        let r = self
            .spi
            .transfer(&mut self.tx_buf[..=send_count])
            .map_err(Error::Spi)?;
        self.ncs.set_high().map_err(Error::Pin)?;

        let status = Status::from(r[0]);

        // Start transmission:
        // pulse CE pin to signal transmission start
        self.ce.set_high().map_err(Error::Pin)?;
        delay.delay_us(10);
        self.ce.set_low().map_err(Error::Pin)?;

        // Clear interrupt flags
        self.write_register(Register::STATUS, Status::flags().value())?;

        // Max retries exceeded
        if status.reached_max_retries() {
            self.flush_tx()?;
            return Err(Error::MaxRT);
        }

        Ok(())
    }

    /// Setup of automatic retransmission.
    ///
    /// # Arguments
    /// * `delay` is the auto retransmit delay.
    /// Values can be between 0 and 15.
    /// The delay before a retransmit is initiated, is calculated according to the following formula:
    /// > ((**delay** + 1) * 250) + 86 µs
    ///
    /// * `count` is number of times there will be an auto retransmission.
    /// Must be a value between 0 and 15.
    ///
    /// # Examples
    /// ```rust
    /// // Set the auto transmit delay to (5 + 1) * 250) + 86 = 1586µs
    /// // and the retransmit count to 15.
    /// nrf24l01.set_retries(5, 15)?;
    /// ```
    pub fn set_retries(&mut self, delay: u8, count: u8) -> Result<(), SPIErr, PinErr> {
        self.write_register(Register::SETUP_RETR, (delay << 4) | (count))
    }

    /// Return the delay between auto retransmissions in ms.
    pub fn auto_retry_delay(&mut self) -> Result<u32, SPIErr, PinErr> {
        self.read_register(Register::SETUP_RETR)
            .map(|raw| (raw >> 4) as u32)
            .map(|x| ((x + 1) * 250) + 86)
    }

    /// Return the number of times there will be an auto retransmissions.
    pub fn auto_retry_attempts(&mut self) -> Result<u8, SPIErr, PinErr> {
        self.read_register(Register::SETUP_RETR)
            .map(|x| x & 0b0000_1111)
    }

    /// Set the frequency channel nRF24L01 operates on.
    ///
    /// # Arguments
    ///
    /// * `channel` number between 0 and 127.
    ///
    /// # Examples
    /// ```rust
    /// nrf24l01.set_channel(74)?;
    /// ```
    pub fn set_channel(&mut self, channel: u8) -> Result<(), SPIErr, PinErr> {
        self.write_register(Register::RF_CH, (u8::MAX >> 1) & channel)
    }

    /// Return the frequency channel nRF24L01 operates on.
    ///
    /// The actual frequency will we the channel +2400 MHz.
    pub fn channel(&mut self) -> Result<u8, SPIErr, PinErr> {
        self.read_register(Register::RF_CH)
    }

    /// Set the address width, saturating values above or below allowed range.
    ///
    /// # Arguments
    ///
    /// * `width` number between 3 and 5.
    ///
    /// # Examples
    /// ```rust
    /// nrf24l01.set_address_width(5)?;
    /// ```
    pub fn set_address_width<T>(&mut self, width: T) -> Result<(), SPIErr, PinErr>
    where
        T: Into<AddressWidth>,
    {
        let width = width.into();
        self.write_register(Register::SETUP_AW, width.value())
    }

    /// Return the current data rate.
    pub fn data_rate(&mut self) -> Result<DataRate, SPIErr, PinErr> {
        self.read_register(Register::RF_SETUP).map(DataRate::from)
    }

    /// Return the current power amplifier level.
    pub fn power_amp_level(&mut self) -> Result<PALevel, SPIErr, PinErr> {
        self.read_register(Register::RF_SETUP).map(PALevel::from)
    }

    /// Flush transmission FIFO, used in TX mode.
    ///
    /// # Examples
    /// ```rust
    /// nrf24l01.flush_tx()?;
    /// ```
    pub fn flush_tx(&mut self) -> Result<(), SPIErr, PinErr> {
        self.send_command(Instruction::FTX).map(|_| ())
    }

    /// Flush reciever FIFO, used in RX mode.
    ///
    /// # Examples
    /// ```rust
    /// nrf24l01.flush_rx()?;
    /// ```
    pub fn flush_rx(&mut self) -> Result<(), SPIErr, PinErr> {
        self.send_command(Instruction::FRX).map(|_| ())
    }

    /// Enable CRC encoding scheme.
    ///
    /// **Note** that this configures the nrf24l01 in transmit mode.
    ///
    /// # Examples
    /// ```rust
    /// chip.enable_crc(EncodingScheme::R2Bytes)?;
    /// ```
    pub fn enable_crc(&mut self, scheme: EncodingScheme) -> Result<(), SPIErr, PinErr> {
        self.write_register(Register::CONFIG, (1 << 3) | (scheme.scheme() << 2))
    }

    /// Configure the data rate and PA level.
    pub fn configure(&mut self, data_rate: DataRate, level: PALevel) -> Result<(), SPIErr, PinErr> {
        self.setup_rf(data_rate, level)
    }

    /// Set the payload size.
    ///
    /// Values bigger than [MAX_PAYLOAD_SIZE](constant.MAX_PAYLOAD_SIZE.html) will be set to the maximum
    pub fn set_payload_size(&mut self, payload_size: u8) -> Result<(), SPIErr, PinErr> {
        let payload_size = core::cmp::min(MAX_PAYLOAD_SIZE, payload_size);
        self.payload_size = PayloadSize::Static(payload_size);
        self.write_register(Register::RX_PW_P0, payload_size)?;
        self.write_register(Register::RX_PW_P1, payload_size)?;
        self.write_register(Register::RX_PW_P2, payload_size)?;
        self.write_register(Register::RX_PW_P3, payload_size)?;
        self.write_register(Register::RX_PW_P4, payload_size)?;
        self.write_register(Register::RX_PW_P5, payload_size)
    }

    /// Get the payload size.
    ///
    /// Guarantueed to be a value betwoon 1 and 32.
    pub fn payload_size(&self) -> u8 {
        if let PayloadSize::Static(s) = self.payload_size {
            s
        } else {
            0
        }
    }

    /// Reads the status register from device.
    pub fn status(&mut self) -> Result<Status, SPIErr, PinErr> {
        self.send_command(Instruction::NOP)
    }

    /// Resets the following flags in the status register:
    /// - data ready RX fifo interrupt
    /// - data sent TX fifo interrupt
    /// - maximum number of number of retries interrupt
    pub fn reset_status(&mut self) -> Result<(), SPIErr, PinErr> {
        self.write_register(Register::STATUS, Self::STATUS_RESET)
    }

    /// Sends an instruction over the SPI bus without extra data.
    ///
    /// Returns the status recieved from the device.
    /// Normally used for the other instructions than read and write.  
    fn send_command(&mut self, instruction: Instruction) -> Result<Status, SPIErr, PinErr> {
        self.send_command_bytes(instruction, &[])
    }

    // Sends an instruction with some payload data over the SPI bus
    //
    // Returns the status from the device
    fn send_command_bytes(
        &mut self,
        instruction: Instruction,
        buf: &[u8],
    ) -> Result<Status, SPIErr, PinErr> {
        // Use tx buffer to copy the values into
        // First byte will be the opcode
        self.tx_buf[0] = instruction.opcode();
        self.tx_buf[1..=buf.len()].copy_from_slice(buf);
        // Write to spi
        self.ncs.set_low().map_err(Error::Pin)?;
        let r = self
            .spi
            .transfer(&mut self.tx_buf[..=buf.len()])
            .map_err(Error::Spi)?;
        self.ncs.set_high().map_err(Error::Pin)?;

        Ok(Status::from(r[0]))
    }

    /// Writes values to a given register.
    ///
    /// This can be anything that can be turned into a buffer of u8's.
    /// `IntoBuf` is currently implemented for T and for &[T].
    /// This means that this function can be polymorphically called for single value writes as well
    /// as for arrays.
    fn write_register<T: IntoBuf<u8>>(
        &mut self,
        register: Register,
        buf: T,
    ) -> Result<(), SPIErr, PinErr> {
        let buf = buf.into_buf();
        // Use tx buffer to copy the values into
        // First byte will be the opcode
        self.tx_buf[0] = Instruction::WR.opcode() | register.addr();
        // Copy over the values
        self.tx_buf[1..=buf.len()].copy_from_slice(buf);
        // Write to spi
        self.ncs.set_low().map_err(Error::Pin)?;
        self.spi
            .write(&self.tx_buf[..=buf.len()])
            .map_err(Error::Spi)?;
        self.ncs.set_high().map_err(Error::Pin)?;

        Ok(())
    }

    fn read_register(&mut self, register: Register) -> Result<u8, SPIErr, PinErr> {
        let mut buffer = [Instruction::RR.opcode() | register.addr(), 0];
        self.ncs.set_low().map_err(Error::Pin)?;
        self.spi.transfer(&mut buffer).map_err(Error::Spi)?;
        self.ncs.set_high().map_err(Error::Pin)?;
        Ok(buffer[1])
    }

    fn setup_rf(&mut self, data_rate: DataRate, level: PALevel) -> Result<(), SPIErr, PinErr> {
        self.write_register(Register::RF_SETUP, data_rate.rate() | level.level())
    }

    fn is_powered_up(&self) -> bool {
        self.config_reg & (1 << 1) != 0
    }
}

impl<SPI, CE, NCS> fmt::Debug for Nrf24l01<SPI, CE, NCS>
where
    SPI: fmt::Debug,
    CE: fmt::Debug,
    NCS: fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Nrf24l01")
            .field("spi", &self.spi)
            .field("ncs", &self.ncs)
            .field("ce", &self.ce)
            .field("config_reg", &self.config_reg)
            //.field("payload_size", &self.payload_size)
            .field("tx_buf", &&self.tx_buf[1..])
            .finish()
    }
}

/// A trait representing a type that can be turned into a buffer.
///
/// Is used for representing single values as well as slices as buffers.
trait IntoBuf<T> {
    fn into_buf(&self) -> &[T];
}

impl<T> IntoBuf<T> for T {
    fn into_buf(&self) -> &[T] {
        core::slice::from_ref(self)
    }
}
impl<T> IntoBuf<T> for &[T] {
    fn into_buf(&self) -> &[T] {
        self
    }
}
