//! # Rust driver for MCP4728 4-channel 12-bit I2C DAC
//!
//! This is a platform agnostic rust driver for the MCP4728 DAC using the [embedded-hal](https://github.com/rust-embedded/embedded-hal) traits.
//!
//! # Example
//!
//! ```no_run
//! # #[cfg(target_os = "linux")] {
//! use linux_embedded_hal::I2cdev;
//! use mcp4728::{MCP4728};
//!
//! let i2c = I2cdev::new("/dev/i2c-1").unwrap();
//! let mut dac = MCP4728::new(i2c, 0x60);
//! dac.fast_write(483, 279, 297, 590).unwrap();
//! # }
//! ```
#![cfg_attr(not(test), no_std)]

extern crate embedded_hal as hal;

use hal::blocking::i2c;
use num_enum::{IntoPrimitive, TryFromPrimitive};

const ADDRESS_GENERAL_CALL: u8 = 0x00;
const COMMAND_GENERAL_CALL_RESET: u8 = 0b00000110;
const COMMAND_GENERAL_CALL_WAKE_UP: u8 = 0b00001001;
const COMMAND_GENERAL_CALL_SOFTWARE_UPDATE: u8 = 0b00001000;
const COMMAND_SINGLE_WRITE: u8 = 0b01011000;
const COMMAND_MULTI_WRITE: u8 = 0b01000000;
const COMMAND_SEQUENTIAL_WRITE: u8 = 0b01010000;
const COMMAND_WRITE_VOLTAGE_REFERENCE_MODE: u8 = 0b10000000;
const COMMAND_WRITE_GAIN_MODE: u8 = 0b11000000;
const COMMAND_WRITE_POWER_DOWN_MODE: u8 = 0b10100000;

// Error type.

/// Error type for the crate, which can represent either an error from this driver or an inner
/// error that comes from the I2C type.
#[derive(Debug, PartialEq, Eq)]
pub enum Error<InnerError> {
    /// A value was larger than the DAC supports.
    ///
    /// The MCP4728 is a 12-bit DAC, so values that it writes must be smaller than 2^12.
    ValueOutOfBounds(u16),
    /// An internal buffer overflowed.
    BufferOverflow,
    /// A sequential write command was issued with a list of updates that didn't match the
    /// associated starting channel.
    ///
    /// For example, a sequential write command that starts with channel B must contain 3 updates:
    /// for channels B, C, and D.
    StartingChannelNotEqualToUpdateLength,
    /// Error representing an error that came from the inner I2C driver.
    I2CError(InnerError),
}

impl<InnerError> From<InnerError> for Error<InnerError> {
    fn from(inner: InnerError) -> Self {
        Error::I2CError(inner)
    }
}

// Enums for configuration.

/// Output channel selection.
#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq, Eq, Copy, Clone)]
#[repr(u8)]
pub enum Channel {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
}

/// Configuration bit for whether to update the analog output.
#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq, Eq, Copy, Clone)]
#[repr(u8)]
pub enum OutputEnableMode {
    /// The analog output will be updated immediately after the command is received.
    Update = 0,
    /// The analog output will not be updated automatically.
    ///
    /// Note that there are other ways to update the analog outputs:
    ///  - Setting the LDAC pin high
    ///  - Issuing a [`MCP4728::general_call_software_update`]
    NoUpdate = 1,
}

/// Configuration bit for which voltage reference a channel should use.
#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq, Eq, Copy, Clone)]
#[repr(u8)]
pub enum VoltageReferenceMode {
    /// Use the external pin VDD as a voltage reference.
    External = 0,
    /// Use the internal 2.048V reference.
    Internal = 1,
}

/// Configuration bits for the powered-down state of a channel.
#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq, Eq, Copy, Clone)]
#[repr(u8)]
pub enum PowerDownMode {
    /// Channel is not powered down.
    Normal = 0,
    /// Channel is powered down and output pin is connected to ground through a 1K resistor.
    PowerDownOneK = 1,
    /// Channel is powered down and output pin is connected to ground through a 100K resistor.
    PowerDownOneHundredK = 2,
    /// Channel is powered down and output pin is connected to ground through a 500K resistor.
    PowerDownFiveHundredK = 3,
}

/// Configuration bit for the gain selection mode of a channel.
///
/// If the channel is using an external reference, this bit is ignored and a gain of 1x is always
/// used.
#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq, Eq, Copy, Clone)]
#[repr(u8)]
pub enum GainMode {
    /// Gain is set to unity (1x).
    TimesOne = 0,
    /// Gain is set to unity (2x).
    TimesTwo = 1,
}

// Enums for status from reads.

/// Status of the EEPROM.
#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq, Eq, Copy, Clone)]
#[repr(u8)]
pub enum ReadyState {
    /// The EEPROM is not busy.
    Ready = 0,
    /// The EEPROM is busy.
    ///
    /// Any additional commands recieved while busy will be ignored.
    Busy = 1,
}

/// The power-on state of the entire device.
#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq, Eq, Copy, Clone)]
#[repr(u8)]
pub enum PowerState {
    /// The device is powered off.
    Off = 0,
    /// The device is powered on.
    On = 1,
}

// Container structs.

/// Representation of all registers of an individual channel.
///
/// Used only for reads.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub struct ChannelRegisters {
    /// All mode configuration bits and value of the channel.
    pub channel_state: ChannelState,
    /// The EEPROM Ready state of the device.
    ///
    /// Note that this is global to the entire device, but the protocol reports it for each
    /// channel so that is duplicated here.
    pub ready_state: ReadyState,
    /// The Power-on-reset state of the device.
    ///
    /// Note that this is global to the entire device, but the protocol reports it for each
    /// channel so that is duplicated here.
    pub power_state: PowerState,
}

/// Representation of all registers of all channels.
///
/// Used only for reads.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub struct Registers {
    /// Contents of the DAC input register for channel A.
    pub channel_a_input: ChannelRegisters,
    /// Contents of the EEPROM register for channel A.
    pub channel_a_eeprom: ChannelRegisters,
    /// Contents of the DAC input register for channel B.
    pub channel_b_input: ChannelRegisters,
    /// Contents of the EEPROM register for channel B.
    pub channel_b_eeprom: ChannelRegisters,
    /// Contents of the DAC input register for channel C.
    pub channel_c_input: ChannelRegisters,
    /// Contents of the EEPROM register for channel C.
    pub channel_c_eeprom: ChannelRegisters,
    /// Contents of the DAC input register for channel D.
    pub channel_d_input: ChannelRegisters,
    /// Contents of the EEPROM register for channel D.
    pub channel_d_eeprom: ChannelRegisters,
}

/// Representation of the register of an indivdual channel.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub struct ChannelState {
    /// The voltage reference mode.
    pub voltage_reference_mode: VoltageReferenceMode,
    /// The power-down mode.
    pub power_down_mode: PowerDownMode,
    /// The gain mode.
    pub gain_mode: GainMode,
    /// The 12-bit value to output.
    ///
    /// Trying to write out-of-range values will result in an [`Error::ValueOutOfBounds`].
    pub value: u16,
}

impl ChannelState {
    /// Creates a ChannelState with all bits set to 0: external voltage reference, powered on,
    /// x1 gain, and value of 0.
    pub fn new() -> ChannelState {
        ChannelState {
            voltage_reference_mode: VoltageReferenceMode::External,
            power_down_mode: PowerDownMode::Normal,
            gain_mode: GainMode::TimesOne,
            value: 0,
        }
    }

    /// Convenience builder method to set voltage reference mode.
    pub fn voltage_reference_mode(mut self, new_val: VoltageReferenceMode) -> ChannelState {
        self.voltage_reference_mode = new_val;
        self
    }

    /// Convenience builder method to set power down mode.
    pub fn power_down_mode(mut self, new_val: PowerDownMode) -> ChannelState {
        self.power_down_mode = new_val;
        self
    }

    /// Convenience builder method to set gain mode.
    pub fn gain_mode(mut self, new_val: GainMode) -> ChannelState {
        self.gain_mode = new_val;
        self
    }

    /// Convenience builder method to set value.
    pub fn value(mut self, new_val: u16) -> ChannelState {
        self.value = new_val;
        self
    }
}

impl Default for ChannelState {
    fn default() -> Self {
        Self::new()
    }
}

/// Trait to abstract away the I2C bus.
///
/// This allows for usage by devices that implement different embedded_hal traits.  Also all
/// functions return the crate [`Error`] type for convenience.
#[doc(hidden)]
pub trait I2CInterface {
    type I2C;
    type Error;

    /// Send a read command to the given address and read until the buffer is full.
    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error<Self::Error>>;

    /// Send a write command to the given address followed by all of the bytes.
    fn write_bytes(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error<Self::Error>>;

    /// Send a write command to the given address followed by all of the bytes.
    fn write_iter<B>(&mut self, address: u8, bytes: B) -> Result<(), Error<Self::Error>>
    where
        B: IntoIterator<Item = u8>;

    /// Destroy this instance and return the inner I2C bus.
    fn release(self) -> Self::I2C;
}

#[doc(hidden)]
pub struct I2CInterfaceDefault<I2C> {
    i2c: I2C,
}

impl<I2C, E> I2CInterfaceDefault<I2C>
where
    I2C: i2c::Read<Error = E> + i2c::Write<Error = E>,
{
    fn new(i2c: I2C) -> Self {
        Self { i2c }
    }
}

impl<I2C, E> I2CInterface for I2CInterfaceDefault<I2C>
where
    I2C: i2c::Read<Error = E> + i2c::Write<Error = E>,
{
    type I2C = I2C;
    type Error = E;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error<Self::Error>> {
        self.i2c.read(address, buffer).map_err(Error::I2CError)
    }

    fn write_bytes(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error<Self::Error>> {
        self.i2c.write(address, bytes).map_err(Error::I2CError)
    }

    fn write_iter<B>(&mut self, address: u8, bytes: B) -> Result<(), Error<Self::Error>>
    where
        B: IntoIterator<Item = u8>,
    {
        let mut buffer = [0; 12];
        let mut i = 0;
        for b in bytes {
            if i >= 12 {
                return Err(Error::BufferOverflow);
            }
            buffer[i] = b;
            i += 1;
        }
        self.i2c
            .write(address, &buffer[0..i])
            .map_err(Error::I2CError)
    }

    fn release(self) -> Self::I2C {
        self.i2c
    }
}

#[doc(hidden)]
pub struct I2CInterfaceIter<I2C> {
    i2c: I2C,
}

impl<I2C, E> I2CInterfaceIter<I2C>
where
    I2C: i2c::Read<Error = E> + i2c::WriteIter<Error = E>,
{
    fn new(i2c: I2C) -> Self {
        Self { i2c }
    }
}

impl<I2C, E> I2CInterface for I2CInterfaceIter<I2C>
where
    I2C: i2c::Read<Error = E> + i2c::WriteIter<Error = E>,
{
    type I2C = I2C;
    type Error = E;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Error<Self::Error>> {
        self.i2c.read(address, buffer).map_err(Error::I2CError)
    }

    fn write_bytes(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error<Self::Error>> {
        self.i2c
            .write(address, bytes.iter().copied())
            .map_err(Error::I2CError)
    }

    fn write_iter<B>(&mut self, address: u8, bytes: B) -> Result<(), Error<Self::Error>>
    where
        B: IntoIterator<Item = u8>,
    {
        self.i2c.write(address, bytes).map_err(Error::I2CError)
    }

    fn release(self) -> Self::I2C {
        self.i2c
    }
}

/// MCP4728 4-channel 12-bit I2C DAC.
pub struct MCP4728<B> {
    i2c: B,
    address: u8,
}

impl<I2C, E> MCP4728<I2CInterfaceDefault<I2C>>
where
    I2C: i2c::Read<Error = E> + i2c::Write<Error = E>,
{
    /// Creates a new [`MCP4728`] from an I2C device that implements the common
    /// [`embedded_hal::blocking::i2c::Read`] and [`embedded_hal::blocking::i2c::Write`] traits.
    ///
    /// Some methods (`multi_write` and `sequential_write`) write a variable number of bytes,
    /// which is not possible to do in the general case with these trait bounds and without
    /// allocation. To work around this we use a 12-byte buffer and return an
    /// [`Error::BufferOverflow`] if a write is larger than that.  In practice, there's not really
    /// a reason to write more than 4 values to this 4 channel DAC, and 12 bytes is sufficient for
    /// that.
    pub fn new(i2c: I2C, address: u8) -> Self {
        MCP4728 {
            i2c: I2CInterfaceDefault::new(i2c),
            address,
        }
    }
}

impl<I2C, E> MCP4728<I2CInterfaceIter<I2C>>
where
    I2C: i2c::Read<Error = E> + i2c::WriteIter<Error = E>,
{
    /// Creates a new [`MCP4728`] from an I2C device that implements the common
    /// [`embedded_hal::blocking::i2c::Read`] and less commonly implemented
    /// [`embedded_hal::blocking::i2c::WriteIter`] traits.
    pub fn from_i2c_iter(i2c: I2C, address: u8) -> Self {
        MCP4728 {
            i2c: I2CInterfaceIter::new(i2c),
            address,
        }
    }
}
/// Implementation of all commands given a generic I2CInterface.
///
/// # Errors
///
/// Any errors encountered within the I2C device will be wrapped in [`Error::I2CError`].
impl<B, I2C, E> MCP4728<B>
where
    B: I2CInterface<I2C = I2C, Error = E>,
{
    /// Destroy this instance and return the inner I2C bus.
    pub fn release(self) -> I2C {
        self.i2c.release()
    }

    /// Issues a general call command (address 0x00) to reset the device.  All MCP4728 devices
    /// on the bus will load the values from EEPROM into the output registers and update the
    /// output voltage.
    pub fn general_call_reset(&mut self) -> Result<(), Error<E>> {
        self.i2c
            .write_bytes(ADDRESS_GENERAL_CALL, &[COMMAND_GENERAL_CALL_RESET])
    }

    /// Issues a general call command (address 0x00) to wake up the device.  All MCP4728 devices
    /// on the bus will reset the power down bits and turn on all channels.
    pub fn general_call_wake_up(&mut self) -> Result<(), Error<E>> {
        self.i2c
            .write_bytes(ADDRESS_GENERAL_CALL, &[COMMAND_GENERAL_CALL_WAKE_UP])
    }

    /// Issues a general call command (address 0x00) to update software.  All MCP4728 devices
    /// on the bus will immediately update the output voltage.
    pub fn general_call_software_update(&mut self) -> Result<(), Error<E>> {
        self.i2c.write_bytes(
            ADDRESS_GENERAL_CALL,
            &[COMMAND_GENERAL_CALL_SOFTWARE_UPDATE],
        )
    }

    /// Updates the values of all four channels and sets them to be powered on (e.g.
    /// [`PowerDownMode::Normal`] is set).
    ///
    /// This command writes to the output registers directly and does not affect the EEPROM.  The
    /// voltage reference mode and gain mode are not affected. The actual voltage output will be
    /// updated as bytes are recieved if the LDAC pin is set low, or can be updated together by
    /// toggling LDAC from high to low after sending this command.  
    pub fn fast_write(
        &mut self,
        val_a: u16,
        val_b: u16,
        val_c: u16,
        val_d: u16,
    ) -> Result<(), Error<E>> {
        self.fast_write_with_power_down_mode(
            (PowerDownMode::Normal, val_a),
            (PowerDownMode::Normal, val_b),
            (PowerDownMode::Normal, val_c),
            (PowerDownMode::Normal, val_d),
        )
    }

    /// Updates the values of all four channels and sets their corresponding [`PowerDownMode`]s.
    ///
    /// This command writes to the output registers directly and does not affect the EEPROM.  The
    /// voltage reference mode and gain mode are not affected. The actual voltage output will be
    /// updated as bytes are recieved if the LDAC pin is set low, or can be updated together by
    /// toggling LDAC from high to low after sending this command.  
    pub fn fast_write_with_power_down_mode(
        &mut self,
        val_a: (PowerDownMode, u16),
        val_b: (PowerDownMode, u16),
        val_c: (PowerDownMode, u16),
        val_d: (PowerDownMode, u16),
    ) -> Result<(), Error<E>> {
        let mut bytes = [0; 8];
        for (i, &(mode, val)) in [val_a, val_b, val_c, val_d].iter().enumerate() {
            if val > 0x0fff {
                return Err(Error::ValueOutOfBounds(val));
            }
            bytes[2 * i] = (mode as u8) << 4 | val.to_be_bytes()[0];
            bytes[2 * i + 1] = val.to_be_bytes()[1];
        }
        self.i2c.write_bytes(self.address, &bytes)
    }

    pub fn single_write(
        &mut self,
        channel: Channel,
        output_enable_mode: OutputEnableMode,
        channel_state: &ChannelState,
    ) -> Result<(), Error<E>> {
        // || 0 1 0 1 1 CH CH OE || VR PD PD G D D D D || D D D D D D D D ||
        // CH = Channel select
        // OE = Output enable
        // VR = Voltage reference mode
        // PD = Power down mode
        // G = Gain mode

        if channel_state.value > 0x0fff {
            return Err(Error::ValueOutOfBounds(channel_state.value));
        }
        let mut bytes = [0; 3];
        bytes[0] = COMMAND_SINGLE_WRITE | (channel as u8) << 1 | output_enable_mode as u8;
        bytes[1] = (channel_state.voltage_reference_mode as u8) << 7
            | (channel_state.power_down_mode as u8) << 5
            | (channel_state.gain_mode as u8) << 4
            | channel_state.value.to_be_bytes()[0];
        bytes[2] = channel_state.value.to_be_bytes()[1];
        self.i2c.write_bytes(self.address, &bytes)
    }

    pub fn write_voltage_reference_mode(
        &mut self,
        mode_a: VoltageReferenceMode,
        mode_b: VoltageReferenceMode,
        mode_c: VoltageReferenceMode,
        mode_d: VoltageReferenceMode,
    ) -> Result<(), Error<E>> {
        let byte = COMMAND_WRITE_VOLTAGE_REFERENCE_MODE
            | (mode_a as u8) << 3
            | (mode_b as u8) << 2
            | (mode_c as u8) << 1
            | mode_d as u8;
        self.i2c.write_bytes(self.address, &[byte])
    }

    pub fn write_gain_mode(
        &mut self,
        mode_a: GainMode,
        mode_b: GainMode,
        mode_c: GainMode,
        mode_d: GainMode,
    ) -> Result<(), Error<E>> {
        let byte = COMMAND_WRITE_GAIN_MODE
            | (mode_a as u8) << 3
            | (mode_b as u8) << 2
            | (mode_c as u8) << 1
            | mode_d as u8;
        self.i2c.write_bytes(self.address, &[byte])
    }

    pub fn write_power_down_mode(
        &mut self,
        mode_a: PowerDownMode,
        mode_b: PowerDownMode,
        mode_c: PowerDownMode,
        mode_d: PowerDownMode,
    ) -> Result<(), Error<E>> {
        let mut bytes = [0; 2];
        bytes[0] = COMMAND_WRITE_POWER_DOWN_MODE | (mode_a as u8) << 2 | mode_b as u8;
        bytes[1] = (mode_c as u8) << 6 | (mode_d as u8) << 4;
        self.i2c.write_bytes(self.address, &bytes)
    }

    fn parse_bytes(bytes: &[u8]) -> ChannelRegisters {
        ChannelRegisters {
            channel_state: ChannelState {
                voltage_reference_mode: VoltageReferenceMode::try_from(
                    (bytes[1] & 0b10000000) >> 7,
                )
                .unwrap(),
                power_down_mode: PowerDownMode::try_from((bytes[1] & 0b01100000) >> 5).unwrap(),
                gain_mode: GainMode::try_from((bytes[1] & 0b00010000) >> 4).unwrap(),
                value: u16::from_be_bytes([bytes[1] & 0b00001111, bytes[2]]),
            },
            ready_state: ReadyState::try_from((bytes[0] & 0b10000000) >> 7).unwrap(),
            power_state: PowerState::try_from((bytes[0] & 0b01000000) >> 6).unwrap(),
        }
    }

    pub fn read(&mut self) -> Result<Registers, Error<E>> {
        let mut bytes = [0; 24];
        self.i2c.read(self.address, &mut bytes)?;
        Ok(Registers {
            channel_a_input: Self::parse_bytes(&bytes[0..3]),
            channel_a_eeprom: Self::parse_bytes(&bytes[3..6]),
            channel_b_input: Self::parse_bytes(&bytes[6..9]),
            channel_b_eeprom: Self::parse_bytes(&bytes[9..12]),
            channel_c_input: Self::parse_bytes(&bytes[12..15]),
            channel_c_eeprom: Self::parse_bytes(&bytes[15..18]),
            channel_d_input: Self::parse_bytes(&bytes[18..21]),
            channel_d_eeprom: Self::parse_bytes(&bytes[21..24]),
        })
    }

    pub fn multi_write(
        &mut self,
        channel_updates: &[(Channel, OutputEnableMode, ChannelState)],
    ) -> Result<(), Error<E>> {
        let mut channel_index = 0;
        let mut byte_index = 0;
        let generator = core::iter::from_fn(move || {
            if channel_index >= channel_updates.len() {
                return None;
            }
            let (channel, output_enable_mode, channel_state) =
                channel_updates.get(channel_index).unwrap();
            let byte = match byte_index {
                0 => COMMAND_MULTI_WRITE | (*channel as u8) << 1 | *output_enable_mode as u8,

                1 => {
                    (channel_state.voltage_reference_mode as u8) << 7
                        | (channel_state.power_down_mode as u8) << 5
                        | (channel_state.gain_mode as u8) << 4
                        | channel_state.value.to_be_bytes()[0]
                }

                2 => channel_state.value.to_be_bytes()[1],

                _ => panic!("Byte index > 2, this should not happen"),
            };
            byte_index = (byte_index + 1) % 3;
            if byte_index == 0 {
                channel_index += 1;
            }
            Some(byte)
        });

        self.i2c.write_iter(self.address, generator)
    }

    pub fn sequential_write(
        &mut self,
        starting_channel: Channel,
        output_enable_mode: OutputEnableMode,
        channel_updates: &[ChannelState],
    ) -> Result<(), Error<E>> {
        let expected_updates = 4 - starting_channel as usize;
        if channel_updates.len() != expected_updates {
            return Err(Error::StartingChannelNotEqualToUpdateLength);
        }
        let mut is_first_byte = true;
        let mut channel_index = 0;
        let mut byte_index = 0;
        let generator = core::iter::from_fn(move || {
            if channel_index >= channel_updates.len() {
                return None;
            }
            let channel_state = channel_updates.get(channel_index).unwrap();
            let byte;
            if is_first_byte {
                byte = COMMAND_SEQUENTIAL_WRITE
                    | (starting_channel as u8) << 1
                    | output_enable_mode as u8;
                is_first_byte = false;
            } else {
                byte = match byte_index {
                    0 => {
                        (channel_state.voltage_reference_mode as u8) << 7
                            | (channel_state.power_down_mode as u8) << 5
                            | (channel_state.gain_mode as u8) << 4
                            | channel_state.value.to_be_bytes()[0]
                    }

                    1 => channel_state.value.to_be_bytes()[1],

                    _ => panic!("Byte index > 1, this should not happen"),
                };
                byte_index = (byte_index + 1) % 2;
                if byte_index == 0 {
                    channel_index += 1;
                }
            }
            Some(byte)
        });

        self.i2c.write_iter(self.address, generator)
    }
}

#[cfg(test)]
mod tests {
    extern crate embedded_hal as hal;

    use hal::blocking::i2c;

    use core::iter::IntoIterator;
    use std::cell::RefCell;
    use std::rc::Rc;

    use crate::*;

    #[derive(Debug, PartialEq)]
    struct FakeI2CMessage {
        address: u8,
        bytes: Vec<u8>,
    }

    #[derive(Debug, PartialEq)]
    enum FakeI2CError {
        ReadError,
        WriteError,
    }

    struct FakeI2C {
        messages: Rc<RefCell<Vec<FakeI2CMessage>>>,
        message_to_read: Rc<RefCell<FakeI2CMessage>>,
        should_fail: Rc<RefCell<bool>>,
    }

    impl FakeI2C {
        fn new() -> FakeI2C {
            FakeI2C {
                messages: Rc::new(RefCell::new(vec![])),
                message_to_read: Rc::new(RefCell::new(FakeI2CMessage {
                    address: 0,
                    bytes: vec![],
                })),
                should_fail: Rc::new(RefCell::new(false)),
            }
        }
    }

    impl i2c::Read for FakeI2C {
        type Error = FakeI2CError;
        fn read(&mut self, address: u8, bytes: &mut [u8]) -> Result<(), FakeI2CError> {
            let message = self.message_to_read.borrow();
            if *self.should_fail.borrow() {
                Err(FakeI2CError::ReadError)
            } else if message.address != address {
                Err(FakeI2CError::ReadError)
            } else {
                bytes.copy_from_slice(&message.bytes);
                Ok(())
            }
        }
    }

    impl i2c::WriteIter for FakeI2C {
        type Error = FakeI2CError;
        fn write<B>(&mut self, address: u8, bytes: B) -> Result<(), FakeI2CError>
        where
            B: IntoIterator<Item = u8>,
        {
            if !*self.should_fail.borrow() {
                self.messages.borrow_mut().push(FakeI2CMessage {
                    address,
                    bytes: Vec::from_iter(bytes),
                });
                Ok(())
            } else {
                Err(FakeI2CError::WriteError)
            }
        }
    }

    impl i2c::Write for FakeI2C {
        type Error = FakeI2CError;
        fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), FakeI2CError> {
            i2c::WriteIter::write(self, address, bytes.iter().copied())
        }
    }

    #[test]
    fn fast_write() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(mcp4728.fast_write(0x0aaa, 0x0000, 0x0aaa, 0x0000), Ok(()));
        assert_eq!(
            *messages.borrow(),
            vec![FakeI2CMessage {
                address: 0x60,
                bytes: vec![0x0a, 0xaa, 0x00, 0x00, 0x0a, 0xaa, 0x00, 0x00]
            }]
        );
    }

    #[test]
    fn fast_write_i2c_error() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        *i2c.should_fail.borrow_mut() = true;
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.fast_write(0x0aaa, 0x0000, 0x0aaa, 0x0000),
            Err(Error::I2CError(FakeI2CError::WriteError))
        );
        assert_eq!(*messages.borrow(), vec![]);
    }

    #[test]
    fn fast_write_out_of_bounds_error() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.fast_write(0x1000, 0x0000, 0x0000, 0x0000),
            Err(Error::ValueOutOfBounds(0x1000))
        );
        assert_eq!(*messages.borrow(), vec![]);
    }

    #[test]
    fn fast_write_with_power_down_mode() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.fast_write_with_power_down_mode(
                (PowerDownMode::Normal, 0x0aaa),
                (PowerDownMode::PowerDownOneK, 0x0000),
                (PowerDownMode::PowerDownOneHundredK, 0x0aaa),
                (PowerDownMode::PowerDownFiveHundredK, 0x0000),
            ),
            Ok(())
        );
        assert_eq!(
            *messages.borrow(),
            vec![FakeI2CMessage {
                address: 0x60,
                bytes: vec![0x0a, 0xaa, 0x10, 0x00, 0x2a, 0xaa, 0x30, 0x00]
            }]
        );
    }

    // || 0 1 0 1 1 CH CH OE || VR PD PD G D D D D || D D D D D D D D ||
    #[test]
    fn single_write_default_values() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.single_write(
                Channel::B,
                OutputEnableMode::Update,
                &ChannelState::new().value(0x0aaa)
            ),
            Ok(())
        );
        assert_eq!(
            *messages.borrow(),
            vec![FakeI2CMessage {
                address: 0x60,
                bytes: vec![0b01011010, 0b00001010, 0b10101010]
            }]
        );
    }

    #[test]
    fn single_write_set_all_values() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.single_write(
                Channel::D,
                OutputEnableMode::NoUpdate,
                &ChannelState::new()
                    .voltage_reference_mode(VoltageReferenceMode::Internal)
                    .power_down_mode(PowerDownMode::PowerDownFiveHundredK)
                    .gain_mode(GainMode::TimesTwo)
                    .value(0x0fff)
            ),
            Ok(())
        );
        assert_eq!(
            *messages.borrow(),
            vec![FakeI2CMessage {
                address: 0x60,
                bytes: vec![0b01011111, 0b11111111, 0b11111111]
            }]
        );
    }

    #[test]
    fn single_write_out_of_bounds_error() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.single_write(
                Channel::B,
                OutputEnableMode::NoUpdate,
                &ChannelState::new().value(0xffff)
            ),
            Err(Error::ValueOutOfBounds(0xffff))
        );
        assert_eq!(*messages.borrow(), vec![]);
    }

    #[test]
    fn multi_write_set_all_values() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.multi_write(&[(
                Channel::D,
                OutputEnableMode::NoUpdate,
                ChannelState::new()
                    .voltage_reference_mode(VoltageReferenceMode::Internal)
                    .power_down_mode(PowerDownMode::PowerDownFiveHundredK)
                    .gain_mode(GainMode::TimesTwo)
                    .value(0x0fff)
            )]),
            Ok(())
        );
        assert_eq!(
            *messages.borrow(),
            vec![FakeI2CMessage {
                address: 0x60,
                bytes: vec![0b01000111, 0b11111111, 0b11111111]
            }]
        );
    }

    #[test]
    fn multi_write_multiple_values() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.multi_write(&[
                (
                    Channel::A,
                    OutputEnableMode::Update,
                    ChannelState::new().value(0x0001)
                ),
                (
                    Channel::B,
                    OutputEnableMode::Update,
                    ChannelState::new().value(0x0002)
                )
            ]),
            Ok(())
        );
        assert_eq!(
            *messages.borrow(),
            vec![FakeI2CMessage {
                address: 0x60,
                bytes: vec![0b01000000, 0b00000000, 0b00000001, 0b01000010, 0b00000000, 0b00000010]
            }]
        );
    }

    #[test]
    fn sequential_write_multiple_values() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.sequential_write(
                Channel::A,
                OutputEnableMode::Update,
                &[
                    ChannelState::new().value(0x0001),
                    ChannelState::new().value(0x0002),
                    ChannelState::new().value(0x0003),
                    ChannelState::new().value(0x0004),
                ]
            ),
            Ok(())
        );
        assert_eq!(
            *messages.borrow(),
            vec![FakeI2CMessage {
                address: 0x60,
                bytes: vec![
                    0b01010000, 0b00000000, 0b00000001, 0b00000000, 0b00000010, 0b00000000,
                    0b00000011, 0b00000000, 0b00000100
                ]
            }]
        );
    }

    #[test]
    fn sequential_write_too_many_values() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.sequential_write(
                Channel::B,
                OutputEnableMode::Update,
                &[
                    ChannelState::new().value(0x0001),
                    ChannelState::new().value(0x0002),
                    ChannelState::new().value(0x0003),
                    ChannelState::new().value(0x0004),
                ]
            ),
            Err(Error::StartingChannelNotEqualToUpdateLength)
        );
        assert_eq!(*messages.borrow(), vec![]);
    }

    #[test]
    fn write_voltage_reference_mode() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.write_voltage_reference_mode(
                VoltageReferenceMode::External,
                VoltageReferenceMode::Internal,
                VoltageReferenceMode::External,
                VoltageReferenceMode::Internal,
            ),
            Ok(())
        );
        assert_eq!(
            *messages.borrow(),
            vec![FakeI2CMessage {
                address: 0x60,
                bytes: vec![0b10000101]
            }]
        );
    }

    #[test]
    fn write_gain_mode() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.write_gain_mode(
                GainMode::TimesOne,
                GainMode::TimesTwo,
                GainMode::TimesOne,
                GainMode::TimesTwo,
            ),
            Ok(())
        );
        assert_eq!(
            *messages.borrow(),
            vec![FakeI2CMessage {
                address: 0x60,
                bytes: vec![0b11000101]
            }]
        );
    }

    #[test]
    fn write_power_down_mode() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.write_power_down_mode(
                PowerDownMode::Normal,
                PowerDownMode::PowerDownOneK,
                PowerDownMode::PowerDownOneHundredK,
                PowerDownMode::PowerDownFiveHundredK,
            ),
            Ok(())
        );
        assert_eq!(
            *messages.borrow(),
            vec![FakeI2CMessage {
                address: 0x60,
                bytes: vec![0b10100001, 0b10110000]
            }]
        );
    }

    #[test]
    fn read() {
        let i2c = FakeI2C::new();
        #[rustfmt::skip]
            let bytes = vec![
                0b00000000, 0b00000000, 0b00000000,
                0b11000000, 0b11111111, 0b11111111,
                0b01000000, 0b01010101, 0b01010101,
                0b00000000, 0b00000000, 0b00000000,
                0b00000000, 0b00000000, 0b00000000,
                0b00000000, 0b00000000, 0b00000000,
                0b00000000, 0b00000000, 0b00000000,
                0b00000000, 0b00000000, 0b00000000,
            ];
        *i2c.message_to_read.borrow_mut() = FakeI2CMessage {
            address: 0x60,
            bytes: bytes,
        };
        let mut mcp4728 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4728.read(),
            Ok(Registers {
                channel_a_input: ChannelRegisters {
                    channel_state: ChannelState::new(),
                    ready_state: ReadyState::Ready,
                    power_state: PowerState::Off
                },
                channel_a_eeprom: ChannelRegisters {
                    channel_state: ChannelState::new()
                        .voltage_reference_mode(VoltageReferenceMode::Internal)
                        .power_down_mode(PowerDownMode::PowerDownFiveHundredK)
                        .gain_mode(GainMode::TimesTwo)
                        .value(0x0fff),
                    ready_state: ReadyState::Busy,
                    power_state: PowerState::On
                },
                channel_b_input: ChannelRegisters {
                    channel_state: ChannelState::new()
                        .power_down_mode(PowerDownMode::PowerDownOneHundredK)
                        .gain_mode(GainMode::TimesTwo)
                        .value(0x0555),
                    ready_state: ReadyState::Ready,
                    power_state: PowerState::On
                },
                channel_b_eeprom: ChannelRegisters {
                    channel_state: ChannelState::new(),
                    ready_state: ReadyState::Ready,
                    power_state: PowerState::Off
                },
                channel_c_input: ChannelRegisters {
                    channel_state: ChannelState::new(),
                    ready_state: ReadyState::Ready,
                    power_state: PowerState::Off
                },
                channel_c_eeprom: ChannelRegisters {
                    channel_state: ChannelState::new(),
                    ready_state: ReadyState::Ready,
                    power_state: PowerState::Off
                },
                channel_d_input: ChannelRegisters {
                    channel_state: ChannelState::new(),
                    ready_state: ReadyState::Ready,
                    power_state: PowerState::Off
                },
                channel_d_eeprom: ChannelRegisters {
                    channel_state: ChannelState::new(),
                    ready_state: ReadyState::Ready,
                    power_state: PowerState::Off
                },
            })
        )
    }
}
