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

mod types;

pub use crate::types::*;

use embedded_hal::blocking::i2c;

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

/// MCP4728 4-channel 12-bit I2C DAC.
pub struct MCP4728<I2C> {
    i2c: I2C,
    address: u8,
}
/// Implementation of all commands given a generic I2CInterface.
///
/// # Errors
///
/// Any errors encountered within the I2C device will be wrapped in [`Error::I2CError`].
impl<I2C, E> MCP4728<I2C>
where
    I2C: i2c::Read<Error = E> + i2c::Write<Error = E>,
{
    /// Creates a new [`MCP4728`] from an I2C device that implements the
    /// [`embedded_hal::blocking::i2c::Read`] and [`embedded_hal::blocking::i2c::Write`] traits.
    pub fn new(i2c: I2C, address: u8) -> Self {
        MCP4728 { i2c, address }
    }

    fn write_bytes(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error<E>> {
        self.i2c.write(address, bytes).map_err(Error::I2CError)
    }

    fn write_iter<B>(&mut self, address: u8, bytes: B) -> Result<(), Error<E>>
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

    /// Destroy this instance and return the inner I2C bus.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Reads all registers of all channels from the device.
    ///
    /// Each channel includes both the values in EEPROM and in the input registers to the DAC, which
    /// might differ if e.g. fast_write has been used.
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

    /// Issues a general call command (address 0x00) to reset the device.  All MCP4728 devices on
    /// the bus will load the values from EEPROM into the output registers and update the output
    /// voltage.
    pub fn general_call_reset(&mut self) -> Result<(), Error<E>> {
        self.write_bytes(ADDRESS_GENERAL_CALL, &[COMMAND_GENERAL_CALL_RESET])
    }

    /// Issues a general call command (address 0x00) to wake up the device.  All MCP4728 devices on
    /// the bus will reset the power down bits and turn on all channels.
    pub fn general_call_wake_up(&mut self) -> Result<(), Error<E>> {
        self.write_bytes(ADDRESS_GENERAL_CALL, &[COMMAND_GENERAL_CALL_WAKE_UP])
    }

    /// Issues a general call command (address 0x00) to update software.  All MCP4728 devices on the
    /// bus will immediately update the output voltage.
    pub fn general_call_software_update(&mut self) -> Result<(), Error<E>> {
        self.write_bytes(
            ADDRESS_GENERAL_CALL,
            &[COMMAND_GENERAL_CALL_SOFTWARE_UPDATE],
        )
    }

    /// Updates the values of all four channels and sets them to be powered on (i.e.
    /// [`PowerDownMode::Normal`] is set).
    ///
    /// This command writes to the output registers directly and does not affect the EEPROM.  The
    /// voltage reference mode and gain mode are not affected.
    ///
    /// # Updating the analog outputs
    ///
    /// This command writes to the input register but does not necessarily update the analog output.
    /// There are several ways to do so:
    ///
    ///   - If the LDAC pin is set low, the output will be updated after the last byte is received
    ///     for each channel.
    ///   - If the LDAC pin transitions from high to low at any time, all channels will be updated.
    ///   - If a General Call Software Update command is received, all channels will be updated.
    ///
    /// # Errors
    ///
    /// In addition to the internal I2C errors, this can return [`Error::ValueOutOfBounds`] if the
    /// value is out of range (greater than 4095).
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
    /// voltage reference mode and gain mode are not affected.
    ///
    /// # Updating the analog outputs
    ///
    /// This command writes to the input register but does not necessarily update the analog output.
    /// There are several ways to do so:
    ///
    ///   - If the LDAC pin is set low, the output will be updated after the last byte is received
    ///     for each channel.
    ///   - If the LDAC pin transitions from high to low at any time, all channels will be updated.
    ///   - If a General Call Software Update command is received, all channels will be updated.
    ///
    /// # Errors
    ///
    /// In addition to the internal I2C errors, this can return [`Error::ValueOutOfBounds`] if the
    /// value is out of range (greater than 4095).
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
        self.write_bytes(self.address, &bytes)
    }

    /// Updates all bits of a single channel to both the DAC input register and EEPROM.
    ///
    /// This sets the voltage reference mode, power down mode, gain mode, and value of the channel.
    ///
    /// # Updating the analog outputs
    ///
    /// This command writes to the input register but does not necessarily update the analog output.
    /// There are several ways to do so:
    ///
    ///   - If `OutputEnableMode` is `Update`, the output will be updated after the last byte is
    ///     received.
    ///   - If the LDAC pin is set low, the output will be updated after the last byte is received.
    ///   - If the LDAC pin transitions from high to low at any time, all channels will be updated.
    ///   - If a General Call Software Update command is received, all channels will be updated.
    ///
    /// # Errors
    ///
    /// In addition to the internal I2C errors, this can return [`Error::ValueOutOfBounds`] if the
    /// value is out of range (greater than 4095).
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
        self.write_bytes(self.address, &bytes)
    }

    /// Updates all bits of 1-4 sequential channels to both the DAC input registers and EEPROM.
    ///
    /// This sets the voltage reference mode, power down mode, gain mode, and value of the channels.
    /// The channels including and after `starting_channel` will be updated (e.g. if
    /// `starting_channel` is channel b, then channels b, c and d will be updated). The number of
    /// entries in `channel_updates` must equal the number of channels that will be updated.
    ///
    /// # Updating the analog outputs
    ///
    /// This command writes to the input register but does not necessarily update the analog output.
    /// There are several ways to do so:
    ///
    ///   - If `OutputEnableMode` is `Update`, the output will be updated after the last byte is
    ///     received for each channel.
    ///   - If the LDAC pin is set low, the output will be updated after the last byte is received
    ///     for each channel.
    ///   - If the LDAC pin transitions from high to low at any time, all channels will be updated.
    ///   - If a General Call Software Update command is received, all channels will be updated.
    ///
    /// # Errors
    ///
    /// In addition to the internal I2C errors, this can return [`Error::ValueOutOfBounds`] if any
    /// value is out of range (greater than 4095) and
    /// [`Error::StartingChannelNotEqualToUpdateLength`] if there is a mismatch between the
    /// starting channel and the number of updates.
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

        self.write_iter(self.address, generator)
    }

    /// Updates all bits of multiple channels to both the DAC input registers and EEPROM.
    ///
    /// This sets the voltage reference mode, power down mode, gain mode, and value of the channels.
    ///
    /// # Updating the analog outputs
    ///
    /// This command writes to the input register but does not necessarily update the analog output.
    /// There are several ways to do so:
    ///
    ///   - If `OutputEnableMode` is `Update`, the output will be updated after the last byte is
    ///     received for each channel.
    ///   - If the LDAC pin is set low, the output will be updated after the last byte is received
    ///     for each channel.
    ///   - If the LDAC pin transitions from high to low at any time, all channels will be updated.
    ///   - If a General Call Software Update command is received, all channels will be updated.
    ///
    /// # Errors
    ///
    /// In addition to the internal I2C errors, this can return [`Error::ValueOutOfBounds`] if any
    /// value is out of range (greater than 4095).
    ///
    /// This function can write an arbitrary number of updates, so it is impossible to statically
    /// allocate a buffer to write using the [`embedded_hal::blocking::i2c::Write`] trait.  There is
    /// the [`embedded_hal::blocking::i2c::WriteIter`] trait, but it is much less commonly
    /// implemented.  To work around this, we will use a buffer large enough to contain four writes
    /// at a time and return [`Error::BufferOverflow`] if more writes are requested.  This is
    /// unlikely to be a limitation given that there are four channels.
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

        self.write_iter(self.address, generator)
    }

    /// Writes only the voltage reference mode bits for all channels.
    ///
    /// The EEPROM data is not affected and the output of each channel is updated after the command
    /// has been received.
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
        self.write_bytes(self.address, &[byte])
    }

    /// Writes only the gain mode bits for all channels.
    ///
    /// The EEPROM data is not affected and the output of each channel is updated after the command
    /// has been received.
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
        self.write_bytes(self.address, &[byte])
    }

    /// Writes only the power down mode bits for all channels.
    ///
    /// The EEPROM data is not affected and the output of each channel is updated after the command
    /// has been received.
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
        self.write_bytes(self.address, &bytes)
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
}

#[cfg(test)]
mod tests {
    mod fake_i2c;

    use std::rc::Rc;

    use crate::tests::fake_i2c::*;
    use crate::*;

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
