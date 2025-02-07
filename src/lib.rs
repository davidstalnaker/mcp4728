//! # Rust driver for MCP4728 4-channel 12-bit I2C DAC
//!
//! This is a platform agnostic rust driver for the MCP4728 DAC using the
//! [embedded-hal](https://github.com/rust-embedded/embedded-hal) traits.
//!
//! This driver allows you to:
//!
//! - Write to a single channel and save to EEPROM
//! - Write to multiple channels and save to EEPROM
//! - Write to all channels, skipping some configuration bits and not saving to EEPROM ("fast
//!   write")
//! - Write voltage reference mode to all channels
//! - Write gain mode to all channels
//! - Write power down mode to all channels
//! - Read EEPROM and output registers for all channels
//! - Issue I2C General Call commands: reset, wake up, and software update
//!
//! There is one feature that this driver does **not** support - reading and writing the I2C address
//! of the device. Three bits of the driver can be set (from 0x60 to 0x67) but this is done by
//! toggling another pin in precise timing with the I2C message, which is not possible to do using
//! the [embedded-hal](https://github.com/rust-embedded/embedded-hal) traits.
//!
//! This crate is `#![no_std]` and does not contain unsafe code.
//!
//! ## Device
//!
//! The MCP4728 is a 4-channel, 12-bit digital-to-analog converter with nonvolatile memory (EEPROM)
//! for its output settings. The device will load its previous setting from EEPROM on startup
//! without receiving any commands. Each channel has separate voltage reference, gain, and power
//! down settings. Channels can be written individually or together.
//!
//! ### Datasheet
//!
//! [MCP4728](https://ww1.microchip.com/downloads/en/DeviceDoc/22187E.pdf)
//!
//! ## Features
//!
//! - `sync`: Adds support for the `embedded-hal`
//!   [`I2c` trait](https://docs.rs/embedded-hal/latest/embedded_hal/i2c/index.html)
//!   (enabled by default)
//! - `async`: Adds support for the `embedded-hal-async`
//!   [`I2c` trait](https://docs.rs/embedded-hal-async/1.0.0/embedded_hal_async/i2c/trait.I2c.html)
//! - `defmt`: Adds implementation of the `defmt`
//!   [`Format` trait](https://docs.rs/defmt/latest/defmt/trait.Format.html) to all types
//!
//! ## Usage
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
//!
//! ### Updating the analog outputs
//!
//! Most commands write to the output register but do not necessarily update the analog output.
//! There are several ways to do so:
//!
//! - If `OutputEnableMode` is `Update`, the output will be updated after the last byte is received
//!   for each channel (not all commands can set this bit)
//! - If the LDAC pin is set low, the output will be updated after the last byte is received for
//!   each channel.
//! - If the LDAC pin transitions from high to low at any time, all channels will be updated.
//! - If a General Call Software Update command is received, all channels will be updated.
//!
//! ### Async
//!
//! This crate optionally supports async, behind the `async` feature. The API is otherwise identical.
//!
//! ```no_build
//! use mcp4728::{MCP4728Async};
//!
//! // Assume i2c implements embedded_hal_async::i2c::I2c.
//! let mut dac = MCP4728Async::new(i2c, 0x60);
//! dac.fast_write(483, 279, 297, 590).await.unwrap();
//! ```

#![cfg_attr(not(test), no_std)]

mod internal_types;
mod types;

use crate::internal_types::*;
pub use crate::types::*;

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
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[maybe_async_cfg::maybe(
    idents(MCP4728(sync, async = "MCP4728Async"),),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct MCP4728<I> {
    i2c: I,
    address: u8,
}
/// Implementation of all commands given a generic I2CInterface.
///
/// # Errors
///
/// Any errors encountered within the I2C device will be wrapped in [`Error::I2CError`].
#[maybe_async_cfg::maybe(
    idents(
        MCP4728(sync, async = "MCP4728Async"),
        embedded_hal(sync, async = "embedded_hal_async"),
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<I, E> MCP4728<I>
where
    I: embedded_hal::i2c::I2c<Error = E>,
{
    /// Creates a new [`MCP4728`] from an I2C device that implements the [`embedded_hal::i2c::I2c`]
    /// trait.
    pub fn new(i2c: I, address: u8) -> Self {
        MCP4728 { i2c, address }
    }

    async fn write_bytes(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error<E>> {
        self.i2c
            .write(address, bytes)
            .await
            .map_err(Error::I2CError)
    }
    async fn write_iter<B>(&mut self, address: u8, bytes: B) -> Result<(), Error<E>>
    where
        B: IntoIterator<Item = u8>,
    {
        let mut buffer = [0; 12];
        let mut i = 0;
        for b in bytes {
            if i >= 12 {
                return Err(Error::WriteSizeExceeded);
            }
            buffer[i] = b;
            i += 1;
        }
        self.i2c
            .write(address, &buffer[0..i])
            .await
            .map_err(Error::I2CError)
    }

    /// Destroy this instance and return the inner I2C bus.
    pub fn release(self) -> I {
        self.i2c
    }
    /// Reads all registers of all channels from the device.
    ///
    /// Each channel includes both the values in EEPROM and in the input registers to the DAC, which
    /// might differ if e.g. fast_write has been used.
    pub async fn read(&mut self) -> Result<Registers, Error<E>> {
        let mut bytes = [0; 24];
        self.i2c.read(self.address, &mut bytes).await?;
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
    pub async fn general_call_reset(&mut self) -> Result<(), Error<E>> {
        self.write_bytes(ADDRESS_GENERAL_CALL, &[COMMAND_GENERAL_CALL_RESET])
            .await
    }

    /// Issues a general call command (address 0x00) to wake up the device.  All MCP4728 devices on
    /// the bus will reset the power down bits and turn on all channels.
    pub async fn general_call_wake_up(&mut self) -> Result<(), Error<E>> {
        self.write_bytes(ADDRESS_GENERAL_CALL, &[COMMAND_GENERAL_CALL_WAKE_UP])
            .await
    }

    /// Issues a general call command (address 0x00) to update software.  All MCP4728 devices on the
    /// bus will immediately update the output voltage.
    pub async fn general_call_software_update(&mut self) -> Result<(), Error<E>> {
        self.write_bytes(
            ADDRESS_GENERAL_CALL,
            &[COMMAND_GENERAL_CALL_SOFTWARE_UPDATE],
        )
        .await
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
    /// # Example
    ///
    /// ```no_run
    /// # #[cfg(target_os = "linux")] {
    /// use linux_embedded_hal::I2cdev;
    /// use mcp4728::{MCP4728};
    ///
    /// let i2c = I2cdev::new("/dev/i2c-1").unwrap();
    /// let mut dac = MCP4728::new(i2c, 0x60);
    /// dac.fast_write(483, 279, 297, 590).unwrap();
    /// # }
    /// ```
    ///
    /// # Errors
    ///
    /// In addition to the internal I2C errors, this can return [`Error::ValueOutOfBounds`] if the
    /// value is out of range (greater than 4095).
    pub async fn fast_write(
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
        .await
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
    /// # Example
    ///
    /// ```no_run
    /// # #[cfg(target_os = "linux")] {
    /// use linux_embedded_hal::I2cdev;
    /// use mcp4728::{MCP4728, PowerDownMode};
    ///
    /// let i2c = I2cdev::new("/dev/i2c-1").unwrap();
    /// let mut dac = MCP4728::new(i2c, 0x60);
    /// dac.fast_write_with_power_down_mode(
    ///     (PowerDownMode::Normal, 100),
    ///     (PowerDownMode::Normal, 200),
    ///     (PowerDownMode::PowerDownOneK, 0),
    ///     (PowerDownMode::PowerDownOneK, 0),
    /// ).unwrap();
    /// # }
    /// ```
    ///
    /// # Errors
    ///
    /// In addition to the internal I2C errors, this can return [`Error::ValueOutOfBounds`] if the
    /// value is out of range (greater than 4095).
    pub async fn fast_write_with_power_down_mode(
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
        self.write_bytes(self.address, &bytes).await
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
    pub async fn single_write(
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
        self.write_bytes(self.address, &bytes).await
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
    /// [`Error::StartingChannelMismatch`] if there is a mismatch between the starting channel and
    /// the number of updates.
    pub async fn sequential_write(
        &mut self,
        starting_channel: Channel,
        output_enable_mode: OutputEnableMode,
        channel_updates: &[ChannelState],
    ) -> Result<(), Error<E>> {
        let expected_updates = 4 - starting_channel as usize;
        if channel_updates.len() != expected_updates {
            return Err(Error::StartingChannelMismatch);
        }
        let mut is_first_byte = true;
        let mut channel_index = 0;
        let mut byte_index = SequentialWriteByteIndex::Zero;
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
                    SequentialWriteByteIndex::Zero => {
                        (channel_state.voltage_reference_mode as u8) << 7
                            | (channel_state.power_down_mode as u8) << 5
                            | (channel_state.gain_mode as u8) << 4
                            | channel_state.value.to_be_bytes()[0]
                    }

                    SequentialWriteByteIndex::One => channel_state.value.to_be_bytes()[1],
                };
                byte_index = byte_index.next();
                if byte_index == SequentialWriteByteIndex::Zero {
                    channel_index += 1;
                }
            }
            Some(byte)
        });

        self.write_iter(self.address, generator).await
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
    /// allocate a buffer to write using the [`embedded_hal::i2c::I2c`] trait.  To work around this,
    /// we will use a buffer large enough to contain four writes at a time and return
    /// [`Error::WriteSizeExceeded`] if more writes are requested.  This is unlikely to be a
    /// limitation given that there are four channels.
    pub async fn multi_write(
        &mut self,
        channel_updates: &[(Channel, OutputEnableMode, ChannelState)],
    ) -> Result<(), Error<E>> {
        let mut channel_index = 0;
        let mut byte_index = MultiWriteByteIndex::Zero;
        let generator = core::iter::from_fn(move || {
            if channel_index >= channel_updates.len() {
                return None;
            }
            let (channel, output_enable_mode, channel_state) =
                channel_updates.get(channel_index).unwrap();
            let byte = match byte_index {
                MultiWriteByteIndex::Zero => {
                    COMMAND_MULTI_WRITE | (*channel as u8) << 1 | *output_enable_mode as u8
                }

                MultiWriteByteIndex::One => {
                    (channel_state.voltage_reference_mode as u8) << 7
                        | (channel_state.power_down_mode as u8) << 5
                        | (channel_state.gain_mode as u8) << 4
                        | channel_state.value.to_be_bytes()[0]
                }

                MultiWriteByteIndex::Two => channel_state.value.to_be_bytes()[1],
            };
            byte_index = byte_index.next();
            if byte_index == MultiWriteByteIndex::Zero {
                channel_index += 1;
            }
            Some(byte)
        });

        self.write_iter(self.address, generator).await
    }

    /// Writes only the voltage reference mode bits for all channels.
    ///
    /// The EEPROM data is not affected and the output of each channel is updated after the command
    /// has been received.
    pub async fn write_voltage_reference_mode(
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
        self.write_bytes(self.address, &[byte]).await
    }

    /// Writes only the gain mode bits for all channels.
    ///
    /// The EEPROM data is not affected and the output of each channel is updated after the command
    /// has been received.
    pub async fn write_gain_mode(
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
        self.write_bytes(self.address, &[byte]).await
    }

    /// Writes only the power down mode bits for all channels.
    ///
    /// The EEPROM data is not affected and the output of each channel is updated after the command
    /// has been received.
    pub async fn write_power_down_mode(
        &mut self,
        mode_a: PowerDownMode,
        mode_b: PowerDownMode,
        mode_c: PowerDownMode,
        mode_d: PowerDownMode,
    ) -> Result<(), Error<E>> {
        let mut bytes = [0; 2];
        bytes[0] = COMMAND_WRITE_POWER_DOWN_MODE | (mode_a as u8) << 2 | mode_b as u8;
        bytes[1] = (mode_c as u8) << 6 | (mode_d as u8) << 4;
        self.write_bytes(self.address, &bytes).await
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
#[maybe_async_cfg::maybe(
    idents(MCP4728(sync, async = "MCP4728Async"),),
    sync(feature = "sync"),
    async(feature = "async")
)]
mod tests {
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock as MockI2C, Transaction};
    #[maybe_async_cfg::only_if(sync)]
    use test as maybe_async_test;
    #[maybe_async_cfg::only_if(async)]
    use tokio::test as maybe_async_test;

    use crate::*;

    #[maybe_async_test]
    async fn fast_write() {
        let expectations = [Transaction::write(
            0x60,
            vec![0x0a, 0xaa, 0x00, 0x00, 0x0a, 0xaa, 0x00, 0x00],
        )];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728.fast_write(0x0aaa, 0x0000, 0x0aaa, 0x0000).await,
            Ok(())
        );
        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }
    #[maybe_async_test]
    async fn fast_write_i2c_error() {
        let expectations =
            [
                Transaction::write(0x60, vec![0x0a, 0xaa, 0x00, 0x00, 0x0a, 0xaa, 0x00, 0x00])
                    .with_error(ErrorKind::Other),
            ];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728.fast_write(0x0aaa, 0x0000, 0x0aaa, 0x0000).await,
            Err(Error::I2CError(ErrorKind::Other))
        );
        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    #[maybe_async_test]
    async fn fast_write_out_of_bounds_error() {
        let expectations = [];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728.fast_write(0x1000, 0x0000, 0x0000, 0x0000).await,
            Err(Error::ValueOutOfBounds(0x1000))
        );
        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    #[maybe_async_test]
    async fn fast_write_with_power_down_mode() {
        let expectations = [Transaction::write(
            0x60,
            vec![0x0a, 0xaa, 0x10, 0x00, 0x2a, 0xaa, 0x30, 0x00],
        )];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728
                .fast_write_with_power_down_mode(
                    (PowerDownMode::Normal, 0x0aaa),
                    (PowerDownMode::PowerDownOneK, 0x0000),
                    (PowerDownMode::PowerDownOneHundredK, 0x0aaa),
                    (PowerDownMode::PowerDownFiveHundredK, 0x0000),
                )
                .await,
            Ok(())
        );
        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    // || 0 1 0 1 1 CH CH OE || VR PD PD G D D D D || D D D D D D D D ||
    #[maybe_async_test]
    async fn single_write_default_values() {
        let expectations = [Transaction::write(
            0x60,
            vec![0b01011010, 0b10001010, 0b10101010],
        )];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728
                .single_write(
                    Channel::B,
                    OutputEnableMode::Update,
                    &ChannelState::new().value(0x0aaa)
                )
                .await,
            Ok(())
        );
        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    #[maybe_async_test]
    async fn single_write_set_all_values() {
        let expectations = [Transaction::write(
            0x60,
            vec![0b01011111, 0b11111111, 0b11111111],
        )];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728
                .single_write(
                    Channel::D,
                    OutputEnableMode::NoUpdate,
                    &ChannelState::new()
                        .voltage_reference_mode(VoltageReferenceMode::Internal)
                        .power_down_mode(PowerDownMode::PowerDownFiveHundredK)
                        .gain_mode(GainMode::TimesTwo)
                        .value(0x0fff)
                )
                .await,
            Ok(())
        );
        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    #[maybe_async_test]
    async fn single_write_out_of_bounds_error() {
        let expectations = [];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728
                .single_write(
                    Channel::B,
                    OutputEnableMode::NoUpdate,
                    &ChannelState::new().value(0xffff)
                )
                .await,
            Err(Error::ValueOutOfBounds(0xffff))
        );
        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    #[maybe_async_test]
    async fn multi_write_set_all_values() {
        let expectations = [Transaction::write(
            0x60,
            vec![0b01000111, 0b11111111, 0b11111111],
        )];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728
                .multi_write(&[(
                    Channel::D,
                    OutputEnableMode::NoUpdate,
                    ChannelState::new()
                        .voltage_reference_mode(VoltageReferenceMode::Internal)
                        .power_down_mode(PowerDownMode::PowerDownFiveHundredK)
                        .gain_mode(GainMode::TimesTwo)
                        .value(0x0fff)
                )])
                .await,
            Ok(())
        );

        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    #[maybe_async_test]
    async fn multi_write_multiple_values() {
        let expectations = [Transaction::write(
            0x60,
            vec![
                0b01000000, 0b00000000, 0b00000001, 0b01000010, 0b00000000, 0b00000010,
            ],
        )];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728
                .multi_write(&[
                    (
                        Channel::A,
                        OutputEnableMode::Update,
                        ChannelState::new()
                            .voltage_reference_mode(VoltageReferenceMode::External)
                            .value(0x0001)
                    ),
                    (
                        Channel::B,
                        OutputEnableMode::Update,
                        ChannelState::new()
                            .voltage_reference_mode(VoltageReferenceMode::External)
                            .value(0x0002)
                    )
                ])
                .await,
            Ok(())
        );

        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    #[maybe_async_test]
    async fn sequential_write_multiple_values() {
        let expectations = [Transaction::write(
            0x60,
            vec![
                0b01010000, 0b10000000, 0b00000001, 0b10000000, 0b00000010, 0b10000000, 0b00000011,
                0b10000000, 0b00000100,
            ],
        )];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728
                .sequential_write(
                    Channel::A,
                    OutputEnableMode::Update,
                    &[
                        ChannelState::new().value(0x0001),
                        ChannelState::new().value(0x0002),
                        ChannelState::new().value(0x0003),
                        ChannelState::new().value(0x0004),
                    ]
                )
                .await,
            Ok(())
        );
        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    #[maybe_async_test]
    async fn sequential_write_too_many_values() {
        let expectations = [];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728
                .sequential_write(
                    Channel::B,
                    OutputEnableMode::Update,
                    &[
                        ChannelState::new().value(0x0001),
                        ChannelState::new().value(0x0002),
                        ChannelState::new().value(0x0003),
                        ChannelState::new().value(0x0004),
                    ]
                )
                .await,
            Err(Error::StartingChannelMismatch)
        );

        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    #[maybe_async_test]
    async fn write_voltage_reference_mode() {
        let expectations = [Transaction::write(0x60, vec![0b10000101])];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728
                .write_voltage_reference_mode(
                    VoltageReferenceMode::External,
                    VoltageReferenceMode::Internal,
                    VoltageReferenceMode::External,
                    VoltageReferenceMode::Internal,
                )
                .await,
            Ok(())
        );

        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    #[maybe_async_test]
    async fn write_gain_mode() {
        let expectations = [Transaction::write(0x60, vec![0b11000101])];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728
                .write_gain_mode(
                    GainMode::TimesOne,
                    GainMode::TimesTwo,
                    GainMode::TimesOne,
                    GainMode::TimesTwo,
                )
                .await,
            Ok(())
        );

        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    #[maybe_async_test]
    async fn write_power_down_mode() {
        let expectations = [Transaction::write(0x60, vec![0b10100001, 0b10110000])];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728
                .write_power_down_mode(
                    PowerDownMode::Normal,
                    PowerDownMode::PowerDownOneK,
                    PowerDownMode::PowerDownOneHundredK,
                    PowerDownMode::PowerDownFiveHundredK,
                )
                .await,
            Ok(())
        );

        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    #[maybe_async_test]
    async fn read() {
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
        let expectations = [Transaction::read(0x60, bytes)];
        let mut i2c = MockI2C::new(&expectations);
        let mut mcp4728 = MCP4728::new(i2c, 0x60);

        assert_eq!(
            mcp4728.read().await,
            Ok(Registers {
                channel_a_input: ChannelRegisters {
                    channel_state: ChannelState::new()
                        .voltage_reference_mode(VoltageReferenceMode::External),
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
                        .voltage_reference_mode(VoltageReferenceMode::External)
                        .power_down_mode(PowerDownMode::PowerDownOneHundredK)
                        .gain_mode(GainMode::TimesTwo)
                        .value(0x0555),
                    ready_state: ReadyState::Ready,
                    power_state: PowerState::On
                },
                channel_b_eeprom: ChannelRegisters {
                    channel_state: ChannelState::new()
                        .voltage_reference_mode(VoltageReferenceMode::External),
                    ready_state: ReadyState::Ready,
                    power_state: PowerState::Off
                },
                channel_c_input: ChannelRegisters {
                    channel_state: ChannelState::new()
                        .voltage_reference_mode(VoltageReferenceMode::External),
                    ready_state: ReadyState::Ready,
                    power_state: PowerState::Off
                },
                channel_c_eeprom: ChannelRegisters {
                    channel_state: ChannelState::new()
                        .voltage_reference_mode(VoltageReferenceMode::External),
                    ready_state: ReadyState::Ready,
                    power_state: PowerState::Off
                },
                channel_d_input: ChannelRegisters {
                    channel_state: ChannelState::new()
                        .voltage_reference_mode(VoltageReferenceMode::External),
                    ready_state: ReadyState::Ready,
                    power_state: PowerState::Off
                },
                channel_d_eeprom: ChannelRegisters {
                    channel_state: ChannelState::new()
                        .voltage_reference_mode(VoltageReferenceMode::External),
                    ready_state: ReadyState::Ready,
                    power_state: PowerState::Off
                },
            })
        );
        i2c = mcp4728.release();

        i2c.done(); // Verify expectations.
    }

    #[maybe_async_test]
    async fn read_fuzz() {
        for i in 0..=u8::MAX {
            let bytes = vec![i; 24];
            let expectations = [Transaction::read(0x60, bytes)];
            let mut i2c = MockI2C::new(&expectations);
            let mut mcp4728 = MCP4728::new(i2c, 0x60);

            assert_eq!(mcp4728.read().await.is_ok(), true);
            i2c = mcp4728.release();

            i2c.done(); // Verify expectations.
        }
    }
}
