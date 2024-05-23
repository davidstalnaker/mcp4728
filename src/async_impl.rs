use crate::internal_types::*;
use crate::types::*;
use crate::MCP4728;

use embedded_hal_async::i2c;

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

/// Implementation of all commands given a generic I2CInterface.
///
/// # Errors
///
/// Any errors encountered within the I2C device will be wrapped in [`Error::I2CError`].
impl<I, E> MCP4728<I>
where
    I: i2c::I2c<Error = E>,
{
    /// Creates a new [`MCP4728`] from an I2C device that implements the
    /// [`embedded_hal_async::i2c::I2c`] trait.
    pub const fn new_async(i2c: I, address: u8) -> Self {
        MCP4728 { i2c, address }
    }

    async fn write_bytes_async(&mut self, address: u8, bytes: &[u8]) -> Result<(), Error<E>> {
        self.i2c.write(address, bytes).await.map_err(Error::I2CError)
    }

    async fn write_iter_async<B>(&mut self, address: u8, bytes: B) -> Result<(), Error<E>>
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
    pub async fn release_async(self) -> I {
        self.i2c
    }

    /// Reads all registers of all channels from the device.
    ///
    /// Each channel includes both the values in EEPROM and in the input registers to the DAC, which
    /// might differ if e.g. fast_write has been used.
    pub async fn read_async(&mut self) -> Result<Registers, Error<E>> {
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
    pub async fn general_call_reset_async(&mut self) -> Result<(), Error<E>> {
        self.write_bytes_async(ADDRESS_GENERAL_CALL, &[COMMAND_GENERAL_CALL_RESET]).await
    }

    /// Issues a general call command (address 0x00) to wake up the device.  All MCP4728 devices on
    /// the bus will reset the power down bits and turn on all channels.
    pub async fn general_call_wake_up_async(&mut self) -> Result<(), Error<E>> {
        self.write_bytes_async(ADDRESS_GENERAL_CALL, &[COMMAND_GENERAL_CALL_WAKE_UP]).await
    }

    /// Issues a general call command (address 0x00) to update software.  All MCP4728 devices on the
    /// bus will immediately update the output voltage.
    pub async fn general_call_software_update_async(&mut self) -> Result<(), Error<E>> {
        self.write_bytes_async(
            ADDRESS_GENERAL_CALL,
            &[COMMAND_GENERAL_CALL_SOFTWARE_UPDATE],
        ).await
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
    /// ```ignore
    /// use mcp4728::MCP4728;
    ///
    /// let mut dac = MCP4728::new_async(i2c, 0x60);
    /// dac.fast_write_async(483, 279, 297, 590).await.unwrap();
    /// ```
    ///
    /// # Errors
    ///
    /// In addition to the internal I2C errors, this can return [`Error::ValueOutOfBounds`] if the
    /// value is out of range (greater than 4095).
    pub async fn fast_write_async(
        &mut self,
        val_a: u16,
        val_b: u16,
        val_c: u16,
        val_d: u16,
    ) -> Result<(), Error<E>> {
        self.fast_write_with_power_down_mode_async(
            (PowerDownMode::Normal, val_a),
            (PowerDownMode::Normal, val_b),
            (PowerDownMode::Normal, val_c),
            (PowerDownMode::Normal, val_d),
        ).await
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
    /// ```ignore
    /// use mcp4728::{MCP4728, PowerDownMode};
    ///
    /// let mut dac = MCP4728::new(i2c, 0x60);
    /// dac.fast_write_with_power_down_mode(
    ///     (PowerDownMode::Normal, 100),
    ///     (PowerDownMode::Normal, 200),
    ///     (PowerDownMode::PowerDownOneK, 0),
    ///     (PowerDownMode::PowerDownOneK, 0),
    /// ).await.unwrap();
    /// ```
    ///
    /// # Errors
    ///
    /// In addition to the internal I2C errors, this can return [`Error::ValueOutOfBounds`] if the
    /// value is out of range (greater than 4095).
    pub async fn fast_write_with_power_down_mode_async(
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
        self.write_bytes_async(self.address, &bytes).await
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
    pub async fn single_write_async(
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
        self.write_bytes_async(self.address, &bytes).await
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
    pub async fn sequential_write_async(
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

        self.write_iter_async(self.address, generator).await
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
    /// allocate a buffer to write using the [`embedded_hal_async::i2c::I2c`] trait.  To work around this,
    /// we will use a buffer large enough to contain four writes at a time and return
    /// [`Error::WriteSizeExceeded`] if more writes are requested.  This is unlikely to be a
    /// limitation given that there are four channels.
    pub async fn multi_write_async(
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

        self.write_iter_async(self.address, generator).await
    }

    /// Writes only the voltage reference mode bits for all channels.
    ///
    /// The EEPROM data is not affected and the output of each channel is updated after the command
    /// has been received.
    pub async fn write_voltage_reference_mode_async(
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
        self.write_bytes_async(self.address, &[byte]).await
    }

    /// Writes only the gain mode bits for all channels.
    ///
    /// The EEPROM data is not affected and the output of each channel is updated after the command
    /// has been received.
    pub async fn write_gain_mode_async(
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
        self.write_bytes_async(self.address, &[byte]).await
    }

    /// Writes only the power down mode bits for all channels.
    ///
    /// The EEPROM data is not affected and the output of each channel is updated after the command
    /// has been received.
    pub async fn write_power_down_mode_async(
        &mut self,
        mode_a: PowerDownMode,
        mode_b: PowerDownMode,
        mode_c: PowerDownMode,
        mode_d: PowerDownMode,
    ) -> Result<(), Error<E>> {
        let mut bytes = [0; 2];
        bytes[0] = COMMAND_WRITE_POWER_DOWN_MODE | (mode_a as u8) << 2 | mode_b as u8;
        bytes[1] = (mode_c as u8) << 6 | (mode_d as u8) << 4;
        self.write_bytes_async(self.address, &bytes).await
    }
}
