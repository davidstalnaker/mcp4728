use num_enum::{IntoPrimitive, TryFromPrimitive};

// Error type.

/// Error type for the crate, which can represent either an error from this driver or an inner error
/// that comes from the I2C type.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub enum Error<InnerError> {
    /// A value was larger than the DAC supports.
    ///
    /// The MCP4728 is a 12-bit DAC, so values that it writes must be smaller than 2^12.
    ValueOutOfBounds(u16),
    /// [`MCP4728::multi_write`](crate::MCP4728::multi_write) can write an arbitrary number of
    /// updates, so it is impossible to statically allocate a buffer to write using the
    /// [`embedded_hal::i2c::I2c`] trait.  To work around this, we will use a buffer large enough to
    /// contain four writes at a time and return [`Error::WriteSizeExceeded`] if more writes are
    /// requested.  This is unlikely to be a limitation given that there are four channels.
    WriteSizeExceeded,
    /// A sequential write command was issued with a list of updates that didn't match the
    /// associated starting channel.
    ///
    /// For example, a sequential write command that starts with channel B must contain 3 updates:
    /// for channels B, C, and D.
    StartingChannelMismatch,
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
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum Channel {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
}

/// Configuration bit for whether to update the analog output.
#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq, Eq, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum OutputEnableMode {
    /// The analog output will be updated immediately after the command is received.
    Update = 0,
    /// The analog output will not be updated automatically.
    ///
    /// Note that there are other ways to update the analog outputs:
    ///  - Setting the LDAC pin high
    ///  - Issuing a [`MCP4728::general_call_software_update`](crate::MCP4728::general_call_software_update)
    NoUpdate = 1,
}

/// Configuration bit for which voltage reference a channel should use.
#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq, Eq, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
#[repr(u8)]
pub enum VoltageReferenceMode {
    /// Use the external pin VDD as a voltage reference.
    External = 0,
    /// Use the internal 2.048V reference.
    Internal = 1,
}

/// Configuration bits for the powered-down state of a channel.
#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq, Eq, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
pub struct ChannelRegisters {
    /// All mode configuration bits and value of the channel.
    pub channel_state: ChannelState,
    /// The EEPROM Ready state of the device.
    ///
    /// Note that this is global to the entire device, but the protocol reports it for each channel
    /// so that is duplicated here.
    pub ready_state: ReadyState,
    /// The Power-on-reset state of the device.
    ///
    /// Note that this is global to the entire device, but the protocol reports it for each channel
    /// so that is duplicated here.
    pub power_state: PowerState,
}

/// Representation of all registers of all channels.
///
/// Used only for reads.
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt-03", derive(defmt::Format))]
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
    /// Creates a ChannelState with a reasonable default state: internal voltage reference, powered
    /// on, x1 gain, and value of 0.
    pub fn new() -> ChannelState {
        ChannelState {
            voltage_reference_mode: VoltageReferenceMode::Internal,
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
