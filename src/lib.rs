#![cfg_attr(not(test), no_std)]

extern crate embedded_hal as hal;

use hal::blocking::i2c;
use num_enum::{IntoPrimitive, TryFromPrimitive};

#[derive(Debug, PartialEq)]
pub enum Error<InnerError> {
    ValueOutOfBounds(u16),
    StartingChannelNotEqualToUpdateLength,
    I2CError(InnerError),
}

impl<InnerError> From<InnerError> for Error<InnerError> {
    fn from(inner: InnerError) -> Self {
        Error::I2CError(inner)
    }
}

#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq)]
#[repr(u8)]
pub enum Channel {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
}

#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq)]
#[repr(u8)]
pub enum OutputEnableMode {
    Update = 0,
    NoUpdate = 1,
}

#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq)]
#[repr(u8)]
pub enum VoltageReferenceMode {
    External = 0,
    Internal = 1,
}

#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq)]
#[repr(u8)]
pub enum PowerDownMode {
    Normal = 0,
    PowerDownOneK = 1,
    PowerDownOneHundredK = 2,
    PowerDownFiveHundredK = 3,
}

#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq)]
#[repr(u8)]
pub enum GainMode {
    TimesOne = 0,
    TimesTwo = 1,
}

#[derive(Debug, PartialEq)]
pub struct ChannelState {
    voltage_reference_mode: VoltageReferenceMode,
    power_down_mode: PowerDownMode,
    gain_mode: GainMode,
    value: u16,
}

#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq)]
#[repr(u8)]
pub enum ReadyState {
    Ready = 0,
    Busy = 1,
}

#[derive(IntoPrimitive, TryFromPrimitive, Debug, PartialEq)]
#[repr(u8)]
pub enum PowerState {
    Off = 0,
    On = 1,
}

#[derive(Debug, PartialEq)]
pub struct ChannelRegisters {
    channel_state: ChannelState,
    ready_state: ReadyState,
    power_state: PowerState,
}

#[derive(Debug, PartialEq)]
pub struct Registers {
    channel_a_input: ChannelRegisters,
    channel_a_eeprom: ChannelRegisters,
    channel_b_input: ChannelRegisters,
    channel_b_eeprom: ChannelRegisters,
    channel_c_input: ChannelRegisters,
    channel_c_eeprom: ChannelRegisters,
    channel_d_input: ChannelRegisters,
    channel_d_eeprom: ChannelRegisters,
}

impl ChannelState {
    pub fn new() -> ChannelState {
        ChannelState {
            voltage_reference_mode: VoltageReferenceMode::External,
            power_down_mode: PowerDownMode::Normal,
            gain_mode: GainMode::TimesOne,
            value: 0,
        }
    }

    pub fn voltage_reference_mode(mut self, new_val: VoltageReferenceMode) -> ChannelState {
        self.voltage_reference_mode = new_val;
        self
    }

    pub fn power_down_mode(mut self, new_val: PowerDownMode) -> ChannelState {
        self.power_down_mode = new_val;
        self
    }

    pub fn gain_mode(mut self, new_val: GainMode) -> ChannelState {
        self.gain_mode = new_val;
        self
    }

    pub fn value(mut self, new_val: u16) -> ChannelState {
        self.value = new_val;
        self
    }
}

pub struct MCP4728<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> MCP4728<I2C>
where
    I2C: i2c::Read<Error = E> + i2c::WriteIter<Error = E> + i2c::Write<Error = E>,
{
    pub fn new(i2c: I2C, address: u8) -> MCP4728<I2C> {
        MCP4728 { i2c, address }
    }

    pub fn destroy(self) -> I2C {
        self.i2c
    }

    pub fn general_call_reset(&mut self) -> Result<(), Error<E>> {
        i2c::Write::write(&mut self.i2c, 0x00, &[0b00000110])?;
        Ok(())
    }

    pub fn general_call_wake_up(&mut self) -> Result<(), Error<E>> {
        i2c::Write::write(&mut self.i2c, 0x00, &[0b00001001])?;
        Ok(())
    }

    pub fn general_call_software_update(&mut self) -> Result<(), Error<E>> {
        i2c::Write::write(&mut self.i2c, 0x00, &[0b00001000])?;
        Ok(())
    }

    pub fn fast_write(
        &mut self,
        val_a: u16,
        val_b: u16,
        val_c: u16,
        val_d: u16,
    ) -> Result<(), Error<E>> {
        let mut bytes = [0; 8];
        for (i, &val) in [val_a, val_b, val_c, val_d].iter().enumerate() {
            if val > 0x0fff {
                return Err(Error::ValueOutOfBounds(val));
            }
            let new_bytes = val.to_be_bytes();
            bytes[2 * i] = new_bytes[0];
            bytes[2 * i + 1] = new_bytes[1];
        }
        i2c::Write::write(&mut self.i2c, self.address, &bytes)?;
        Ok(())
    }

    pub fn fast_power_down(
        &mut self,
        mode_a: &PowerDownMode,
        mode_b: &PowerDownMode,
        mode_c: &PowerDownMode,
        mode_d: &PowerDownMode,
    ) -> Result<(), Error<E>> {
        let mut bytes = [0; 8];
        for (i, &mode) in [mode_a, mode_b, mode_c, mode_d].iter().enumerate() {
            bytes[2 * i] = (*mode as u8) << 4;
        }
        i2c::Write::write(&mut self.i2c, self.address, &bytes)?;
        Ok(())
    }

    pub fn fast_power_down_all(&mut self, mode: &PowerDownMode) -> Result<(), Error<E>> {
        self.fast_power_down(&mode, &mode, &mode, &mode)
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
        bytes[0] |= 0b01011000;
        bytes[0] |= (channel as u8) << 1;
        bytes[0] |= output_enable_mode as u8;
        bytes[1] |= (channel_state.voltage_reference_mode as u8) << 7;
        bytes[1] |= (channel_state.power_down_mode as u8) << 5;
        bytes[1] |= (channel_state.gain_mode as u8) << 4;
        bytes[1] |= channel_state.value.to_be_bytes()[0];
        bytes[2] = channel_state.value.to_be_bytes()[1];
        i2c::Write::write(&mut self.i2c, self.address, &bytes)?;
        Ok(())
    }

    pub fn multi_write(
        &mut self,
        channel_updates: &[(Channel, OutputEnableMode, &ChannelState)],
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
                0 => 0b01000000 | (*channel as u8) << 1 | *output_enable_mode as u8,

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

        i2c::WriteIter::write(&mut self.i2c, self.address, generator)?;
        Ok(())
    }

    pub fn sequential_write(
        &mut self,
        starting_channel: Channel,
        output_enable_mode: OutputEnableMode,
        channel_updates: &[&ChannelState],
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
                byte = 0b01010000 | (starting_channel as u8) << 1 | output_enable_mode as u8;
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

        i2c::WriteIter::write(&mut self.i2c, self.address, generator)?;
        Ok(())
    }

    pub fn write_voltage_reference_mode(
        &mut self,
        mode_a: VoltageReferenceMode,
        mode_b: VoltageReferenceMode,
        mode_c: VoltageReferenceMode,
        mode_d: VoltageReferenceMode,
    ) -> Result<(), Error<E>> {
        let mut byte = 0b10000000;
        byte |= (mode_a as u8) << 3;
        byte |= (mode_b as u8) << 2;
        byte |= (mode_c as u8) << 1;
        byte |= mode_d as u8;
        i2c::Write::write(&mut self.i2c, self.address, &[byte])?;
        Ok(())
    }

    pub fn write_gain_mode(
        &mut self,
        mode_a: GainMode,
        mode_b: GainMode,
        mode_c: GainMode,
        mode_d: GainMode,
    ) -> Result<(), Error<E>> {
        let mut byte = 0b11000000;
        byte |= (mode_a as u8) << 3;
        byte |= (mode_b as u8) << 2;
        byte |= (mode_c as u8) << 1;
        byte |= mode_d as u8;
        i2c::Write::write(&mut self.i2c, self.address, &[byte])?;
        Ok(())
    }

    pub fn write_power_down_mode(
        &mut self,
        mode_a: PowerDownMode,
        mode_b: PowerDownMode,
        mode_c: PowerDownMode,
        mode_d: PowerDownMode,
    ) -> Result<(), Error<E>> {
        let mut bytes = [0; 2];
        bytes[0] = 0b10100000;
        bytes[0] |= (mode_a as u8) << 2;
        bytes[0] |= mode_b as u8;
        bytes[1] |= (mode_c as u8) << 6;
        bytes[1] |= (mode_d as u8) << 4;
        i2c::Write::write(&mut self.i2c, self.address, &bytes)?;
        Ok(())
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
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(mcp4782.fast_write(0x0aaa, 0x0000, 0x0aaa, 0x0000), Ok(()));
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
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.fast_write(0x0aaa, 0x0000, 0x0aaa, 0x0000),
            Err(Error::I2CError(FakeI2CError::WriteError))
        );
        assert_eq!(*messages.borrow(), vec![]);
    }

    #[test]
    fn fast_write_out_of_bounds_error() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.fast_write(0x1000, 0x0000, 0x0000, 0x0000),
            Err(Error::ValueOutOfBounds(0x1000))
        );
        assert_eq!(*messages.borrow(), vec![]);
    }

    #[test]
    fn fast_power_down() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.fast_power_down(
                &PowerDownMode::Normal,
                &PowerDownMode::PowerDownOneK,
                &PowerDownMode::PowerDownOneHundredK,
                &PowerDownMode::PowerDownFiveHundredK
            ),
            Ok(())
        );
        assert_eq!(
            *messages.borrow(),
            vec![FakeI2CMessage {
                address: 0x60,
                bytes: vec![0x00, 0x00, 0x10, 0x00, 0x20, 0x00, 0x30, 0x00]
            }]
        );
    }

    #[test]
    fn fast_power_down_i2c_error() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        *i2c.should_fail.borrow_mut() = true;
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.fast_power_down(
                &PowerDownMode::Normal,
                &PowerDownMode::PowerDownOneK,
                &PowerDownMode::PowerDownOneHundredK,
                &PowerDownMode::PowerDownFiveHundredK
            ),
            Err(Error::I2CError(FakeI2CError::WriteError))
        );
        assert_eq!(*messages.borrow(), vec![]);
    }

    // || 0 1 0 1 1 CH CH OE || VR PD PD G D D D D || D D D D D D D D ||
    #[test]
    fn single_write_default_values() {
        let i2c = FakeI2C::new();
        let messages = Rc::clone(&i2c.messages);
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.single_write(
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
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.single_write(
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
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.single_write(
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
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.multi_write(&[(
                Channel::D,
                OutputEnableMode::NoUpdate,
                &ChannelState::new()
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
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.multi_write(&[
                (
                    Channel::A,
                    OutputEnableMode::Update,
                    &ChannelState::new().value(0x0001)
                ),
                (
                    Channel::B,
                    OutputEnableMode::Update,
                    &ChannelState::new().value(0x0002)
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
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.sequential_write(
                Channel::A,
                OutputEnableMode::Update,
                &[
                    &ChannelState::new().value(0x0001),
                    &ChannelState::new().value(0x0002),
                    &ChannelState::new().value(0x0003),
                    &ChannelState::new().value(0x0004),
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
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.sequential_write(
                Channel::B,
                OutputEnableMode::Update,
                &[
                    &ChannelState::new().value(0x0001),
                    &ChannelState::new().value(0x0002),
                    &ChannelState::new().value(0x0003),
                    &ChannelState::new().value(0x0004),
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
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.write_voltage_reference_mode(
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
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.write_gain_mode(
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
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.write_power_down_mode(
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
        let mut mcp4782 = MCP4728::new(i2c, 0x60);
        assert_eq!(
            mcp4782.read(),
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
