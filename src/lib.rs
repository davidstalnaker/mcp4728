#![cfg_attr(not(test), no_std)]

extern crate embedded_hal as hal;

use hal::blocking::i2c;

#[derive(Debug, PartialEq)]
pub enum Error<InnerError> {
    ValueOutOfBounds(u16),
    I2CError(InnerError),
}

impl<InnerError> From<InnerError> for Error<InnerError> {
    fn from(inner: InnerError) -> Self {
        Error::I2CError(inner)
    }
}

pub enum Channel {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
}

pub enum OutputEnableMode {
    Update = 0,
    NoUpdate = 1,
}

pub enum VoltageReferenceMode {
    External = 0,
    Internal = 1,
}

pub enum PowerDownMode {
    Normal = 0,
    PowerDownOneK = 1,
    PowerDownOneHundredK = 2,
    PowerDownFiveHundredK = 3,
}

pub enum GainMode {
    TimesOne = 0,
    TimesTwo = 1,
}

pub struct ChannelState {
    output_enable_mode: OutputEnableMode,
    voltage_reference_mode: VoltageReferenceMode,
    power_down_mode: PowerDownMode,
    gain_mode: GainMode,
    value: u16,
}

impl ChannelState {
    pub fn new() -> ChannelState {
        ChannelState {
            output_enable_mode: OutputEnableMode::Update,
            voltage_reference_mode: VoltageReferenceMode::External,
            power_down_mode: PowerDownMode::Normal,
            gain_mode: GainMode::TimesOne,
            value: 0,
        }
    }

    pub fn output_enable_mode<'a>(&'a mut self, new_val: OutputEnableMode) -> &'a mut ChannelState {
        self.output_enable_mode = new_val;
        self
    }

    pub fn voltage_reference_mode<'a>(
        &'a mut self,
        new_val: VoltageReferenceMode,
    ) -> &'a mut ChannelState {
        self.voltage_reference_mode = new_val;
        self
    }

    pub fn power_down_mode<'a>(&'a mut self, new_val: PowerDownMode) -> &'a mut ChannelState {
        self.power_down_mode = new_val;
        self
    }

    pub fn gain_mode<'a>(&'a mut self, new_val: GainMode) -> &'a mut ChannelState {
        self.gain_mode = new_val;
        self
    }

    pub fn value<'a>(&'a mut self, new_val: u16) -> &'a mut ChannelState {
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
    I2C: i2c::Write<Error = E>,
{
    pub fn new(i2c: I2C, address: u8) -> MCP4728<I2C> {
        MCP4728 { i2c, address }
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
        self.i2c.write(self.address, &bytes).map_err(|e| e.into())
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
        self.i2c.write(self.address, &bytes).map_err(|e| e.into())
    }

    pub fn fast_power_down_all(&mut self, mode: &PowerDownMode) -> Result<(), Error<E>> {
        self.fast_power_down(&mode, &mode, &mode, &mode)
    }

    pub fn single_write(
        &mut self,
        channel: Channel,
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
        bytes[0] |= channel_state.output_enable_mode as u8;
        bytes[1] |= (channel_state.voltage_reference_mode as u8) << 7;
        bytes[1] |= (channel_state.power_down_mode as u8) << 5;
        bytes[1] |= (channel_state.gain_mode as u8) << 4;
        bytes[1] |= channel_state.value.to_be_bytes()[0];
        bytes[2] = channel_state.value.to_be_bytes()[1];
        self.i2c.write(self.address, &bytes).map_err(|e| e.into())
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
        self.i2c.write(self.address, &[byte]).map_err(|e| e.into())
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
        self.i2c.write(self.address, &[byte]).map_err(|e| e.into())
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
        self.i2c.write(self.address, &bytes).map_err(|e| e.into())
    }
}

#[cfg(test)]
mod tests {
    extern crate embedded_hal as hal;

    use hal::blocking::i2c;

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
        WriteError,
    }

    struct FakeI2C {
        messages: Rc<RefCell<Vec<FakeI2CMessage>>>,
        should_fail: Rc<RefCell<bool>>,
    }

    impl FakeI2C {
        fn new() -> FakeI2C {
            FakeI2C {
                messages: Rc::new(RefCell::new(vec![])),
                should_fail: Rc::new(RefCell::new(false)),
            }
        }
    }

    impl i2c::Write for FakeI2C {
        type Error = FakeI2CError;
        fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), FakeI2CError> {
            if !*self.should_fail.borrow() {
                self.messages.borrow_mut().push(FakeI2CMessage {
                    address,
                    bytes: bytes.to_vec(),
                });
                Ok(())
            } else {
                Err(FakeI2CError::WriteError)
            }
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
            mcp4782.single_write(Channel::B, ChannelState::new().value(0x0aaa)),
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
                ChannelState::new()
                    .output_enable_mode(OutputEnableMode::NoUpdate)
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
            mcp4782.single_write(Channel::B, ChannelState::new().value(0xffff)),
            Err(Error::ValueOutOfBounds(0xffff))
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
}
