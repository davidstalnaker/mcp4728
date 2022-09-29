#![cfg_attr(not(test), no_std)]

extern crate embedded_hal as hal;

use hal::blocking::i2c;
pub struct MCP4728<I2C> {
    i2c: I2C,
    address: u8,
}

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

impl<I2C, E> MCP4728<I2C>
where
    I2C: i2c::Write<Error = E>, // I2C: i2c::Write<Error = E> + i2c::Read<Error = E> + i2c::WriteRead<Error = E>,
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
}

#[cfg(test)]
mod tests {
    extern crate embedded_hal as hal;

    use hal::blocking::i2c;

    use std::cell::RefCell;
    use std::rc::Rc;

    use crate::MCP4728;

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
            Err(crate::Error::I2CError(FakeI2CError::WriteError))
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
            Err(crate::Error::ValueOutOfBounds(0x1000))
        );
        assert_eq!(*messages.borrow(), vec![]);
    }
}
