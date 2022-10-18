use embedded_hal::blocking::i2c;

use std::cell::RefCell;
use std::rc::Rc;

#[derive(Debug, PartialEq)]
pub struct FakeI2CMessage {
    pub address: u8,
    pub bytes: Vec<u8>,
}

#[derive(Debug, PartialEq)]
pub enum FakeI2CError {
    ReadError,
    WriteError,
}

pub struct FakeI2C {
    pub messages: Rc<RefCell<Vec<FakeI2CMessage>>>,
    pub message_to_read: Rc<RefCell<FakeI2CMessage>>,
    pub should_fail: Rc<RefCell<bool>>,
}

impl FakeI2C {
    pub fn new() -> FakeI2C {
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

impl i2c::Write for FakeI2C {
    type Error = FakeI2CError;
    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), FakeI2CError> {
        if !*self.should_fail.borrow() {
            self.messages.borrow_mut().push(FakeI2CMessage {
                address,
                bytes: Vec::from(bytes),
            });
            Ok(())
        } else {
            Err(FakeI2CError::WriteError)
        }
    }
}
