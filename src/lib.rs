#![no_std]

extern crate embedded_hal as hal;

use hal::blocking::i2c;
pub struct MCP4728<I2C> {
    i2c: I2C,
    address: u8,
}

#[derive(Debug)]
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
    I2C: i2c::Write<Error = E> + i2c::Read<Error = E> + i2c::WriteRead<Error = E>,
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
