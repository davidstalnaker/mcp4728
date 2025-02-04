#![no_std]
#![no_main]

use embedded_hal::digital::OutputPin;
use fugit::RateExtU32;
use hal::pac;
use mcp4728::MCP4728;
use panic_halt as _;
use rp2040_hal as hal;
use rp2040_hal::Clock;
use rp_pico::entry;
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[entry]
fn main() -> ! {
    // Device-specific setup for Raspberry Pi Pico.
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    let sda_pin = pins
        .gpio4
        .into_function::<hal::gpio::FunctionI2C>()
        .into_pull_type::<hal::gpio::PullUp>();
    let scl_pin = pins
        .gpio5
        .into_function::<hal::gpio::FunctionI2C>()
        .into_pull_type::<hal::gpio::PullUp>();

    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.peripheral_clock,
    );

    // MCP4728 example usage.
    let mut dac = MCP4728::new(i2c, 0x64);

    loop {
        led_pin.set_high().unwrap();
        dac.fast_write(0xfff, 0xfff, 0xfff, 0xfff).unwrap();
        delay.delay_ms(1000);

        led_pin.set_low().unwrap();
        dac.fast_write(0, 0, 0, 0).unwrap();
        delay.delay_ms(1000);
    }
}
