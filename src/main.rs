#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger
extern crate cortex_m;
extern crate cortex_m_rt;
extern crate stm32f4xx_hal as hal;

use hal::{
    pac,
    gpio::{Speed, PinState},
    prelude::*,
    spi::*,
};

//use cortex_m::asm;
use cortex_m_rt::entry;
//use cortex_m::peripheral::{Peripherals, syst};

use fugit::RateExtU32;
use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::image::*;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use st7789::{Orientation, ST7789};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // SPI1 (max 42 Mbit/s) (SCK: PB3, MISO: PB4, MOSI: PB5)
    // SPI2 (max 21 Mbit/s)

    let rcc = dp.RCC.constrain();
    
    let clocks = rcc
        .cfgr
        .use_hse(25.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz())
        .pclk2(24.MHz())
        .freeze();

    let gpioc = dp.GPIOC.split();
    
    let mut led_blue = gpioc.pc13.into_push_pull_output();
    led_blue.set_low();

    let gpiob = dp.GPIOB.split();

    let mut cp_delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().to_Hz());

    // for st7789 display
    let rst = gpiob.pb6.into_push_pull_output_in_state(PinState::Low); // reset pin
    let dc = gpiob.pb7.into_push_pull_output_in_state(PinState::Low); // dc pin
    // Note. We set GPIO speed as VeryHigh to it corresponds to SPI frequency 3MHz.
    // Otherwise it may lead to the 'wrong last bit in every received byte' problem.
    let spi_mosi = gpiob
        .pb5
        .into_alternate()
        .speed(Speed::VeryHigh)
        .internal_pull_up(true);

    let spi_sclk = gpiob.pb3.into_alternate().speed(Speed::VeryHigh);

    
    let spi = Spi::new(dp.SPI1, (spi_sclk, NoMiso::new(), spi_mosi), embedded_hal::spi::MODE_3, 16.MHz(), &clocks);
    
    // display interface abstraction from SPI and DC
    let di = SPIInterfaceNoCS::new(spi, dc);
    
    // create driver
    let mut display = ST7789::new(di, rst, 240, 240);

    // initialize
    display.init(&mut cp_delay).unwrap();
    // set default orientation
    display.set_orientation(Orientation::LandscapeSwapped).unwrap();

    let raw_image_data = ImageRawLE::new(include_bytes!("../assets/rust.raw"), 240);
    let ferris = Image::new(&raw_image_data, Point::new(80, 0));

    // draw image on black background
    display.clear(Rgb565::BLACK).unwrap();
    ferris.draw(&mut display).unwrap();


    loop {
        led_blue.set_high();
        cp_delay.delay_ms(500_u32);
        led_blue.set_low();
        cp_delay.delay_ms(500_u32);
    }
}
