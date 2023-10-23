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

use core::f32::consts::{PI};
use micromath::F32Ext;

use hal::{
    pac::{self, ADC1},
    gpio::{Speed, PinState},
    prelude::*,
    spi::*,
    adc::{
        config::{AdcConfig, Clock, Dma, Resolution, SampleTime, Scan, Sequence},
        Adc,
    },
    dma::{config::DmaConfig, PeripheralToMemory, Stream0, StreamsTuple, Transfer},
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

use microfft;

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
        .sysclk(84.MHz())
        .hclk(84.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
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

    
    let spi = Spi::new(dp.SPI1, (spi_sclk, NoMiso::new(), spi_mosi), embedded_hal::spi::MODE_3, 42.MHz(), &clocks);
    
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
    //cp_delay.delay_ms(500_u32);
    
    led_blue.set_high(); // disable led

    display.clear(Rgb565::BLACK).unwrap();

    // setup GPIOA
    let gpioa = dp.GPIOA.split();

    // Configure pa0 as an analog input
    let adc_ch0 = gpioa.pa0.into_analog();

    let adc_config = AdcConfig::default()
            //.dma(Dma::Continuous)
            //Scan mode is also required to convert a sequence
            .scan(Scan::Enabled)
            .resolution(Resolution::Twelve)
            .clock(Clock::Pclk2_div_4); // 84 / 4 = 21MHz need more down ? (adc max datasheet says
                                        // 36MHz)
                                        // 12-bit resolution single ADC 2 Msps

    // setup ADC
    let mut adc = Adc::adc1(dp.ADC1, true, adc_config);

    //adc.configure_channel(&adc_ch0, Sequence::One, SampleTime::Cycles_480); // need 480 cycles from datasheet

    let buffer1: &mut [u8; 240] = &mut [0; 240];
    let buffer2: &mut [u8; 240] = &mut [0; 240];
    let mut flip: bool = true;
    loop {
        let adc_results: &mut [u16; 256] = &mut [0; 256];
        for i in 0..256 {
            adc_results[i] = adc.convert(&adc_ch0, SampleTime::Cycles_480);
        }

        let mut samples: [f32; 256] = [0.0; 256];
        for i in 0..256 {
            samples[i] = adc_results[i] as f32;
        }

        let spectrum = microfft::real::rfft_256(&mut samples);
        spectrum[0].im = 0.0; // need ?

        let mut buffer: &mut [u8; 240] = &mut [0; 240];
        let mut prev: &mut [u8; 240] = &mut [0; 240];
        if flip == true {
            buffer = buffer1;
            prev = buffer2;
            flip = false;
        } else {
            buffer = buffer2;
            prev = buffer1;
            flip = true;
        }

        let mut max: u32 = 0;
        let mut amplitudes: [u32; 240] = [0; 240];
        for i in 0..128 { // 128 ~ 240 should 00
            amplitudes[i] = (spectrum[i].re.abs() + spectrum[i].im.abs()) as u32;

            if amplitudes[i] > max {
                max = amplitudes[i];
            }
        }

        for i in 0..128 { // show fft result
            // buffer[i] = (amplitudes[i] as f32 / max as f32 * 240.0) as u8;
            let adc_8bit: u8 = (amplitudes[i] >> 4) as u8;
            buffer[i] = if adc_8bit > 239 { 239 } else { adc_8bit };
        }

        for i in 128..240 { // show raw input in free space
            // raw adc_in is 12bit >> 4 => 8bit
            // u8 max is 255
            // need clamp or scale to 240
            let adc_8bit: u8 = (adc_results[i] >> 4) as u8;
            buffer[i] = if adc_8bit > 239 { 239 } else { adc_8bit };
        }
        
        // clear
        for i in 0..240 {
            let value = prev[i as usize] as u8;
            //for pos in 0.. value { 
                display.set_pixel(i+80, value as u16, 0b0000000000000000).ok();
            //}
        }

        // draw
        for i in 0..240 {
            let value = buffer[i as usize] as u8;
            //for pos in 0.. value { 
                display.set_pixel(i+80, value as u16, 0b1111111111111111).ok();
            //}
        }

        cp_delay.delay_ms(1000_u32);
    }
}
