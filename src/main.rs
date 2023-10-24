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

// reference from https://github.com/vha3/Hunter-Adams-RP2040-Demos/blob/master/Audio/g_Audio_FFT/fft.c
const NUM_SAMPLES: usize = 128;
const LOG2_NUM_SAMPLES: u16 = 7;// 256 = 2^8
// Length of short (16 bits) minus log2 number of samples (6)
const SHIFT_AMOUNT: u16 = 16 - LOG2_NUM_SAMPLES;

fn multfix15(a: i16, b: i16) -> i16 {
    ((a as i32 * b as i32) >> 15) as i16
}

fn fftfix(fr: &mut [i16; NUM_SAMPLES], fi: &mut [i16; NUM_SAMPLES], sinewave: &[i16; NUM_SAMPLES]) -> () {
    //bit order reverse
    for m in 1..(NUM_SAMPLES - 1) {
        // swap odd and even bits
        let mut mr = ((m >> 1) & 0x5555) | ((m & 0x5555) << 1);
        // swap consecutive pairs
        mr = ((mr >> 2) & 0x3333) | ((mr & 0x3333) << 2);
        // swap nibbles ... 
        mr = ((mr >> 4) & 0x0F0F) | ((mr & 0x0F0F) << 4);
        // swap bytes
        mr = ((mr >> 8) & 0x00FF) | ((mr & 0x00FF) << 8);
        // shift down mr
        mr >>= SHIFT_AMOUNT ;
        // don't swap that which has already been swapped
        if mr<=m { continue; }
        // swap the bit-reveresed indices
        let tr = fr[m] ;
        fr[m] = fr[mr] ;
        fr[mr] = tr ;
        let ti = fi[m] ;
        fi[m] = fi[mr] ;
        fi[mr] = ti ;
    }
    //println!("{:?}", fr);
    // Adapted from code by:
    // Tom Roberts 11/8/89 and Malcolm Slaney 12/15/94 malcolm@interval.com
    // Length of the FFT's being combined (starts at 1)
    let mut l: usize = 1 ;
    // Log2 of number of samples, minus 1
    let mut k: u16 = LOG2_NUM_SAMPLES - 1 ;
    // While the length of the FFT's being combined is less than the number 
    // of gathered samples . . .
    while l < NUM_SAMPLES {
        // Determine the length of the FFT which will result from combining two FFT's
        let istep: usize = l << 1 ;
        // For each element in the FFT's that are being combined . . .
        for m in 0..l {
            let j = m << k;
            let mut wr: i16 =  sinewave[j + (NUM_SAMPLES / 4)] ; // cos(2pi m/N)
            let mut wi: i16 = -sinewave[j] ;                 // sin(2pi m/N)
            wr >>= 1 ;                          // divide by two
            wi >>= 1 ;                          // divide by two
            // i gets the index of one of the FFT elements being combined
            let mut i: usize = m;
            while i < NUM_SAMPLES {
                // j gets the index of the FFT element being combined with i
                let j = i + l ;
                // compute the trig terms (bottom half of the above matrix)
                let tr = multfix15(wr, fr[j]) - multfix15(wi, fi[j]) ;
                let ti = multfix15(wr, fi[j]) + multfix15(wi, fr[j]) ;
                // divide ith index elements by two (top half of above matrix)
                let qr = fr[i] >> 1 ;
                let qi = fi[i] >> 1 ;
                // compute the new values at each index
                fr[j] = qr - tr ;
                fi[j] = qi - ti ;
                fr[i] = qr + tr ;
                fi[i] = qi + ti ;

                i += istep;
            }
        }
        k = k - 1;
        l = istep;
    }
}

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

        // should calc once
    let mut sinewave: [i16; NUM_SAMPLES] = [0; NUM_SAMPLES];
    for i in 0..NUM_SAMPLES {
        sinewave[i] = ((6.283 * (i as f32 / NUM_SAMPLES as f32)).sin() as f32 * 32768.0 as f32) as i16; // float2fix15 //2^15
    }

    let buffer1: &mut [u8; 240] = &mut [0; 240];
    let buffer2: &mut [u8; 240] = &mut [0; 240];
    let mut flip: bool = true;
    loop {
        let adc_results: &mut [u16; NUM_SAMPLES] = &mut [0; NUM_SAMPLES];
        for i in 0..NUM_SAMPLES {
            adc_results[i] = adc.convert(&adc_ch0, SampleTime::Cycles_480);
        }

        let mut fr: [i16; NUM_SAMPLES] = [0; NUM_SAMPLES];
        let mut fi: [i16; NUM_SAMPLES] = [0; NUM_SAMPLES];
        for i in 0..NUM_SAMPLES {
            fr[i] = adc_results[i] as i16;
        }

        fftfix(&mut fr, &mut fi, &sinewave);

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
        let mut amplitudes: [u32; NUM_SAMPLES/2] = [0; NUM_SAMPLES/2];
        for i in 0..NUM_SAMPLES/2 { // 128 ~ 240 should 00
            amplitudes[i] = ((fr[i].abs() + fi[i].abs()) as u32 ) << 5;

            if amplitudes[i] > max {
                max = amplitudes[i];
            }
        }

        for i in 0..NUM_SAMPLES/2 { // show fft result
            // buffer[i] = (amplitudes[i] as f32 / max as f32 * 240.0) as u8;
            let adc_8bit: u8 = (amplitudes[i] >> 4) as u8;
            buffer[i] = if adc_8bit > 239 { 239 } else { adc_8bit };
        }

        for i in 0..NUM_SAMPLES { // show raw input in free space
            // raw adc_in is 12bit >> 4 => 8bit
            // u8 max is 255
            // need clamp or scale to 240
            let adc_8bit: u8 = (adc_results[i] >> 4) as u8;
            buffer[(NUM_SAMPLES/2) + i] = if adc_8bit > 239 { 239 } else { adc_8bit };
        }

        let pulse_strength: u8 = ((amplitudes[2] + amplitudes[4] + amplitudes[6] + amplitudes[8]) >> 5) as u8; // depend ball pulse
        for i in ((NUM_SAMPLES/2) + NUM_SAMPLES)..240 {
            buffer[i] = if pulse_strength > 239 { 239 } else { pulse_strength };
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
