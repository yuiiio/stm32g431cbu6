#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger
extern crate cortex_m;
extern crate cortex_m_rt;
extern crate stm32g4xx_hal as hal;

use core::f32::consts::{PI};
use micromath::F32Ext;

use crate::hal::{
    prelude::*,
    spi,
    adc::{
        config::{Continuous, Dma as AdcDma, SampleTime, Sequence},
        AdcClaim, ClockSource, Temperature, Vref,
    },
    delay::SYSTDelayExt,
    dma::{config::DmaConfig, stream::DMAExt, TransferExt},
    gpio::GpioExt,
    gpio::gpioa,
    gpio::Alternate,
    gpio::AF5,
    rcc::{Config, RccExt},
    stm32::Peripherals,
    block,
    time::Hertz,
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
    let mut sinewave: [i16; NUM_SAMPLES] = [0; NUM_SAMPLES];
    for i in 0..NUM_SAMPLES {
        sinewave[i] = ((6.283 * (i as f32 / NUM_SAMPLES as f32)).sin() as f32 * 32768.0 as f32) as i16; // float2fix15 //2^15
    }

    let mut rsqrt_table: [u16; ((u16::MAX as u32 + 1) / 8) as usize] = [0; ((u16::MAX as u32 + 1) / 8) as usize]; // /8 resolution: (65536 * 16 bit = 130.xx KBytes) /8 => 
    for i in 0..((u16::MAX as u32 + 1) / 8) {
        let x = i << 3; // *8
        rsqrt_table[i as usize] = ((1.0 / (x as f32).sqrt()) * u16::MAX as f32) as u16;
    }

    let dp = Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let mut rcc = rcc.freeze(Config::hsi());
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    let mut cp_delay = cp.SYST.delay(&rcc.clocks);

    // for st7789
    let rst = gpioc.pc4.into_push_pull_output(); // reset pin
    let dc = gpiob.pb2.into_push_pull_output(); // dc pin

    let spi1_sclk: gpioa::PA5<Alternate<AF5>> = gpioa.pa5.into_alternate();
    let spi1_miso: gpioa::PA6<Alternate<AF5>> = gpioa.pa6.into_alternate();
    let spi1_mosi: gpioa::PA7<Alternate<AF5>> = gpioa.pa7.into_alternate();
    let spi1 = dp
        .SPI1
        .spi((spi1_sclk, spi1_miso, spi1_mosi), spi::MODE_3, 8000000.hz(), &mut rcc);
    
    // display interface abstraction from SPI and DC
    let di = SPIInterfaceNoCS::new(spi1, dc);
    // create driver
    let mut display = ST7789::new(di, rst, 240, 240);
    // initialize
    display.init(&mut cp_delay).unwrap();
    // set default orientation
    display.set_orientation(Orientation::LandscapeSwapped).unwrap();
    display.clear(Rgb565::GREEN).unwrap();
    cp_delay.delay_ms(1000_u32);
    display.clear(Rgb565::BLACK).unwrap();

    let mut led_blue = gpioc.pc6.into_push_pull_output();
    led_blue.set_low().unwrap();
    
    let streams = dp.DMA1.split(&rcc);
    let config = DmaConfig::default()
        .transfer_complete_interrupt(false)
        .circular_buffer(true) // circular mode affects FFT result ?
        .memory_increment(true);

    let pa0 = gpioa.pa0.into_analog();
    let pa1 = gpioa.pa1.into_analog();
    let pa2 = gpioa.pa2.into_analog();
    let pa3 = gpioa.pa3.into_analog();
    //let pb14 = gpiob.pb14.into_analog();
    let pb12 = gpiob.pb12.into_analog();
    let pb1 = gpiob.pb1.into_analog();
    let pb11 = gpiob.pb11.into_analog();
    let pb0 = gpiob.pb0.into_analog();
    
    let mut adc = dp
        .ADC1
        .claim(ClockSource::SystemClock, &rcc, &mut cp_delay, true);

    adc.set_continuous(Continuous::Continuous);
    adc.reset_sequence();
    adc.configure_channel(&pa0,  Sequence::One,   SampleTime::Cycles_6_5);
    adc.configure_channel(&pa1,  Sequence::Two,   SampleTime::Cycles_6_5);
    adc.configure_channel(&pa2,  Sequence::Three, SampleTime::Cycles_6_5);
    adc.configure_channel(&pa3,  Sequence::Four,  SampleTime::Cycles_6_5);
    adc.configure_channel(&pb12, Sequence::Five,  SampleTime::Cycles_6_5);
    adc.configure_channel(&pb1,  Sequence::Six,   SampleTime::Cycles_6_5);
    adc.configure_channel(&pb11, Sequence::Seven, SampleTime::Cycles_6_5);
    adc.configure_channel(&pb0,  Sequence::Eight, SampleTime::Cycles_6_5);
    
    let first_buffer = cortex_m::singleton!(: [u16; (8*NUM_SAMPLES) + 1] = [0; (8*NUM_SAMPLES) + 1]).unwrap();
    let mut transfer = streams.0.into_circ_peripheral_to_memory_transfer(
        adc.enable_dma(AdcDma::Continuous),
        &mut first_buffer[..],
        config,
    );

    transfer.start(|adc| adc.start_conversion());

    loop {
        let mut dma_buf = [0_u16; 8*NUM_SAMPLES];
        let r = transfer.read_exact(&mut dma_buf);
        assert!(r == dma_buf.len());

        let adc_results: &mut [[u16; NUM_SAMPLES]; 8] = &mut [[0; NUM_SAMPLES]; 8];
        for i in 0..NUM_SAMPLES {
            let j = i * 8;
            adc_results[0][i] = dma_buf[j+0];
            adc_results[1][i] = dma_buf[j+1];
            adc_results[2][i] = dma_buf[j+2];
            adc_results[3][i] = dma_buf[j+3];
            adc_results[4][i] = dma_buf[j+4];
            adc_results[5][i] = dma_buf[j+5];
            adc_results[6][i] = dma_buf[j+6];
            adc_results[7][i] = dma_buf[j+7];
        }

        let mut fr: [i16; NUM_SAMPLES] = [0; NUM_SAMPLES];
        let mut fi: [i16; NUM_SAMPLES] = [0; NUM_SAMPLES];
        for i in 0..NUM_SAMPLES {
            fr[i] = adc_results[0][i] as i16;
        }

        fftfix(&mut fr, &mut fi, &sinewave);

        let mut amplitudes: [u16; NUM_SAMPLES/2] = [0; NUM_SAMPLES/2];
        for i in 0..NUM_SAMPLES/2 { // 128 ~ 240 should 00
            amplitudes[i] = (fr[i].abs() + fi[i].abs()) as u16;
        }

        let pulse_strength: u16 = amplitudes[5] as u16; // depend ball pulse
        
        let ball_dist: u16 = rsqrt_table[(pulse_strength >> 3) as usize];
        
        cp_delay.delay_ms((ball_dist >> 4) as u32);

        led_blue.set_high().unwrap();
        cp_delay.delay_ms(1000_u32);
        led_blue.set_low().unwrap();
        cp_delay.delay_ms(1000_u32);
    }
}
