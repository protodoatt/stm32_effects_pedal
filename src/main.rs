#![no_main]
#![no_std]

use embassy_executor::{Spawner, InterruptExecutor};
use embassy_stm32::sai::*;
use embassy_stm32::sai;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::{bind_interrupts, dma, peripherals, time::mhz};
use embassy_stm32::peripherals::*;
use embassy_time::Instant;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use defmt::{error, info};
use crate::dsp::{prep_for_i2s, prep_for_processing};

use {defmt_rtt as _, panic_probe as _};
use core::f32::consts::PI;

mod dsp;

static AUDIO_CLIP: &[u8] = include_bytes!("clip.raw");

//force dma buffers in sram 1 where dma wants it
#[unsafe(link_section = ".sram1_bss")]
static mut DMA_RING_BUFFER: [u32; 4096] = [0; 4096];
#[unsafe(link_section = ".sram1_bss")]
static mut DMA_RX_RING_BUFFER: [u32; 4096] = [0; 4096];

static PRE_GAIN_BITS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0x3f800000); //1.0f32
static POST_GAIN_BITS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0x3f800000); //1.0f32

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn CEC() {
    unsafe { EXECUTOR_HIGH.on_interrupt() };
}

bind_interrupts!(struct Irqs {
    DMA2_STREAM1 => dma::InterruptHandler<peripherals::DMA2_CH1>;
    DMA2_STREAM4 => dma::InterruptHandler<peripherals::DMA2_CH4>;
});

#[embassy_executor::task]
async fn audio_task(
    mut sai_rx: Sai<'static, peripherals::SAI1, u32>,
    mut sai_tx: Sai<'static, peripherals::SAI1, u32>,
) {
    let mut sine_buffer = [0f32; 1024];
    let mut filtered_buffer = [0f32; 1024];
    let mut fir_filter = dsp::CmsisFir::new();
    fir_filter.init();
    let mut odp = dsp::overdrive_params::default();
    
    info!("Init successful!");
    let mut process_buf = [0u32; 2048];
    
    let mut loop_counter = 0;
    sai_rx.start().unwrap();
    sai_tx.write(&process_buf).await.unwrap(); //according to embassy documentation, we need to start write first
    sai_tx.write(&process_buf).await.unwrap();
    
    loop {
        //read the mapped values from the background adc task via atomics     
        odp.pre_gain = f32::from_bits(PRE_GAIN_BITS.load(core::sync::atomic::Ordering::Relaxed));
        odp.post_gain = f32::from_bits(POST_GAIN_BITS.load(core::sync::atomic::Ordering::Relaxed));

        if let Err(e) = sai_rx.read(&mut process_buf).await {
            error!("SAI RX Glitch: {:?}", defmt::Debug2Format(&e));
        }
        let start = Instant::now();
        dsp::prep_for_processing(&process_buf, &mut filtered_buffer);       
        fir_filter.process(&mut filtered_buffer, &mut sine_buffer);
        dsp::overdrive(&mut sine_buffer, &odp);
        dsp::prep_for_i2s(&mut process_buf, &sine_buffer);
        let elapsed = start.elapsed();
        loop_counter += 1;
        if loop_counter >= 48 {
            info!("DSP processing took: {} us", elapsed.as_micros());       
            loop_counter = 0;
        }
        if let Err(e) = sai_tx.write(&process_buf).await {
            error!("DMA Audio Glitch: {:?}", defmt::Debug2Format(&e));      
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    
    //8mhz external clock on the debugger
    config.rcc.hse = Some(embassy_stm32::rcc::Hse {
        freq: mhz(8),
        mode: embassy_stm32::rcc::HseMode::Bypass, //stlink clock
    });
    config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_P;
    config.rcc.pll_src = embassy_stm32::rcc::PllSource::HSE;

    //main cpu pll
    config.rcc.pll = Some(embassy_stm32::rcc::Pll {
        prediv: embassy_stm32::rcc::PllPreDiv::DIV4,
        mul: embassy_stm32::rcc::PllMul::MUL216,
        divp: Some(embassy_stm32::rcc::PllPDiv::DIV2),
        divq: Some(embassy_stm32::rcc::PllQDiv::DIV9),
        divr: None,
    });

    config.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV1;
    config.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::DIV4;
    config.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::DIV2;

    config.rcc.plli2s = Some(embassy_stm32::rcc::Pll {
        prediv: embassy_stm32::rcc::PllPreDiv::DIV4,
        mul: embassy_stm32::rcc::PllMul::MUL172,
        divp: None,
        divq: Some(embassy_stm32::rcc::PllQDiv::DIV7),
        divr: None,
    });

    config.rcc.mux.sai1sel = embassy_stm32::rcc::mux::Saisel::PLLI2S1_Q;        

    let p = embassy_stm32::init(config);

    //enable cpu caches for massive dsp speedups
    unsafe {
        let mut cp = cortex_m::Peripherals::steal();
        cp.SCB.enable_icache();
        cp.SCB.enable_dcache(&mut cp.CPUID);
    }

    use embassy_stm32::adc::Adc;
    let mut adc = Adc::new(p.ADC3);
    let mut pot_gain = p.PA3;
    let mut pot_thresh = p.PC0;

    let (sub_block_tx, sub_block_rx) = sai::split_subblocks(p.SAI1);

    let mut tx_config = Config::default();
    tx_config.mode = sai::Mode::Master;
    tx_config.tx_rx = sai::TxRx::Transmitter;
    tx_config.data_size = sai::DataSize::Data32;

    tx_config.stereo_mono = sai::StereoMono::Stereo;
    tx_config.bit_order = sai::BitOrder::MsbFirst;

    tx_config.frame_sync_polarity = sai::FrameSyncPolarity::ActiveLow;
    tx_config.frame_sync_offset = sai::FrameSyncOffset::BeforeFirstBit;
    tx_config.clock_strobe = sai::ClockStrobe::Falling;

    tx_config.frame_length = 64; //32 bits left + 32 bits right
    tx_config.frame_sync_active_level_length = embassy_stm32::sai::word::U7(32);

    tx_config.master_clock_divider = embassy_stm32::sai::MasterClockDivider::DIV4;

    let ring_buffer = unsafe { &mut *core::ptr::addr_of_mut!(DMA_RING_BUFFER) };
    let sai_tx = Sai::new_asynchronous_with_mclk(
        sub_block_tx,
        p.PE5,
        p.PE6,
        p.PE4,
        p.PE2,
        p.DMA2_CH1,
        ring_buffer,
        Irqs,
        tx_config
    );

    let mut rx_config = tx_config.clone();
    rx_config.tx_rx = sai::TxRx::Receiver;
    rx_config.data_size = sai::DataSize::Data32;
    rx_config.sync_input = sai::SyncInput::Internal; //sync internally
    rx_config.bit_order = sai::BitOrder::MsbFirst;
    rx_config.stereo_mono = sai::StereoMono::Stereo;
    let rx_ring_buffer = unsafe { &mut *core::ptr::addr_of_mut!(DMA_RX_RING_BUFFER) };

    let sai_rx = Sai::new_synchronous( 
        sub_block_rx,
        p.PE3,
        p.DMA2_CH4,
        rx_ring_buffer,
        Irqs,
        rx_config
    );

    //start high priority executor
    interrupt::CEC.set_priority(Priority::P6); //high priority, exti/etc can be lower
    let spawner_high = EXECUTOR_HIGH.start(interrupt::CEC);

    spawner_high.spawn(audio_task(sai_rx, sai_tx).unwrap());

    //main ui/adc loop runs on lowest priority thread executor
    let mut smoothed_gain = adc.blocking_read(&mut pot_gain, embassy_stm32::adc::SampleTime::CYCLES480) as f32;
    let mut smoothed_thresh = adc.blocking_read(&mut pot_thresh, embassy_stm32::adc::SampleTime::CYCLES480) as f32;
    let alpha = 0.1; //smoothing factor: lower means smoother but slightly more delay

    let mut print_counter = 0;

    loop {
        let gain_raw = adc.blocking_read(&mut pot_gain, embassy_stm32::adc::SampleTime::CYCLES480) as f32;
        let thresh_raw = adc.blocking_read(&mut pot_thresh, embassy_stm32::adc::SampleTime::CYCLES480) as f32;

        smoothed_gain += (gain_raw - smoothed_gain) * alpha;
        smoothed_thresh += (thresh_raw - smoothed_thresh) * alpha;

        let pre_gain = 1.0 + (smoothed_gain / 4095.0) * 9.0;
        let post_gain = smoothed_thresh / 4095.0;

        PRE_GAIN_BITS.store(pre_gain.to_bits(), core::sync::atomic::Ordering::Relaxed);
        POST_GAIN_BITS.store(post_gain.to_bits(), core::sync::atomic::Ordering::Relaxed);

        print_counter += 1;
        if print_counter >= 25 {
            info!("Pot1 (Gain): {}, Pot2 (Vol): {}", gain_raw, thresh_raw);
            print_counter = 0;
        }

        embassy_time::Timer::after_millis(20).await;
    }
}