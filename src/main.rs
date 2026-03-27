#![no_main]
#![no_std]

use embassy_executor::Spawner;
use embassy_stm32::sai::*;
use embassy_stm32::sai;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::{bind_interrupts, dma, peripherals, time::mhz};
use embassy_time::Instant;
use embassy_futures::join::join;
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

    
    let toggler = Input::new(p.PC13, Pull::Down);
    

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

    tx_config.master_clock_divider = embassy_stm32::sai::MasterClockDivider::Div4;

    let ring_buffer = unsafe { &mut *core::ptr::addr_of_mut!(DMA_RING_BUFFER) };
    let mut sai_tx = Sai::new_asynchronous_with_mclk(
        sub_block_tx,
        p.PE5,
        p.PE6,
        p.PE4,
        p.PE2,
        p.DMA2_CH1,
        ring_buffer,
        tx_config
    );

    let mut rx_config = tx_config.clone();
    rx_config.tx_rx = sai::TxRx::Receiver;
    rx_config.data_size = sai::DataSize::Data32;
    rx_config.sync_input = sai::SyncInput::Internal; //sync internally
    rx_config.bit_order = sai::BitOrder::MsbFirst;
    rx_config.stereo_mono = sai::StereoMono::Stereo;
    let rx_ring_buffer = unsafe { &mut *core::ptr::addr_of_mut!(DMA_RX_RING_BUFFER) };

    let mut sai_rx = Sai::new_synchronous( //this is why companies pay for this shit ig, the damn peripherals are amazing
        sub_block_rx,
        p.PE3,
        p.DMA2_CH4,
        rx_ring_buffer,
        rx_config
    );

    let mut phase: f32 = 0.0;
    let mut phase_step: f32 = 2.0*PI*480.0 /48000.0;
    let mut sine_buffer = [0f32; 1024];
    let mut filtered_buffer = [0f32; 1024];
    //let mut fir_filter = dsp::AntiAliasFir::new(dsp::FIR_COEFFS);
    let mut fir_filter = dsp::CmsisFir::new();
    let mut odp = dsp::overdrive_params::default();
    let mut sampler = dsp::sampler::new(AUDIO_CLIP);
    
    info!("Init successful!");
    let mut process_buf = [0u32; 2048];
    
    let mut loop_counter = 0;
    sai_rx.start().unwrap();
    sai_tx.write(&process_buf).await.unwrap(); //according to embassy documentation, we need to start write first
    sai_tx.write(&process_buf).await.unwrap();
    //this loop double buffers even though it doesn't look like it, the peripheral throws an interrupt when its done with half the buffer(i think)
    loop {
        if toggler.is_high() {
            odp.pre_gain = 3.0;
        } else {
            odp.pre_gain = 1.0;
        }

        sai_rx.read(&mut process_buf).await.unwrap(); 
        let start = Instant::now();
        dsp::prep_for_processing(&process_buf, &mut filtered_buffer);
        fir_filter.process(&mut filtered_buffer, &mut sine_buffer);
        //dsp::overdrive(&mut sine_buffer, &odp);
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
