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


#[unsafe(link_section = ".sram1_bss")]
static mut DMA_RING_BUFFER: [u32; 2048] = [0; 2048];
#[unsafe(link_section = ".sram1_bss")]
static mut DMA_RX_RING_BUFFER: [u32; 2048] = [0; 2048];
//force dma buffers in accessible ram
#[unsafe(link_section = ".sram1_bss")]
static mut BUF_A: [u32; 2048] = [0; 2048];
#[unsafe(link_section = ".sram1_bss")]
static mut BUF_B: [u32; 2048] = [0; 2048];




#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    
    // 1. Switch to the Rock-Solid 8 MHz External Crystal
    config.rcc.hse = Some(embassy_stm32::rcc::Hse {
        freq: mhz(8),
        mode: embassy_stm32::rcc::HseMode::Bypass, //nucleos get their clock from the st-link!
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

    let p = embassy_stm32::init(config);

    // 3. The 48kHz Precision Audio PLL Override
    unsafe {
        let rcc_cr = 0x4002_3800 as *mut u32;
        let rcc_plli2scfgr = 0x4002_3884 as *mut u32;
        let rcc_dckcfgr1 = 0x4002_388C as *mut u32;

        //n = 258 (bits 14:6), q = 7 (bits 27:24)
        core::ptr::write_volatile(rcc_plli2scfgr, (258 << 6) | (7 << 24));

        //turn on plli2s
        let mut cr = core::ptr::read_volatile(rcc_cr);
        cr |= 1 << 26;
        core::ptr::write_volatile(rcc_cr, cr);

        //wait for quartz crystal lock
        while core::ptr::read_volatile(rcc_cr) & (1 << 27) == 0 {}

        //route to sai1
        let mut dck = core::ptr::read_volatile(rcc_dckcfgr1);
        dck &= !(0b11 << 20);
        dck |= (0b01 << 20);
        core::ptr::write_volatile(rcc_dckcfgr1, dck);
    }
    let toggler = Input::new(p.PC13, Pull::None);

    let (sub_block_tx, sub_block_rx) = sai::split_subblocks(p.SAI1);

    let mut tx_config = Config::default();
    tx_config.mode = sai::Mode::Master;
    tx_config.tx_rx = sai::TxRx::Transmitter;
    tx_config.data_size = sai::DataSize::Data32;

    tx_config.data_size = sai::DataSize::Data32;
    tx_config.stereo_mono = sai::StereoMono::Stereo;
    tx_config.bit_order = sai::BitOrder::MsbFirst;

    tx_config.frame_sync_polarity = sai::FrameSyncPolarity::ActiveLow; 
    tx_config.frame_sync_offset = sai::FrameSyncOffset::BeforeFirstBit;
    tx_config.clock_strobe = sai::ClockStrobe::Falling;

    tx_config.frame_length = 64; //32 bits left + 32 bits right
    tx_config.frame_sync_active_level_length = embassy_stm32::sai::word::U7(32);

    tx_config.master_clock_divider = embassy_stm32::sai::MasterClockDivider::Div6;

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

    let mut rx_config = Config::default();
    rx_config.tx_rx = sai::TxRx::Receiver;
    rx_config.data_size = sai::DataSize::Data32;
    rx_config.sync_input = sai::SyncInput::Internal; //sync to the transmitter
    rx_config.bit_order = sai::BitOrder::MsbFirst;
    rx_config.stereo_mono = sai::StereoMono::Stereo;
    let rx_ring_buffer = unsafe { &mut *core::ptr::addr_of_mut!(DMA_RX_RING_BUFFER) };

    let mut sai_rx = Sai::new_asynchronous_with_mclk(
        sub_block_rx,
        p.PF8,
        p.PE3,
        p.PF9,
        p.PF7,
        p.DMA2_CH4,
        rx_ring_buffer,
        rx_config
    );

    let mut phase: f32 = 0.0;
    let mut phase_step: f32 = 2.0*PI*480.0 /48000.0;
    let mut sine_buffer = [0f32; 1024];
    let mut filtered_buffer = [0f32; 1024];
    let mut fir_filter = dsp::AntiAliasFir::new(dsp::FIR_COEFFS);

    let mut odp = dsp::overdrive_params::default();
    let mut sampler = dsp::sampler::new(AUDIO_CLIP);
    
    info!("Init successful!");

    let buf_a_ref = unsafe { &mut *core::ptr::addr_of_mut!(BUF_A) };
    let buf_b_ref = unsafe { &mut *core::ptr::addr_of_mut!(BUF_B) };

    sampler.fill_buffer(&mut sine_buffer);
    dsp::prep_for_i2s(buf_a_ref, &sine_buffer);
    let mut loop_counter = 0;
    loop {
        let (tx_result, _) = join(
            sai_tx.write(buf_a_ref),
            async {
                let start = Instant::now();
                //sampler.fill_buffer(&mut sine_buffer);
                //dsp::fill_sine_buffer(&mut sine_buffer, &mut phase, phase_step);
                sai_rx.read(buf_b_ref).await.unwrap();
                dsp::prep_for_processing(buf_b_ref, &mut sine_buffer);
                fir_filter.process(&sine_buffer, &mut filtered_buffer);
                if toggler.is_high() {
                    dsp::overdrive(&mut filtered_buffer, &odp);
                }
                dsp::prep_for_i2s(buf_b_ref, &filtered_buffer);

                let elapsed = start.elapsed();

                loop_counter += 1;
                if loop_counter >= 48 {
                    info!("DSP processing took: {} us", elapsed.as_micros());
                    loop_counter = 0; 
                }
            }
        ).await;
        if let Err(e) = tx_result {
            error!("DMA Audio Glitch: {:?}", defmt::Debug2Format(&e));
        }

        let (tx_result, _) = join(
            sai_tx.write(buf_b_ref),
            async {
                let start = Instant::now();
                //sampler.fill_buffer(&mut sine_buffer);
                //dsp::fill_sine_buffer(&mut sine_buffer, &mut phase, phase_step);
                sai_rx.read(buf_a_ref).await.unwrap();
                dsp::prep_for_processing(buf_a_ref, &mut sine_buffer);
                fir_filter.process(&sine_buffer, &mut filtered_buffer);
                if toggler.is_high() {
                    dsp::overdrive(&mut filtered_buffer, &odp);
                }
                dsp::prep_for_i2s(buf_a_ref, &filtered_buffer);

                let elapsed = start.elapsed();

                loop_counter += 1;
                if loop_counter >= 48 {
                    info!("DSP processing took: {} us", elapsed.as_micros());
                    loop_counter = 0; 
                }
            }
        ).await;
        if let Err(e) = tx_result {
            error!("DMA Audio Glitch: {:?}", defmt::Debug2Format(&e));
        }

        if toggler.is_low() {
            odp.pre_gain = 3.0;
        } else {
            odp.pre_gain = 1.0;
        }
    }
}
