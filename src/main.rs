#![no_main]
#![no_std]

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::sai;
use embassy_stm32::sai::*;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::timer::qei::{self, Qei};
use embedded_hal_bus::spi::ExclusiveDevice;
use embassy_stm32::{bind_interrupts, dma, peripherals, time::mhz};
use embassy_time::{Instant, Timer};
use defmt::{error, info};

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use display_interface_spi::SPIInterface;
use embassy_time::Delay;
use oled_async::{Builder, displays::sh1106::Sh1106_128_64, prelude::*};

use {defmt_rtt as _, panic_probe as _};

mod dsp;

//dma buffers, we gotta force them into sram1
#[unsafe(link_section = ".sram1_bss")]
static mut DMA_RING_BUFFER: [u32; 4096] = [0; 4096];
#[unsafe(link_section = ".sram1_bss")]
static mut DMA_RX_RING_BUFFER: [u32; 4096] = [0; 4096];

//audio-control atomics published by the UI loop, consumed by the audio task.
static VOLUME_BITS: AtomicU32 = AtomicU32::new(0x3f800000); //1.0
static PRE_GAIN_BITS: AtomicU32 = AtomicU32::new(0x3f800000); //1.0
static OVERDRIVE_ACTIVE: AtomicBool = AtomicBool::new(false);

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn CEC() {
    unsafe { EXECUTOR_HIGH.on_interrupt() };
}

bind_interrupts!(struct Irqs {
    DMA2_STREAM1 => dma::InterruptHandler<peripherals::DMA2_CH1>;
    DMA2_STREAM4 => dma::InterruptHandler<peripherals::DMA2_CH4>;
    DMA2_STREAM6 => dma::InterruptHandler<peripherals::DMA2_CH6>;
});

#[embassy_executor::task]
async fn audio_task(
    mut sai_rx: Sai<'static, peripherals::SAI1, u32>,
    mut sai_tx: Sai<'static, peripherals::SAI1, u32>,
) {
    let mut input_buffer = [0f32; 1024];
    let mut filtered_buffer = [0f32; 1024];
    let mut output_buffer = [0f32; 1024];
    let mut fir_filter = dsp::CmsisFir::new();
    fir_filter.init();
    let mut odp = dsp::overdrive_params::default();

    info!("Audio task init complete");
    let mut process_buf = [0u32; 2048];
    let mut loop_counter = 0;

    sai_rx.start().unwrap();
    sai_tx.write(&process_buf).await.unwrap();
    sai_tx.write(&process_buf).await.unwrap();

    loop {
        let overdrive_on = OVERDRIVE_ACTIVE.load(Ordering::Relaxed);
        let volume = f32::from_bits(VOLUME_BITS.load(Ordering::Relaxed));
        let pre_gain = f32::from_bits(PRE_GAIN_BITS.load(Ordering::Relaxed));

        if let Err(e) = sai_rx.read(&mut process_buf).await {
            error!("SAI RX Glitch: {:?}", defmt::Debug2Format(&e));
        }
        let start = Instant::now();

        if overdrive_on {
            dsp::prep_for_processing(&process_buf, &mut input_buffer);
            fir_filter.process(&mut input_buffer, &mut filtered_buffer);
            odp.pre_gain = pre_gain;
            odp.post_gain = volume;
            dsp::overdrive(&mut filtered_buffer, &odp);
            dsp::prep_for_i2s(&mut process_buf, &filtered_buffer);
        } else {
            dsp::prep_for_processing(&process_buf, &mut output_buffer);
            for s in output_buffer.iter_mut() {
                *s *= volume;
            }
            dsp::prep_for_i2s(&mut process_buf, &output_buffer);
        }

        let elapsed = start.elapsed();
        loop_counter += 1;
        if loop_counter >= 48 {
            info!("DSP {} us  od={}", elapsed.as_micros(), overdrive_on);
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

    config.rcc.hse = Some(embassy_stm32::rcc::Hse {
        freq: mhz(8),
        mode: embassy_stm32::rcc::HseMode::Bypass,
    });
    config.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_P;
    config.rcc.pll_src = embassy_stm32::rcc::PllSource::HSE;

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

    //this makes it fast af but could break shi
    unsafe {
        let mut cp = cortex_m::Peripherals::steal();
        cp.SCB.enable_icache();
        cp.SCB.enable_dcache(&mut cp.CPUID);
    }

    //sai setup
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
    tx_config.frame_length = 64;
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
        tx_config,
    );

    let mut rx_config = tx_config.clone();
    rx_config.tx_rx = sai::TxRx::Receiver;
    rx_config.sync_input = sai::SyncInput::Internal;

    let rx_ring_buffer = unsafe { &mut *core::ptr::addr_of_mut!(DMA_RX_RING_BUFFER) };
    let sai_rx = Sai::new_synchronous(
        sub_block_rx,
        p.PE3,
        p.DMA2_CH4,
        rx_ring_buffer,
        Irqs,
        rx_config,
    );

    //spawn audio task on high-priority interrupt executor
    interrupt::CEC.set_priority(Priority::P6);
    let spawner_high = EXECUTOR_HIGH.start(interrupt::CEC);
    spawner_high.spawn(audio_task(sai_rx, sai_tx).unwrap());

    let mut adc = Adc::new(p.ADC3);
    let mut pot_pa3 = p.PA3; //pre-Gain pot (drive)
    let mut pot_pc0 = p.PC0; //volume pot

    //spi5 to oled
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = mhz(4);
    let spi = Spi::new_txonly(p.SPI5, p.PF7, p.PF9, p.DMA2_CH6, Irqs, spi_config);
    let cs = Output::new(p.PF8, Level::High, Speed::VeryHigh);
    let dc = Output::new(p.PF10, Level::High, Speed::VeryHigh);
    let mut oled_rst = Output::new(p.PG1, Level::High, Speed::VeryHigh);

    let spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();
    let di = SPIInterface::new(spi_dev, dc);

    let raw_disp = Builder::new(Sh1106_128_64 {}).connect(di);
    let mut display: GraphicsMode<_, _> = raw_disp.into();

    display.reset(&mut oled_rst, &mut Delay).unwrap();
    display.init().await.unwrap();
    display.clear();
    display.flush().await.unwrap();

    //rotary encoder on TIM3 CH1/CH2 + push button on PG3 (active low)
    let qei = Qei::new(p.TIM3, p.PC6, p.PC7, qei::Config::default());
    let button = Input::new(p.PG3, Pull::Up);

    info!("UI peripherals init complete");

    //ui loop
    let mut ui = ui_core::UiState::new();
    let mut last_qei: u16 = qei.count() as u16;
    let mut qei_accum: i32 = 0;
    let mut last_btn_pressed = false;

    const QEI_PER_DETENT: i32 = 4;

    //pot smoothing
    let alpha = 0.2f32;
    let mut sm_vol = adc.blocking_read(&mut pot_pc0, SampleTime::CYCLES480) as f32;
    let mut sm_pre = adc.blocking_read(&mut pot_pa3, SampleTime::CYCLES480) as f32;


    loop {
        //encoder delta
        let now = qei.count() as u16;
        let raw_delta = now.wrapping_sub(last_qei) as i16 as i32;
        last_qei = now;
        qei_accum += raw_delta;
        while qei_accum >= QEI_PER_DETENT {
            ui.handle(ui_core::Event::Rotate(1));
            qei_accum -= QEI_PER_DETENT;
        }
        while qei_accum <= -QEI_PER_DETENT {
            ui.handle(ui_core::Event::Rotate(-1));
            qei_accum += QEI_PER_DETENT;
        }

        //active low button, we check for falling edge
        let now_pressed = button.is_low();
        if now_pressed && !last_btn_pressed {
            ui.handle(ui_core::Event::ClickPress);
        } else if !now_pressed && last_btn_pressed {
            ui.handle(ui_core::Event::ClickRelease);
        }
        last_btn_pressed = now_pressed;

        //pots — EMA smoothing on the raw count, normalize at the end
        let raw_vol = adc.blocking_read(&mut pot_pc0, SampleTime::CYCLES480) as f32;
        let raw_pre = adc.blocking_read(&mut pot_pa3, SampleTime::CYCLES480) as f32;
        sm_vol += (raw_vol - sm_vol) * alpha;
        sm_pre += (raw_pre - sm_pre) * alpha;

        let vol = (sm_vol / 4095.0).clamp(0.0, 1.0);
        let pre = (sm_pre / 4095.0).clamp(0.0, 1.0);
        ui.pots[0] = vol;
        ui.pots[1] = pre;

        //publish to audio task
        VOLUME_BITS.store(vol.to_bits(), Ordering::Relaxed);
        let pre_gain = 1.0 + pre * 9.0; //0..1 -> 1..10
        PRE_GAIN_BITS.store(pre_gain.to_bits(), Ordering::Relaxed);
        OVERDRIVE_ACTIVE.store(ui.audio_active(ui_core::OVERDRIVE_PAGE), Ordering::Relaxed);

        //render the UI to the framebuffer and push to the OLED
        display.clear();
        ui.draw(&mut display).unwrap();
        let _ = display.flush().await;

        Timer::after_millis(20).await;
    }
}
