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

#[unsafe(link_section = ".sram1_bss")]
static mut DMA_BT_RX_RING_BUFFER: [u32; 4096] = [0; 4096];

//bt audio chunk shared between bt_task (writer) and audio_task (reader)
//ordering is gated by BT_FRESH, bt_task writes ~every 21ms, audio reads in <2ms
#[unsafe(link_section = ".sram1_bss")]
static mut BT_F32_BUFFER: [f32; 1024] = [0.0; 1024];

//audio-control atomics published by the UI loop, consumed by the audio task.
static VOLUME_BITS: AtomicU32 = AtomicU32::new(1.0_f32.to_bits());
static PRE_GAIN_BITS: AtomicU32 = AtomicU32::new(1.0_f32.to_bits());
static OVERDRIVE_ACTIVE: AtomicBool = AtomicBool::new(false);
static BT_MIX_BITS: AtomicU32 = AtomicU32::new(0.0_f32.to_bits());
static BT_ACTIVE: AtomicBool = AtomicBool::new(false);
static BT_FRESH: AtomicBool = AtomicBool::new(false);

//iir cutoffs/damp published by UI, consumed by audio_task
static HPF_CUTOFF_BITS: AtomicU32 = AtomicU32::new(120.0_f32.to_bits());
static LPF_CUTOFF_BITS: AtomicU32 = AtomicU32::new(6000.0_f32.to_bits());
static LPF_DAMP_BITS: AtomicU32 = AtomicU32::new(0.707_f32.to_bits());

//noise gate params + enable
static NG_THRESH_BITS:  AtomicU32 = AtomicU32::new(0.01_f32.to_bits());   //linear amplitude
static NG_ATTACK_BITS:  AtomicU32 = AtomicU32::new(10.0_f32.to_bits());   //ms
static NG_RELEASE_BITS: AtomicU32 = AtomicU32::new(100.0_f32.to_bits());  //ms
static NG_HOLD_BITS:    AtomicU32 = AtomicU32::new(50.0_f32.to_bits());   //ms
static NOISEGATE_ACTIVE: AtomicBool = AtomicBool::new(false);

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn CEC() {
    unsafe { EXECUTOR_HIGH.on_interrupt() };
}

bind_interrupts!(struct Irqs {
    DMA2_STREAM1 => dma::InterruptHandler<peripherals::DMA2_CH1>;
    DMA2_STREAM4 => dma::InterruptHandler<peripherals::DMA2_CH4>;
    DMA2_STREAM5 => dma::InterruptHandler<peripherals::DMA2_CH5>;
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
    let mut hpf = dsp::IIRHpf1::new(48_000.0, 120.0);
    let mut lpf = dsp::IIRLpf1::new(48_000.0, 6_000.0, 0.707);
    let mut ng = dsp::NoiseGate::new(48_000.0, 0.01, 10.0, 100.0, 50.0);

    info!("Audio task init complete");
    let mut process_buf = [0u32; 2048];
    let mut loop_counter = 0;

    sai_rx.start().unwrap();
    sai_tx.write(&process_buf).await.unwrap();
    sai_tx.write(&process_buf).await.unwrap();

    loop {
        let overdrive_on = OVERDRIVE_ACTIVE.load(Ordering::Relaxed);
        let bt_on = BT_ACTIVE.load(Ordering::Relaxed);
        let volume = f32::from_bits(VOLUME_BITS.load(Ordering::Relaxed));
        let pre_gain = f32::from_bits(PRE_GAIN_BITS.load(Ordering::Relaxed));
        let bt_mix = f32::from_bits(BT_MIX_BITS.load(Ordering::Relaxed));
        let hpf_hz = f32::from_bits(HPF_CUTOFF_BITS.load(Ordering::Relaxed));
        let lpf_hz = f32::from_bits(LPF_CUTOFF_BITS.load(Ordering::Relaxed));
        let lpf_damp = f32::from_bits(LPF_DAMP_BITS.load(Ordering::Relaxed));
        hpf.set_cutoff(hpf_hz);
        lpf.set_cutoff_damp(lpf_hz, lpf_damp);

        let ng_on = NOISEGATE_ACTIVE.load(Ordering::Relaxed);
        let ng_thresh  = f32::from_bits(NG_THRESH_BITS.load(Ordering::Relaxed));
        let ng_attack  = f32::from_bits(NG_ATTACK_BITS.load(Ordering::Relaxed));
        let ng_release = f32::from_bits(NG_RELEASE_BITS.load(Ordering::Relaxed));
        let ng_hold    = f32::from_bits(NG_HOLD_BITS.load(Ordering::Relaxed));
        ng.set_threshold(ng_thresh);
        ng.set_hold(ng_hold);
        ng.set_times(48_000.0, ng_attack, ng_release);

        if let Err(e) = sai_rx.read(&mut process_buf).await {
            error!("SAI RX Glitch: {:?}", defmt::Debug2Format(&e));
        }
        let start = Instant::now();

        //both branches leave the result in final_buf so we can mix + prep once
        let final_buf: &mut [f32; 1024] = if overdrive_on {
            dsp::prep_for_processing(&process_buf, &mut input_buffer);
            if ng_on { ng.process(&mut input_buffer); }
            fir_filter.process(&mut input_buffer, &mut filtered_buffer);
            hpf.process(&mut filtered_buffer);
            odp.pre_gain = pre_gain;
            odp.post_gain = volume;
            dsp::overdrive(&mut filtered_buffer, &odp);
            lpf.process(&mut filtered_buffer);
            &mut filtered_buffer
        } else {
            dsp::prep_for_processing(&process_buf, &mut output_buffer);
            if ng_on { ng.process(&mut output_buffer); }
            for s in output_buffer.iter_mut() {
                *s *= volume;
            }
            &mut output_buffer
        };

        //bt joins at the tail, only the master volume touches it, no fir/overdrive
        if bt_on && BT_FRESH.swap(false, Ordering::Acquire) {
            let bt_gain = bt_mix * volume;
            unsafe {
                let bt = &*core::ptr::addr_of!(BT_F32_BUFFER);
                for (d, b) in final_buf.iter_mut().zip(bt.iter()) {
                    *d = (*d + *b * bt_gain).clamp(-1.0, 1.0);
                }
            }
        }

        dsp::prep_for_i2s(&mut process_buf, final_buf);

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

//pulls from sai2 rx, publishes a fresh chunk only when bt mix is active
//embassy sai has no stop() so we drain the dma either way to avoid overrun
#[embassy_executor::task]
async fn bt_task(mut bt_rx: Sai<'static, peripherals::SAI2, u32>) {
    let mut u32_buf = [0u32; 2048];
    bt_rx.start().unwrap();
    info!("BT task init complete");
    loop {
        if let Err(e) = bt_rx.read(&mut u32_buf).await {
            error!("BT SAI RX Glitch: {:?}", defmt::Debug2Format(&e));
            continue;
        }
        if !BT_ACTIVE.load(Ordering::Relaxed) {
            continue;
        }
        //mark stale, write, then mark fresh so the reader never sees a torn buffer
        BT_FRESH.store(false, Ordering::Release);
        unsafe {
            let buf = &mut *core::ptr::addr_of_mut!(BT_F32_BUFFER);
            for (chunk, sample) in u32_buf.chunks(2).zip(buf.iter_mut()) {
                let signed = chunk[0] as i32;
                *sample = (signed as f32) / 2147483648.0;
            }
        }
        BT_FRESH.store(true, Ordering::Release);
    }
}

//tracks a param that shares a physical pot with other pages.
//on page change, the pickup is locked: the stored audio value holds steady until
//the user twists the pot close enough to the stored value, then it unlocks and tracks.
struct PotPickup {
    stored: f32,
    locked: bool,
}

impl PotPickup {
    fn new(initial: f32) -> Self {
        Self { stored: initial, locked: false }
    }

    fn lock(&mut self) {
        self.locked = true;
    }

    //epsilon = how close (in param units) the pot must be to the stored value to unlock.
    fn update(&mut self, pot_value: f32, epsilon: f32) {
        if self.locked {
            if (pot_value - self.stored).abs() < epsilon {
                self.locked = false;
                self.stored = pot_value;
            }
        } else {
            self.stored = pot_value;
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
    config.rcc.mux.sai2sel = embassy_stm32::rcc::mux::Saisel::PLLI2S1_Q;

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
        p.DMA2_CH5,
        rx_ring_buffer,
        Irqs,
        rx_config,
    );

    let (sub_block_bt, sub_block_spdif) = sai::split_subblocks(p.SAI2);
    let mut bt_rx_config = rx_config.clone();
    bt_rx_config.sync_input = sai::SyncInput::None;
    let bt_rx_ring_buffer = unsafe {&mut *core::ptr::addr_of_mut!(DMA_BT_RX_RING_BUFFER)};
    let bt_rx = Sai::new_asynchronous_with_mclk(
        sub_block_bt,
        p.PD13,
        p.PD11,
        p.PD12,
        p.PE0,
        p.DMA2_CH4,
        bt_rx_ring_buffer,
        Irqs,
        bt_rx_config,
    );

    //spawn audio + bt tasks on high-priority interrupt executor
    interrupt::CEC.set_priority(Priority::P6);
    let spawner_high = EXECUTOR_HIGH.start(interrupt::CEC);
    spawner_high.spawn(audio_task(sai_rx, sai_tx).unwrap());
    spawner_high.spawn(bt_task(bt_rx).unwrap());

    let mut adc = Adc::new(p.ADC3);
    let mut pot_pa3 = p.PA3; //pre-gain pot (drive)
    let mut pot_pc0 = p.PC0; //volume pot
    let mut pot_pc3 = p.PC3; //hpf cutoff
    let mut pot_pf3 = p.PF3; //lpf cutoff
    let mut pot_pf5 = p.PF5; //lpf damp
    let mut adc_pdw = Output::new(p.PA7, Level::High, Speed::Low);
    adc_pdw.set_high();

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

    const QEI_PER_DETENT: i32 = 2;

    //pot smoothing
    let alpha = 0.2f32;
    let mut sm_vol  = adc.blocking_read(&mut pot_pc0, SampleTime::CYCLES480) as f32;
    let mut sm_pre  = adc.blocking_read(&mut pot_pa3, SampleTime::CYCLES480) as f32;
    let mut sm_hpf  = adc.blocking_read(&mut pot_pc3, SampleTime::CYCLES480) as f32;
    let mut sm_lpf  = adc.blocking_read(&mut pot_pf3, SampleTime::CYCLES480) as f32;
    let mut sm_damp = adc.blocking_read(&mut pot_pf5, SampleTime::CYCLES480) as f32;

    //last-displayed value, Schmitt hysteresis kills boundary flicker.
    //init to 0 so the first loop iteration crosses the threshold and locks on.
    let mut disp_hpf  = 0.0_f32;
    let mut disp_lpf  = 0.0_f32;
    let mut disp_damp = 0.0_f32;

    //per-param pickup state. defaults match the audio_task initial values so the
    //audio stays consistent until the user actually starts twisting.
    let mut pp_pregain = PotPickup::new(1.0);    //1..10
    let mut pp_hpf     = PotPickup::new(120.0);  //20..500 Hz
    let mut pp_lpf     = PotPickup::new(6000.0); //1k..12k Hz
    let mut pp_damp    = PotPickup::new(0.707);  //0.3..1.5
    let mut pp_thresh  = PotPickup::new(0.01);   //0.001..0.05
    let mut pp_attack  = PotPickup::new(10.0);   //1..50 ms
    let mut pp_release = PotPickup::new(100.0);  //10..500 ms
    let mut pp_hold    = PotPickup::new(50.0);   //0..200 ms
    let mut pp_bt_mix  = PotPickup::new(0.0);    //0..1

    let mut last_page: usize = ui.current_page();


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

        //detect page change -> lock pickups for the new page so the audio holds
        //its stored value until the user twists the pot close to it.
        let page = ui.current_page();
        if page != last_page {
            match page {
                ui_core::OVERDRIVE_PAGE => {
                    pp_pregain.lock(); pp_hpf.lock(); pp_lpf.lock(); pp_damp.lock();
                }
                ui_core::NOISEGATE_PAGE => {
                    pp_thresh.lock(); pp_attack.lock(); pp_release.lock(); pp_hold.lock();
                }
                ui_core::BLUETOOTH_PAGE => {
                    pp_bt_mix.lock();
                }
                _ => {}
            }
            last_page = page;
        }

        //pots — EMA smoothing on the raw count, normalize at the end
        let raw_vol  = adc.blocking_read(&mut pot_pc0, SampleTime::CYCLES480) as f32;
        let raw_pre  = adc.blocking_read(&mut pot_pa3, SampleTime::CYCLES480) as f32;
        let raw_hpf  = adc.blocking_read(&mut pot_pc3, SampleTime::CYCLES480) as f32;
        let raw_lpf  = adc.blocking_read(&mut pot_pf3, SampleTime::CYCLES480) as f32;
        let raw_damp = adc.blocking_read(&mut pot_pf5, SampleTime::CYCLES480) as f32;
        sm_vol  += (raw_vol  - sm_vol)  * alpha;
        sm_pre  += (raw_pre  - sm_pre)  * alpha;
        sm_hpf  += (raw_hpf  - sm_hpf)  * alpha;
        sm_lpf  += (raw_lpf  - sm_lpf)  * alpha;
        sm_damp += (raw_damp - sm_damp) * alpha;

        let vol    = (sm_vol  / 4095.0).clamp(0.0, 1.0);
        let n_pre  = (sm_pre  / 4095.0).clamp(0.0, 1.0);
        let n_hpf  = (sm_hpf  / 4095.0).clamp(0.0, 1.0);
        let n_lpf  = (sm_lpf  / 4095.0).clamp(0.0, 1.0);
        let n_damp = (sm_damp / 4095.0).clamp(0.0, 1.0);

        //candidate pot-implied values for every param. cheap math, compute always.
        let pregain_pot = 1.0    + n_pre  * 9.0;     //PA3 -> 1..10 gain
        let hpf_pot     = 20.0   + n_hpf  * 480.0;   //PC3 -> 20..500 Hz
        let lpf_pot     = 1000.0 + n_lpf  * 11000.0; //PF3 -> 1k..12k Hz
        let damp_pot    = 0.3    + n_damp * 1.2;     //PF5 -> 0.3..1.5
        let thresh_pot  = 0.001  + n_pre  * 0.024;   //PA3 -> 0.001..0.025 amplitude (finer at low end)
        let attack_pot  = 1.0    + n_hpf  * 49.0;    //PC3 -> 1..50 ms
        let release_pot = 10.0   + n_lpf  * 490.0;   //PF3 -> 10..500 ms
        let hold_pot    =          n_damp * 200.0;   //PF5 -> 0..200 ms
        let bt_mix_pot  = n_pre;                     //PA3 -> 0..1

        //update only the pickups whose params live on the current page.
        //epsilon ~ 5% of each param's range so pickup engages with a small twist.
        match page {
            ui_core::OVERDRIVE_PAGE => {
                pp_pregain.update(pregain_pot, 0.5);
                pp_hpf.update(hpf_pot, 25.0);
                pp_lpf.update(lpf_pot, 550.0);
                pp_damp.update(damp_pot, 0.06);
            }
            ui_core::NOISEGATE_PAGE => {
                pp_thresh.update(thresh_pot, 0.0012);
                pp_attack.update(attack_pot, 2.5);
                pp_release.update(release_pot, 25.0);
                pp_hold.update(hold_pot, 10.0);
            }
            ui_core::BLUETOOTH_PAGE => {
                pp_bt_mix.update(bt_mix_pot, 0.05);
            }
            _ => {}
        }

        //publish all atomics from stored values, audio always sees the latest
        //regardless of which page the user is currently viewing.
        VOLUME_BITS.store(vol.to_bits(), Ordering::Relaxed);
        PRE_GAIN_BITS.store(pp_pregain.stored.to_bits(), Ordering::Relaxed);
        HPF_CUTOFF_BITS.store(pp_hpf.stored.to_bits(), Ordering::Relaxed);
        LPF_CUTOFF_BITS.store(pp_lpf.stored.to_bits(), Ordering::Relaxed);
        LPF_DAMP_BITS.store(pp_damp.stored.to_bits(), Ordering::Relaxed);
        NG_THRESH_BITS.store(pp_thresh.stored.to_bits(), Ordering::Relaxed);
        NG_ATTACK_BITS.store(pp_attack.stored.to_bits(), Ordering::Relaxed);
        NG_RELEASE_BITS.store(pp_release.stored.to_bits(), Ordering::Relaxed);
        NG_HOLD_BITS.store(pp_hold.stored.to_bits(), Ordering::Relaxed);
        BT_MIX_BITS.store(pp_bt_mix.stored.to_bits(), Ordering::Relaxed);

        OVERDRIVE_ACTIVE.store(ui.audio_active(ui_core::OVERDRIVE_PAGE), Ordering::Relaxed);
        NOISEGATE_ACTIVE.store(ui.audio_active(ui_core::NOISEGATE_PAGE), Ordering::Relaxed);
        BT_ACTIVE.store(ui.audio_active(ui_core::BLUETOOTH_PAGE), Ordering::Relaxed);

        //display values: volume always; the rest depends on current page.
        //OD HPF/LPF/Damp keep Schmitt hysteresis; NG params have coarse-enough
        //quantization that noise doesn't cross the bucket boundaries.
        //lock flags drive the on-screen '*' marker per row.
        //precision defaults to 2 decimals; overridden per row where a different
        //resolution is more useful (e.g. NG threshold needs 4).
        ui.pots[0] = vol;
        ui.locked = [false; ui_core::NUM_POTS];
        ui.precision = [2; ui_core::NUM_POTS];
        match page {
            ui_core::OVERDRIVE_PAGE => {
                let hpf_q  = ((pp_hpf.stored + 0.5) as i32) as f32;
                let lpf_q  = (((pp_lpf.stored / 10.0) + 0.5) as i32 * 10) as f32;
                let damp_q = (((pp_damp.stored * 20.0) + 0.5) as i32) as f32 / 20.0;
                if (pp_hpf.stored  - disp_hpf ).abs() > 0.7   { disp_hpf  = hpf_q;  }
                if (pp_lpf.stored  - disp_lpf ).abs() > 7.0   { disp_lpf  = lpf_q;  }
                if (pp_damp.stored - disp_damp).abs() > 0.035 { disp_damp = damp_q; }
                ui.pots[1] = pp_pregain.stored;
                ui.pots[2] = disp_hpf;
                ui.pots[3] = disp_lpf;
                ui.pots[4] = disp_damp;
                ui.locked[1] = pp_pregain.locked;
                ui.locked[2] = pp_hpf.locked;
                ui.locked[3] = pp_lpf.locked;
                ui.locked[4] = pp_damp.locked;
            }
            ui_core::NOISEGATE_PAGE => {
                //quantize to: 0.0001 amp, 1 ms attack, 10 ms release, 5 ms hold
                ui.pots[1] = (((pp_thresh.stored  * 10000.0) + 0.5) as i32) as f32 / 10000.0;
                ui.pots[2] = ((pp_attack.stored + 0.5) as i32) as f32;
                ui.pots[3] = (((pp_release.stored / 10.0) + 0.5) as i32 * 10) as f32;
                ui.pots[4] = (((pp_hold.stored    / 5.0)  + 0.5) as i32 *  5) as f32;
                ui.locked[1] = pp_thresh.locked;
                ui.locked[2] = pp_attack.locked;
                ui.locked[3] = pp_release.locked;
                ui.locked[4] = pp_hold.locked;
                //threshold needs 4 decimals so values like 0.0050 are readable
                ui.precision[1] = 4;
            }
            ui_core::BLUETOOTH_PAGE => {
                ui.pots[1] = pp_bt_mix.stored;
                ui.locked[1] = pp_bt_mix.locked;
            }
            _ => {}
        }

        //render the UI to the framebuffer and push to the OLED
        display.clear();
        ui.draw(&mut display).unwrap();
        let _ = display.flush().await;

        Timer::after_millis(20).await;
    }
}
