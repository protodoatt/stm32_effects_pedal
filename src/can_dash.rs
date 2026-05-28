use core::sync::atomic::{AtomicU32, Ordering};
use defmt::info;
use embassy_stm32::bind_interrupts;
use embassy_stm32::can::{
    self, BufferedCan, Can, Fifo, Frame, RxBuf, StandardId, TxBuf,
    filter::Mask32,
};
use embassy_stm32::{Peri, peripherals};
use embassy_time::{Duration, Timer};

pub static VU_LEVEL_BITS: AtomicU32 = AtomicU32::new(0.0_f32.to_bits());

bind_interrupts!(pub struct CanIrqs {
    CAN1_RX0 => can::Rx0InterruptHandler<peripherals::CAN1>;
    CAN1_RX1 => can::Rx1InterruptHandler<peripherals::CAN1>;
    CAN1_SCE => can::SceInterruptHandler<peripherals::CAN1>;
    CAN1_TX  => can::TxInterruptHandler<peripherals::CAN1>;
});

//peak DBC bit-counts at vu=1.0, tune per gauge sweep.
//  0x280 RPM: 16-bit LE, bytes 2..=3, 0.25 RPM/bit
//  0x1A0 ABS: 15-bit LE, start bit 17, 0.01 km/h/bit
//  0x5A0 spd: 15-bit LE, start bit 9 (bytes 1..=2!), 0.002 unit/bit
const RPM_MAX:       f32 = 26_000.0;
const SPEED_MAX_1A0: f32 = 20_000.0;
const SPEED_MAX_5A0: f32 = 28_000.0;

const BRIGHTNESS:   u8 = 0x32;
const BRIGHTVALUE:  u8 = 1;     //wake on, brights off
const SIGNAL_STATE: u8 = 0x00;
const ABS_BYTE:     u8 = 0x88;  //ABS+ESP+handbrake all off

const TX_BUF_LEN: usize = 32;
const RX_BUF_LEN: usize = 4;
static mut TX_BUF: TxBuf<TX_BUF_LEN> = TxBuf::new();
static mut RX_BUF: RxBuf<RX_BUF_LEN> = RxBuf::new();

#[embassy_executor::task]
pub async fn can_task(
    can_peri: Peri<'static, peripherals::CAN1>,
    rx_pin:   Peri<'static, peripherals::PB8>,
    tx_pin:   Peri<'static, peripherals::PB9>,
) {
    let mut raw = Can::new(can_peri, rx_pin, tx_pin, CanIrqs);
    raw.set_bitrate(500_000);
    raw.modify_filters()
        .enable_bank(0, Fifo::Fifo0, Mask32::accept_all());
    raw.enable().await;
    info!("CAN1 dashboard task up");

    let tx_buf = unsafe { &mut *core::ptr::addr_of_mut!(TX_BUF) };
    let rx_buf = unsafe { &mut *core::ptr::addr_of_mut!(RX_BUF) };
    let mut can: BufferedCan<'_, TX_BUF_LEN, RX_BUF_LEN> = raw.buffered(tx_buf, rx_buf);

    let mut slow: u8 = 0;

    loop {
        Timer::after(Duration::from_millis(20)).await;

        let vu = f32::from_bits(VU_LEVEL_BITS.load(Ordering::Relaxed)).clamp(0.0, 1.0);
        let rpm     = (vu * RPM_MAX)         as u16;
        let spd_1a0 = ((vu * SPEED_MAX_1A0)  as u16) & 0x7FFF;
        let spd_5a0 = ((vu * SPEED_MAX_5A0)  as u16) & 0x7FFF;

        let rpm_lo  = (rpm & 0xFF) as u8;
        let rpm_hi  = (rpm >> 8)   as u8;
        //15-bit at start bit 17: byte 2 bit 0 reserved, value shifted left by 1.
        let s1a0_b2 = ((spd_1a0 & 0x7F) << 1) as u8;
        let s1a0_b3 = (spd_1a0 >> 7)          as u8;
        //15-bit at start bit 9: byte 1 bit 0 reserved, value shifted left by 1.
        let s5a0_b1 = ((spd_5a0 & 0x7F) << 1) as u8;
        let s5a0_b2 = (spd_5a0 >> 7)          as u8;

        send(&mut can, 0x575, &[0x07, 0x00]).await;
        send(&mut can, 0x480, &[0; 8]).await;
        send(&mut can, 0x1A0, &[0, ABS_BYTE, s1a0_b2, s1a0_b3, 0, 0, 0, 0]).await;
        send(&mut can, 0x280, &[0, 0,        rpm_lo,  rpm_hi,  0, 0, 0, 0]).await;
        send(&mut can, 0x5A0, &[0, s5a0_b1,  s5a0_b2, 0,       0, 0, 0, 0]).await;
        send(&mut can, 0x3D0, &[0; 8]).await;

        slow = slow.wrapping_add(1);
        if slow >= 10 {
            slow = 0;
            send(&mut can, 0x470, &[0, 0, BRIGHTNESS, 0, 0, 0, 0, 0]).await;
            send(&mut can, 0x531, &[BRIGHTVALUE, 0, SIGNAL_STATE, 0, 0, 0, 0, 0]).await;
            send(&mut can, 0x050, &[0x04, 0, 0, 0, 0, 0, 0, 0]).await;
            send(&mut can, 0x288, &[0, 0x5B, 0, 0, 0, 0, 0, 0]).await;  //coolant ~warmed up
            send(&mut can, 0x635, &[0x64, 0, 0, 0, 0, 0, 0, 0]).await;
        }
    }
}

async fn send(
    can: &mut BufferedCan<'_, TX_BUF_LEN, RX_BUF_LEN>,
    id: u16,
    bytes: &[u8],
) {
    let Some(sid) = StandardId::new(id) else { return };
    let Ok(frame) = Frame::new_data(sid, bytes) else { return };
    can.write(&frame).await;
}
