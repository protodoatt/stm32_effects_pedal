use embedded_graphics::{Pixel, pixelcolor::BinaryColor, prelude::*};
use minifb::{Key, KeyRepeat, Window, WindowOptions};
use ui_core::{Event, SCREEN_H, SCREEN_W, UiState};

const SCALE: usize = 5;
const W: usize = SCREEN_W as usize;
const H: usize = SCREEN_H as usize;
const WIN_W: usize = W * SCALE;
const WIN_H: usize = H * SCALE;

const PIXEL_ON: u32 = 0x00FF_FFFF;
const PIXEL_OFF: u32 = 0x0000_0000;

struct Sim {
    pixels: [bool; W * H],
}

impl Sim {
    fn new() -> Self {
        Self { pixels: [false; W * H] }
    }

    fn blit(&self, buf: &mut [u32]) {
        for y in 0..H {
            for x in 0..W {
                let color = if self.pixels[y * W + x] { PIXEL_ON } else { PIXEL_OFF };
                for sy in 0..SCALE {
                    let row = (y * SCALE + sy) * WIN_W;
                    for sx in 0..SCALE {
                        buf[row + x * SCALE + sx] = color;
                    }
                }
            }
        }
    }
}

impl OriginDimensions for Sim {
    fn size(&self) -> Size {
        Size::new(SCREEN_W, SCREEN_H)
    }
}

impl DrawTarget for Sim {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(p, c) in pixels {
            if p.x >= 0 && p.x < W as i32 && p.y >= 0 && p.y < H as i32 {
                self.pixels[p.y as usize * W + p.x as usize] = c.is_on();
            }
        }
        Ok(())
    }
}

fn snapshot(state: &UiState, name: &str) {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../snaps");
    std::fs::create_dir_all(dir).ok();
    let path = format!("{dir}/{name}");
    let mut sim = Sim::new();
    state.draw(&mut sim).unwrap();
    let mut bytes = vec![0u8; W * H];
    for y in 0..H {
        for x in 0..W {
            bytes[y * W + x] = if sim.pixels[y * W + x] { 255 } else { 0 };
        }
    }
    let file = std::fs::File::create(&path).expect("create png");
    let w = std::io::BufWriter::new(file);
    let mut encoder = png::Encoder::new(w, W as u32, H as u32);
    encoder.set_color(png::ColorType::Grayscale);
    encoder.set_depth(png::BitDepth::Eight);
    encoder
        .write_header()
        .unwrap()
        .write_image_data(&bytes)
        .unwrap();
    println!("wrote {path}");
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.get(1).map(String::as_str) == Some("snap") {
        let mut s = UiState::new();
        s.pots = [0.65, 0.40, 0.30, 0.55, 0.20];

        // Page 1: Bypass OFF
        snapshot(&s, "01_bypass_off.png");
        // Page 2: Overdrive OFF (bypass still off)
        s.handle(Event::Rotate(1));
        snapshot(&s, "02_overdrive_off.png");
        // Click → Overdrive ON
        s.handle(Event::ClickPress);
        s.handle(Event::ClickRelease);
        snapshot(&s, "03_overdrive_on.png");
        // Move to Delay, turn it on
        s.handle(Event::Rotate(1));
        s.handle(Event::ClickPress);
        s.handle(Event::ClickRelease);
        snapshot(&s, "04_delay_on.png");
        // Go back to Bypass and turn it ON — locks other effects
        s.handle(Event::Rotate(-2));
        s.handle(Event::ClickPress);
        s.handle(Event::ClickRelease);
        snapshot(&s, "05_bypass_on.png");
        // Visit Overdrive — should now show BYP, click should do nothing
        s.handle(Event::Rotate(1));
        snapshot(&s, "06_overdrive_locked.png");
        s.handle(Event::ClickPress);
        s.handle(Event::ClickRelease);
        snapshot(&s, "07_overdrive_still_locked.png");
        // Menu page
        s.handle(Event::Rotate(2));
        snapshot(&s, "08_menu.png");
        return;
    }

    let mut state = UiState::new();
    let mut sim = Sim::new();
    let mut framebuffer = vec![0u32; WIN_W * WIN_H];

    let mut window = Window::new(
        "ui_sim - SSD1306 128x64  |  Left/Right: rotate  Space: click  Esc: quit",
        WIN_W,
        WIN_H,
        WindowOptions::default(),
    )
    .expect("create window");
    window.set_target_fps(60);

    let mut t: f32 = 0.0;

    while window.is_open() && !window.is_key_down(Key::Escape) {
        t += 0.016;
        for (i, p) in state.pots.iter_mut().enumerate() {
            let phase = t + (i as f32) * 1.3;
            *p = (0.5 + 0.5 * phase.sin()).clamp(0.0, 1.0);
        }

        if window.is_key_pressed(Key::Right, KeyRepeat::Yes) {
            state.handle(Event::Rotate(1));
        }
        if window.is_key_pressed(Key::Left, KeyRepeat::Yes) {
            state.handle(Event::Rotate(-1));
        }
        if window.is_key_pressed(Key::Space, KeyRepeat::No) {
            state.handle(Event::ClickPress);
            state.handle(Event::ClickRelease);
        }

        state.draw(&mut sim).unwrap();
        sim.blit(&mut framebuffer);
        window.update_with_buffer(&framebuffer, WIN_W, WIN_H).unwrap();
    }
}
