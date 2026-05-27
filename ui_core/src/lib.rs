#![no_std]

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};
use heapless::String;

pub const SCREEN_W: u32 = 128;
pub const SCREEN_H: u32 = 64;
pub const NUM_POTS: usize = 5;
pub const MAX_EFFECTS: usize = 8;

const HEADER_LINE_Y: i32 = 11;
const ROW_TOP: i32 = 13;
const ROW_H: i32 = 10;
const CHAR_W: i32 = 6;

const VOLUME_LABEL: &str = "Volume";

pub struct EffectDef {
    pub name: &'static str,
    ///names for pots[1..5]. Empty string = unused row.
    pub params: [&'static str; NUM_POTS - 1],
}

pub const EFFECTS: &[EffectDef] = &[
    EffectDef { name: "Bypass",     params: ["", "", "", ""] },
    EffectDef { name: "Noise Gate", params: ["Thresh", "Att ms", "Rel ms", "Hold ms"] },
    EffectDef { name: "Overdrive",  params: ["Pre-Gain", "HPF Hz", "LPF Hz", "Damp"] },
    EffectDef { name: "Bluetooth",  params: ["Mix", "", "", ""] },
];

pub const BYPASS_PAGE: usize = 0;
pub const NOISEGATE_PAGE: usize = 1;
pub const OVERDRIVE_PAGE: usize = 2;
pub const BLUETOOTH_PAGE: usize = 3;

const MENU_PAGE: usize = EFFECTS.len();
const TOTAL_PAGES: usize = EFFECTS.len() + 1;

pub enum Event {
    Rotate(i8),
    ClickPress,
    ClickRelease,
}

pub struct UiState {
    page: usize,
    enabled: [bool; MAX_EFFECTS],
    pub pots: [f32; NUM_POTS],
    //true for any pot whose physical position doesn't match the stored audio
    //value (pickup locked), renderer shows a marker so the user knows the
    //knob is currently inert until they sweep through the stored value.
    pub locked: [bool; NUM_POTS],
    //decimal places shown for each pot's value. defaults to 2; tune for params
    //whose useful range needs more (e.g. noise gate threshold) or fewer digits.
    pub precision: [u8; NUM_POTS],
}

impl UiState {
    pub fn new() -> Self {
        Self {
            page: 0,
            enabled: [false; MAX_EFFECTS],
            pots: [0.0; NUM_POTS],
            locked: [false; NUM_POTS],
            precision: [2; NUM_POTS],
        }
    }

    pub fn is_menu(&self) -> bool {
        self.page == MENU_PAGE
    }

    pub fn current_page(&self) -> usize {
        self.page
    }

    pub fn current_effect(&self) -> Option<&'static EffectDef> {
        if self.is_menu() { None } else { Some(&EFFECTS[self.page]) }
    }

    ///true iff bypass is enabled AND we're looking at a non-bypass effect page.
    pub fn locked_by_bypass(&self) -> bool {
        !self.is_menu() && self.page != BYPASS_PAGE && self.enabled[BYPASS_PAGE]
    }

    pub fn is_current_enabled(&self) -> bool {
        !self.is_menu() && self.enabled[self.page]
    }

    ///raw enabled flag for a given effect page.
    pub fn effect_enabled(&self, idx: usize) -> bool {
        self.enabled[idx]
    }

    ///true iff the effect is enabled AND not overridden by bypass.
    ///use this from the audio task to decide whether to apply the effect.
    pub fn audio_active(&self, idx: usize) -> bool {
        if idx == BYPASS_PAGE {
            self.enabled[BYPASS_PAGE]
        } else {
            self.enabled[idx] && !self.enabled[BYPASS_PAGE]
        }
    }

    pub fn handle(&mut self, evt: Event) {
        match evt {
            Event::Rotate(delta) => {
                let n = TOTAL_PAGES as i32;
                let mut p = self.page as i32 + delta as i32;
                if p < 0 { p = 0; }
                if p >= n { p = n - 1; }
                self.page = p as usize;
            }
            Event::ClickPress => {
                if self.is_menu() || self.locked_by_bypass() {
                    return;
                }
                self.enabled[self.page] = !self.enabled[self.page];
            }
            Event::ClickRelease => {}
        }
    }

    fn draw_header<D: DrawTarget<Color = BinaryColor>>(
        &self,
        target: &mut D,
        title: &str,
    ) -> Result<(), D::Error> {
        let text = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        let text_inv = MonoTextStyle::new(&FONT_6X10, BinaryColor::Off);
        let line = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
        let fill = PrimitiveStyle::with_fill(BinaryColor::On);

        Text::with_baseline(title, Point::new(0, 0), text, Baseline::Top).draw(target)?;

        let mut cursor_x = SCREEN_W as i32;

        //right-most slot: status badge
        if !self.is_menu() {
            if self.locked_by_bypass() {
                //no box; plain "BYP" text differentiates from ON/OFF
                let badge = "BYP";
                let bw = badge.len() as i32 * CHAR_W;
                cursor_x -= bw;
                Text::with_baseline(badge, Point::new(cursor_x, 0), text, Baseline::Top)
                    .draw(target)?;
            } else {
                let on = self.is_current_enabled();
                let badge = if on { "ON" } else { "OFF" };
                let bw = badge.len() as i32 * CHAR_W + 4;
                cursor_x -= bw;
                if on {
                    Rectangle::new(Point::new(cursor_x, 0), Size::new(bw as u32, 10))
                        .into_styled(fill)
                        .draw(target)?;
                    Text::with_baseline(
                        badge,
                        Point::new(cursor_x + 2, 0),
                        text_inv,
                        Baseline::Top,
                    )
                    .draw(target)?;
                } else {
                    Rectangle::new(Point::new(cursor_x, 0), Size::new(bw as u32, 10))
                        .into_styled(line)
                        .draw(target)?;
                    Text::with_baseline(
                        badge,
                        Point::new(cursor_x + 2, 0),
                        text,
                        Baseline::Top,
                    )
                    .draw(target)?;
                }
            }
        }

        //x/y counter to the left of the badge slot
        let mut counter: String<8> = String::new();
        let _ = write!(counter, "{}/{}", self.page + 1, TOTAL_PAGES);
        let cw = counter.len() as i32 * CHAR_W;
        cursor_x -= cw + 4;
        Text::with_baseline(&counter, Point::new(cursor_x, 0), text, Baseline::Top)
            .draw(target)?;

        Line::new(Point::new(0, HEADER_LINE_Y), Point::new(127, HEADER_LINE_Y))
            .into_styled(line)
            .draw(target)?;

        Ok(())
    }

    fn draw_param_row<D: DrawTarget<Color = BinaryColor>>(
        &self,
        target: &mut D,
        row: usize,
        label: &str,
        value: f32,
    ) -> Result<(), D::Error> {
        let text = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        let y = ROW_TOP + (row as i32) * ROW_H;

        //leading char is space normally, '*' when the pot is locked by pickup.
        //same width either way so the row text doesn't shift in/out of lock.
        let lock_mark = if self.locked[row] { '*' } else { ' ' };
        let mut left: String<16> = String::new();
        let _ = write!(left, "{}{} {}", lock_mark, row + 1, label);
        Text::with_baseline(&left, Point::new(0, y), text, Baseline::Top).draw(target)?;

        //per-row precision lets each param choose its decimal count (e.g. NG
        //threshold needs 4 digits to be useful in the 0..0.025 range).
        let mut val: String<12> = String::new();
        let _ = write!(val, "{:.*}", self.precision[row] as usize, value);
        let vw = val.len() as i32 * CHAR_W;
        Text::with_baseline(&val, Point::new(SCREEN_W as i32 - vw, y), text, Baseline::Top)
            .draw(target)?;
        Ok(())
    }

    fn draw_effect_page<D: DrawTarget<Color = BinaryColor>>(
        &self,
        target: &mut D,
        eff: &EffectDef,
    ) -> Result<(), D::Error> {
        //row 1: Volume (pot 0) — always present
        self.draw_param_row(target, 0, VOLUME_LABEL, self.pots[0])?;

        //rows 2..5: effect-specific params on pots[1..5]
        for i in 0..(NUM_POTS - 1) {
            let name = eff.params[i];
            if name.is_empty() {
                continue;
            }
            self.draw_param_row(target, i + 1, name, self.pots[i + 1])?;
        }
        Ok(())
    }

    fn draw_menu_page<D: DrawTarget<Color = BinaryColor>>(
        &self,
        target: &mut D,
    ) -> Result<(), D::Error> {
        let text = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        let items = ["Brightness", "Save Preset", "About"];
        for (i, item) in items.iter().enumerate() {
            let y = ROW_TOP + (i as i32) * ROW_H;
            Text::with_baseline(item, Point::new(2, y), text, Baseline::Top).draw(target)?;
        }
        let hint = "(stub)";
        let hw = hint.len() as i32 * CHAR_W;
        let hx = (SCREEN_W as i32 - hw) / 2;
        Text::with_baseline(hint, Point::new(hx, 53), text, Baseline::Top).draw(target)?;
        Ok(())
    }

    pub fn draw<D: DrawTarget<Color = BinaryColor>>(&self, target: &mut D) -> Result<(), D::Error> {
        target.clear(BinaryColor::Off)?;

        if let Some(eff) = self.current_effect() {
            self.draw_header(target, eff.name)?;
            self.draw_effect_page(target, eff)?;
        } else {
            self.draw_header(target, "Menu")?;
            self.draw_menu_page(target)?;
        }

        Ok(())
    }
}

impl Default for UiState {
    fn default() -> Self {
        Self::new()
    }
}
