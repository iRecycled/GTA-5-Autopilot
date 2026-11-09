//! The surround part of the HUD (D8): what the core says the driver should
//! see and hear for BSD, RCTA and park assist. The adapter only draws.
//!
//! - **BSD:** a mirror icon at each screen edge, amber while that side is
//!   occupied; flashing red with a chime when signalling into it.
//! - **RCTA:** a red arrow from the side the traffic comes from, a chime,
//!   and REAR BRAKING when it brakes.
//! - **Park assist:** a small top view of the car with six segments per
//!   bumper (green, then amber below 1 m, red below 0.4 m), the nearest
//!   distance, and beeps at the rate the core asks for.

use adapter_api::{Hud, surround_flags as sf, wall_time_ns};
use shv_sys::invoke;

use crate::{draw_text, n};

#[derive(Default)]
pub struct Sounds {
    last_beep: u64,
    bsd_warn: bool,
    rcta: bool,
}

fn chime() {
    let () = unsafe { invoke!(n::PLAY_SOUND_FRONTEND, -1i32, c"ERROR", c"HUD_FRONTEND_DEFAULT_SOUNDSET", true) };
}

fn beep() {
    let () = unsafe { invoke!(n::PLAY_SOUND_FRONTEND, -1i32, c"NAV_UP_DOWN", c"HUD_FRONTEND_DEFAULT_SOUNDSET", true) };
}

unsafe fn rect(x: f32, y: f32, w: f32, h: f32, c: (i32, i32, i32), a: i32) {
    let () = unsafe { invoke!(n::DRAW_RECT, x, y, w, h, c.0, c.1, c.2, a, false) };
}

const AMBER: (i32, i32, i32) = (255, 170, 0);
const RED: (i32, i32, i32) = (230, 40, 30);
const GREY: (i32, i32, i32) = (90, 90, 90);

fn park_colour(d: f32) -> (i32, i32, i32) {
    if d < 0.4 {
        RED
    } else if d < 1.0 {
        AMBER
    } else {
        (80, 200, 90)
    }
}

/// Draws the surround HUD and plays its sounds. `live`: the HUD is fresh.
pub fn draw(h: &Hud, live: bool, frame: i32, s: &mut Sounds) {
    if !live {
        *s = Sounds { last_beep: s.last_beep, ..Default::default() };
        return;
    }
    let f = h.surround;
    let flash = (frame / 8) % 2 == 0;
    unsafe {
        // BSD: mirror icons at the screen edges.
        for (left, occupied, warn) in [(true, f & sf::BSD_LEFT != 0, f & sf::BSD_WARN_LEFT != 0), (false, f & sf::BSD_RIGHT != 0, f & sf::BSD_WARN_RIGHT != 0)] {
            let x = if left { 0.03 } else { 0.97 };
            if warn && flash {
                rect(x, 0.45, 0.03, 0.06, RED, 230);
            } else if occupied {
                rect(x, 0.45, 0.03, 0.06, AMBER, 220);
            } else if f & sf::BSD_ACTIVE != 0 {
                rect(x, 0.45, 0.03, 0.06, GREY, 90);
            }
        }
        let warn = f & (sf::BSD_WARN_LEFT | sf::BSD_WARN_RIGHT) != 0;
        if warn && !s.bsd_warn {
            chime();
        }
        s.bsd_warn = warn;

        // RCTA: an arrow from the side the traffic comes from.
        let cross = f & (sf::RCTA_LEFT | sf::RCTA_RIGHT) != 0;
        if cross && !s.rcta {
            chime();
        }
        s.rcta = cross;
        if f & sf::RCTA_LEFT != 0 && flash {
            draw_text(0.33, 0.80, ">> CROSS TRAFFIC", (255, 90, 70));
        }
        if f & sf::RCTA_RIGHT != 0 && flash {
            draw_text(0.56, 0.80, "CROSS TRAFFIC <<", (255, 90, 70));
        }
        if f & (sf::RCTA_BRAKE | sf::PARK_BRAKE) != 0 {
            rect(0.5, 0.75, 0.2, 0.04, RED, 210);
            draw_text(0.43, 0.735, if f & sf::RCTA_BRAKE != 0 { "REAR BRAKING" } else { "PARK ASSIST BRAKING" }, (255, 255, 255));
        }

        // Park assist: a top view of the car with its sensor segments.
        if f & sf::PARK_ACTIVE != 0 {
            let (cx, cy) = (0.90, 0.72);
            rect(cx, cy, 0.034, 0.11, (40, 40, 40), 200);
            rect(cx, cy, 0.03, 0.105, (200, 200, 200), 160);
            for i in 0..12 {
                let d = h.park[i];
                if d.is_nan() {
                    continue;
                }
                let k = (i % 6) as f32;
                let x = cx - 0.025 + k * 0.01;
                let front = i < 6;
                // Segments sit closer to the car the nearer the obstacle.
                let reach = if d.is_finite() { d.clamp(0.2, 2.0) / 2.0 } else { 1.0 };
                let y = if front { cy - 0.06 - 0.03 * reach } else { cy + 0.06 + 0.03 * reach };
                let c = if d.is_finite() { park_colour(d) } else { GREY };
                rect(x, y, 0.009, 0.008, c, if d.is_finite() { 230 } else { 80 });
            }
            let near = h.park.iter().copied().filter(|d| d.is_finite()).fold(f32::INFINITY, f32::min);
            if near.is_finite() {
                draw_text(cx - 0.02, cy + 0.11, &format!("{near:.2} m"), (255, 255, 255));
            }
            // The tone.
            let now = wall_time_ns();
            let period = if h.park_tone.is_infinite() { 100_000_000 } else if h.park_tone > 0.0 { (1e9 / h.park_tone as f64) as u64 } else { 0 };
            if period > 0 && now.saturating_sub(s.last_beep) >= period {
                beep();
                s.last_beep = now;
            }
        }

        // Unavailable features (D21).
        let mut off = Vec::new();
        if f & sf::BSD_OFF != 0 {
            off.push("BSD");
        }
        if f & sf::RCTA_OFF != 0 {
            off.push("RCTA");
        }
        if f & sf::PARK_OFF != 0 {
            off.push("PARK");
        }
        if !off.is_empty() {
            draw_text(0.80, 0.88, &format!("{} OFF", off.join(" / ")), (255, 170, 60));
        }
    }
}
