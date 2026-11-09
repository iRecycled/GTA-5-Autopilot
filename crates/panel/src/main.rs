//! The engineering panel (D8, PRD "Engineering panel"): what the engineer
//! sees beside the game.
//!
//! - **CAN trace:** every frame on both buses, decoded with the generated
//!   message types; as a scrolling trace or one row per message (count, rate,
//!   latest signals).
//! - **Bus load** per network over the last two minutes.
//! - **Latency:** sample → command and the core's step time, p50/p99/max.
//! - **Vehicle and HUD:** state, speeds as the host has them and as the ADAS
//!   ECU reads them from the bus, lead, TTC.
//! - **Fault injection:** drop, corrupt or delay a message (or a whole bus)
//!   for a while. The core records it, so the drive still replays exactly.
//!
//! It listens for the core's telemetry on UDP port 47300 (`adas-core`, or
//! `replay --udp` to watch a recorded drive).
//!
//! Usage: `adas-panel [--port N] [--trace] [--select BUS:HEXID]
//!                    [--screenshot OUT.png --after SECONDS]`.
//! `--trace` starts on the scrolling trace; `--select chassis:343` opens a
//! message's signals.
//! `--screenshot` saves the window after that long and exits (for docs and
//! checks without a desktop).

use std::collections::{BTreeMap, VecDeque};
use std::time::{Duration, Instant};

use adas_core::msgs::{adas, chassis};
use eframe::egui::{self, Color32, RichText, Sense, Stroke, Vec2};
use telemetry::{BUS_NAMES, CanFrame, Control, Packet, Receiver, Step};

const TRACE_MAX: usize = 4000;
const LOAD_WINDOW_S: f64 = 0.25;
const LOAD_HISTORY_S: f64 = 120.0;
const BUS_COLORS: [Color32; 2] = [Color32::from_rgb(90, 160, 255), Color32::from_rgb(255, 150, 60)];

fn msg_name(bus: u8, id: u32) -> &'static str {
    match bus {
        telemetry::CHASSIS => chassis::name(id),
        _ => adas::name(id),
    }
    .unwrap_or("?")
}

fn decode(bus: u8, id: u32, data: &[u8]) -> Vec<(&'static str, f64)> {
    match bus {
        telemetry::CHASSIS => chassis::decode_signals(id, data),
        _ => adas::decode_signals(id, data),
    }
    .unwrap_or_default()
}

fn messages(bus: u8) -> &'static [(u32, &'static str, usize)] {
    match bus {
        telemetry::CHASSIS => chassis::MESSAGES,
        _ => adas::MESSAGES,
    }
}

fn hex(d: &[u8]) -> String {
    d.iter().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(" ")
}

/// The non-zero signals (counters and checksums left out), at most `max`.
fn signals_text(sig: &[(&str, f64)], max: usize) -> String {
    let shown: Vec<&(&str, f64)> = sig.iter().filter(|(n, v)| *v != 0.0 && *n != "CHECKSUM" && *n != "COUNTER").collect();
    let mut s: Vec<String> = shown.iter().take(max).map(|(n, v)| format!("{n}={}", fmt_num(*v))).collect();
    if shown.len() > max {
        s.push("…".into());
    }
    s.join("  ")
}

fn fmt_num(v: f64) -> String {
    if v.fract() == 0.0 && v.abs() < 1e9 { format!("{v:.0}") } else { format!("{v:.3}") }
}

fn pct(v: &VecDeque<u32>, p: f64) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    let mut s: Vec<u32> = v.iter().copied().collect();
    s.sort_unstable();
    s[((s.len() - 1) as f64 * p).round() as usize] as f64
}

/// One row per message in the by-message view.
struct MsgRow {
    count: u64,
    last_t: f64,
    /// Mean interval between frames (s), smoothed.
    dt: f64,
    data: Vec<u8>,
}

#[derive(Clone, Copy, PartialEq)]
enum TraceMode {
    ByMessage,
    Scroll,
}

struct FaultForm {
    bus: u8,
    id: Option<u32>,
    kind: u8,
    delay_ms: f64,
    p: f64,
    /// s; 0 = until cleared.
    duration: f64,
}

struct App {
    rx: Option<Receiver>,
    error: Option<String>,
    port: u16,
    hello: Option<(String, String, String, String)>,
    last_packet: Option<Instant>,
    packets: u64,
    step: Option<Step>,
    latency: VecDeque<u32>,
    step_ns: VecDeque<u32>,
    /// (bus time, load) per bus.
    load: [VecDeque<(f64, f32)>; 2],
    last_busy: [Option<(f64, f64)>; 2],
    trace: VecDeque<CanFrame>,
    msgs: BTreeMap<(u8, u32), MsgRow>,
    selected: Option<(u8, u32)>,
    paused: bool,
    filter: String,
    show_bus: [bool; 2],
    mode: TraceMode,
    events: VecDeque<String>,
    form: FaultForm,
    sent: Option<String>,
    /// Save the window to this file once this much time has passed, then exit.
    screenshot: Option<(std::path::PathBuf, Instant)>,
}

fn save_png(path: &std::path::Path, img: &egui::ColorImage) -> Result<(), String> {
    let f = std::fs::File::create(path).map_err(|e| e.to_string())?;
    let mut enc = png::Encoder::new(std::io::BufWriter::new(f), img.size[0] as u32, img.size[1] as u32);
    enc.set_color(png::ColorType::Rgba);
    enc.set_depth(png::BitDepth::Eight);
    let mut w = enc.write_header().map_err(|e| e.to_string())?;
    let bytes: Vec<u8> = img.pixels.iter().flat_map(|c| c.to_array()).collect();
    w.write_image_data(&bytes).map_err(|e| e.to_string())
}

impl App {
    fn new(port: u16) -> Self {
        let (rx, error) = match Receiver::bind(port) {
            Ok(r) => (Some(r), None),
            Err(e) => (None, Some(format!("cannot listen on UDP {port}: {e} (is another panel open?)"))),
        };
        Self {
            rx,
            error,
            port,
            hello: None,
            last_packet: None,
            packets: 0,
            step: None,
            latency: VecDeque::new(),
            step_ns: VecDeque::new(),
            load: [VecDeque::new(), VecDeque::new()],
            last_busy: [None, None],
            trace: VecDeque::new(),
            msgs: BTreeMap::new(),
            selected: None,
            paused: false,
            filter: String::new(),
            show_bus: [true, true],
            mode: TraceMode::ByMessage,
            events: VecDeque::new(),
            form: FaultForm { bus: telemetry::ADAS, id: None, kind: 0, delay_ms: 40.0, p: 1.0, duration: 2.0 },
            sent: None,
            screenshot: None,
        }
    }

    fn reset(&mut self) {
        self.load = [VecDeque::new(), VecDeque::new()];
        self.last_busy = [None, None];
        self.trace.clear();
        self.msgs.clear();
        self.latency.clear();
        self.step_ns.clear();
    }

    fn pump(&mut self) {
        let Some(rx) = self.rx.as_mut() else { return };
        let mut got = Vec::new();
        // Bounded, so a flood can't freeze the UI.
        for _ in 0..20_000 {
            match rx.try_recv() {
                Some(p) => got.push(p),
                None => break,
            }
        }
        for p in got {
            self.packets += 1;
            self.last_packet = Some(Instant::now());
            self.on_packet(p);
        }
    }

    fn on_packet(&mut self, p: Packet) {
        match p {
            Packet::Hello { feature, host, vehicle, recording, .. } => self.hello = Some((feature, host, vehicle, recording)),
            Packet::Step(s) => self.on_step(*s),
            Packet::Frames { frames, .. } => {
                for f in frames {
                    self.on_frame(f);
                }
            }
            Packet::Event { text, .. } => {
                self.events.push_back(text);
                while self.events.len() > 200 {
                    self.events.pop_front();
                }
            }
            Packet::Control(_) => {}
        }
    }

    fn on_step(&mut self, s: Step) {
        // A new run (time went backwards): start the histories over.
        if self.step.as_ref().is_some_and(|p| s.t + 1.0 < p.t) {
            self.reset();
        }
        if s.latency_ns > 0 {
            self.latency.push_back(s.latency_ns);
            self.step_ns.push_back(s.step_ns);
            while self.latency.len() > 720 {
                self.latency.pop_front();
                self.step_ns.pop_front();
            }
        }
        for (b, snap) in s.buses.iter().enumerate() {
            match self.last_busy[b] {
                Some((t0, busy0)) if snap.now - t0 >= LOAD_WINDOW_S => {
                    let load = ((snap.busy_total - busy0) / (snap.now - t0)) as f32;
                    self.load[b].push_back((snap.now, load));
                    self.last_busy[b] = Some((snap.now, snap.busy_total));
                    while self.load[b].front().is_some_and(|(t, _)| snap.now - t > LOAD_HISTORY_S) {
                        self.load[b].pop_front();
                    }
                }
                Some(_) => {}
                None => self.last_busy[b] = Some((snap.now, snap.busy_total)),
            }
        }
        self.step = Some(s);
    }

    fn on_frame(&mut self, f: CanFrame) {
        let row = self.msgs.entry((f.bus, f.id)).or_insert(MsgRow { count: 0, last_t: f.t, dt: f64::NAN, data: Vec::new() });
        if row.count > 0 {
            let dt = (f.t - row.last_t).max(0.0);
            row.dt = if row.dt.is_nan() { dt } else { 0.95 * row.dt + 0.05 * dt };
        }
        row.count += 1;
        row.last_t = f.t;
        row.data.clone_from(&f.data);
        if !self.paused {
            self.trace.push_back(f);
            while self.trace.len() > TRACE_MAX {
                self.trace.pop_front();
            }
        }
    }

    fn send(&mut self, c: Control) {
        let Some(rx) = self.rx.as_ref() else { return };
        self.sent = Some(match rx.send_to(&Packet::Control(c), telemetry::CONTROL_PORT) {
            Ok(()) => format!("sent: {c:?}"),
            Err(e) => format!("send failed: {e}"),
        });
    }

    fn filtered(&self, bus: u8, id: u32) -> bool {
        if !self.show_bus[bus as usize & 1] {
            return false;
        }
        if self.filter.is_empty() {
            return true;
        }
        let q = self.filter.to_ascii_uppercase();
        msg_name(bus, id).contains(&q) || format!("{id:X}").contains(&q) || format!("{id}") == q
    }

    // --- Views ---

    fn status_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.strong("ADAS engineering panel");
            ui.separator();
            let live = self.last_packet.is_some_and(|t| t.elapsed() < Duration::from_secs(2));
            if let Some(e) = &self.error {
                ui.colored_label(Color32::LIGHT_RED, e);
            } else if live {
                ui.colored_label(Color32::LIGHT_GREEN, format!("receiving on UDP {}", self.port));
            } else {
                ui.colored_label(Color32::GRAY, format!("waiting for adas-core (UDP {})", self.port));
            }
            if let Some((feature, host, vehicle, rec)) = &self.hello {
                ui.separator();
                ui.label(format!("{feature} · {host} · {vehicle}"));
                if !rec.is_empty() {
                    ui.weak(rec);
                }
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("Reset views").clicked() {
                    self.reset();
                    self.events.clear();
                }
                ui.weak(format!("{} packets", self.packets));
            });
        });
    }

    fn vehicle_view(&self, ui: &mut egui::Ui) {
        let Some(s) = &self.step else {
            ui.weak("no data yet");
            return;
        };
        let state = ["off", "active", "override"].get(s.state as usize).copied().unwrap_or("?");
        egui::Grid::new("vehicle").num_columns(2).striped(true).show(ui, |ui| {
            ui.label("frame / game time");
            ui.monospace(format!("{} / {:.2} s", s.frame, s.t));
            ui.end_row();
            ui.label("state");
            let c = match s.state {
                1 => Color32::LIGHT_GREEN,
                2 => Color32::YELLOW,
                _ => Color32::GRAY,
            };
            ui.colored_label(c, state);
            ui.end_row();
            ui.label("speed host / bus");
            ui.monospace(format!("{:.1} / {:.1} km/h", s.speed * 3.6, s.bus_speed * 3.6));
            ui.end_row();
            ui.label("accel estimate");
            ui.monospace(format!("{:+.2} m/s²", s.accel));
            ui.end_row();
            ui.label("set speed / gap");
            ui.monospace(format!("{:.0} km/h / {:.1} s", s.set_speed * 3.6, s.time_gap));
            ui.end_row();
            ui.label("lead");
            if s.lead_gap.is_finite() {
                ui.monospace(format!("{:.1} m @ {:.1} km/h", s.lead_gap, s.lead_speed * 3.6));
            } else {
                ui.weak("none");
            }
            ui.end_row();
            ui.label("TTC");
            ui.monospace(if s.ttc.is_finite() { format!("{:.2} s", s.ttc) } else { "∞".into() });
            ui.end_row();
            ui.label("command thr / brk");
            ui.monospace(format!("{:.3} / {:.3}", s.throttle, s.brake));
            ui.end_row();
            ui.label("steer cmd / host");
            ui.monospace(format!("{:+.3} / {:+.3} rad", s.steer, s.steer_angle));
            ui.end_row();
            ui.label("driver thr / brk");
            ui.monospace(format!("{:.2} / {:.2}", s.driver_throttle, s.driver_brake));
            ui.end_row();
            ui.label("HUD");
            ui.monospace(hud_text(s.hud_flags));
            ui.end_row();
            if s.hud_message != 0 {
                ui.label("unavailable");
                ui.colored_label(Color32::YELLOW, adapter_text(s.hud_message));
                ui.end_row();
            }
            ui.label("brake gain / AEBs");
            ui.monospace(format!("{:.1} / {}", s.brake_gain, s.aeb_activations));
            ui.end_row();
            ui.label("tracks");
            ui.monospace(format!("{} ({} actors)", s.tracks.len(), s.actors.len()));
            ui.end_row();
        });
    }

    fn latency_view(&self, ui: &mut egui::Ui) {
        if self.latency.is_empty() {
            ui.weak("live core only (a replay has no latency)");
            return;
        }
        egui::Grid::new("latency").num_columns(4).striped(true).show(ui, |ui| {
            ui.label("");
            ui.strong("p50");
            ui.strong("p99");
            ui.strong("max");
            ui.end_row();
            for (name, v) in [("sample to command", &self.latency), ("core step", &self.step_ns)] {
                ui.label(name);
                for p in [0.5, 0.99, 1.0] {
                    ui.monospace(format!("{:.0} µs", pct(v, p) / 1e3));
                }
                ui.end_row();
            }
        });
        ui.weak(format!("last {} steps", self.latency.len()));
    }

    fn bus_view(&self, ui: &mut egui::Ui) {
        let Some(s) = &self.step else { return };
        egui::Grid::new("buses").num_columns(3).striped(true).show(ui, |ui| {
            ui.label("");
            for (b, n) in BUS_NAMES.iter().enumerate() {
                ui.colored_label(BUS_COLORS[b], RichText::new(*n).strong());
            }
            ui.end_row();
            ui.label("bit rate");
            for b in &s.buses {
                ui.monospace(format!("{} kbit/s", b.nominal_bps / 1000));
            }
            ui.end_row();
            ui.label("load (now)");
            for l in &self.load {
                ui.monospace(l.back().map(|(_, v)| format!("{:.1} %", v * 100.0)).unwrap_or("-".into()));
            }
            ui.end_row();
            ui.label("frames delivered");
            for b in &s.buses {
                ui.monospace(format!("{}", b.delivered));
            }
            ui.end_row();
            ui.label("dropped / CRC / delayed");
            for b in &s.buses {
                ui.monospace(format!("{} / {} / {}", b.faulted[0], b.faulted[1], b.faulted[2]));
            }
            ui.end_row();
            ui.label("backlog");
            for b in &s.buses {
                ui.monospace(format!("{}", b.backlog));
            }
            ui.end_row();
        });
        ui.label(format!("checksum errors at the ADAS ECU: {}   radar path: {} frame(s)", s.checksum_errors, s.radar_latency_frames));
    }

    fn load_plot(&self, ui: &mut egui::Ui, height: f32) {
        let (rect, _) = ui.allocate_exact_size(Vec2::new(ui.available_width(), height), Sense::hover());
        let p = ui.painter_at(rect);
        p.rect_filled(rect, 4.0, ui.visuals().extreme_bg_color);
        let t_end = self.load.iter().filter_map(|l| l.back().map(|v| v.0)).fold(f64::NEG_INFINITY, f64::max);
        if !t_end.is_finite() {
            p.text(rect.center(), egui::Align2::CENTER_CENTER, "bus load: no data", egui::FontId::proportional(13.0), Color32::GRAY);
            return;
        }
        let t0 = t_end - LOAD_HISTORY_S;
        let peak = self.load.iter().flat_map(|l| l.iter().map(|v| v.1)).fold(0.0f32, f32::max);
        let y_max = (peak * 1.25).clamp(0.2, 1.5);
        let to_px = |t: f64, v: f32| egui::pos2(rect.left() + ((t - t0) / LOAD_HISTORY_S) as f32 * rect.width(), rect.bottom() - (v / y_max) * rect.height());
        // Grid every 10 %.
        let mut g = 0.1;
        while g < y_max {
            let y = rect.bottom() - (g / y_max) * rect.height();
            p.hline(rect.x_range(), y, Stroke::new(1.0, Color32::from_gray(60)));
            p.text(egui::pos2(rect.left() + 4.0, y - 2.0), egui::Align2::LEFT_BOTTOM, format!("{:.0} %", g * 100.0), egui::FontId::monospace(10.0), Color32::GRAY);
            g += 0.1;
        }
        for (b, l) in self.load.iter().enumerate() {
            let pts: Vec<egui::Pos2> = l.iter().filter(|(t, _)| *t >= t0).map(|(t, v)| to_px(*t, *v)).collect();
            if pts.len() > 1 {
                p.line(pts, Stroke::new(1.5, BUS_COLORS[b]));
            }
            let label = format!("{} {:.1} %", BUS_NAMES[b], l.back().map(|v| v.1 * 100.0).unwrap_or(0.0));
            p.text(egui::pos2(rect.right() - 6.0, rect.top() + 6.0 + 14.0 * b as f32), egui::Align2::RIGHT_TOP, label, egui::FontId::monospace(11.0), BUS_COLORS[b]);
        }
        p.text(egui::pos2(rect.left() + 4.0, rect.bottom() - 2.0), egui::Align2::LEFT_BOTTOM, "-120 s", egui::FontId::monospace(10.0), Color32::GRAY);
    }

    fn fault_view(&mut self, ui: &mut egui::Ui) {
        let f = &mut self.form;
        egui::Grid::new("fault").num_columns(2).show(ui, |ui| {
            ui.label("bus");
            ui.horizontal(|ui| {
                for (b, n) in BUS_NAMES.iter().enumerate() {
                    if ui.selectable_value(&mut f.bus, b as u8, *n).changed() {
                        f.id = None;
                    }
                }
            });
            ui.end_row();
            ui.label("message");
            let current = f.id.map(|id| format!("{} (0x{id:X})", msg_name(f.bus, id))).unwrap_or("every frame".into());
            egui::ComboBox::from_id_salt("fault-msg").width(220.0).selected_text(current).show_ui(ui, |ui| {
                ui.selectable_value(&mut f.id, None, "every frame");
                // Messages seen on this bus first.
                let mut list: Vec<(bool, u32, &str)> = messages(f.bus).iter().map(|m| (!self.msgs.contains_key(&(f.bus, m.0)), m.0, m.1)).collect();
                list.sort();
                for (unseen, id, name) in list {
                    let text = format!("{name} (0x{id:X}){}", if unseen { "" } else { " •" });
                    ui.selectable_value(&mut f.id, Some(id), text);
                }
            });
            ui.end_row();
            ui.label("fault");
            ui.horizontal(|ui| {
                ui.selectable_value(&mut f.kind, 0, "drop");
                ui.selectable_value(&mut f.kind, 1, "CRC error");
                ui.selectable_value(&mut f.kind, 2, "delay");
            });
            ui.end_row();
            if f.kind == 2 {
                ui.label("delay");
                ui.add(egui::Slider::new(&mut f.delay_ms, 1.0..=500.0).logarithmic(true).suffix(" ms"));
                ui.end_row();
            }
            ui.label("probability");
            ui.add(egui::Slider::new(&mut f.p, 0.0..=1.0));
            ui.end_row();
            ui.label("duration");
            ui.add(egui::Slider::new(&mut f.duration, 0.0..=30.0).suffix(" s").text("0 = until cleared"));
            ui.end_row();
        });
        let inject = Control::Inject {
            bus: f.bus,
            id: f.id,
            kind: f.kind,
            delay: f.delay_ms / 1000.0,
            p: f.p,
            duration: if f.duration > 0.0 { f.duration } else { f64::INFINITY },
        };
        let bus = f.bus;
        ui.horizontal(|ui| {
            if ui.button("Inject").clicked() {
                self.send(inject);
            }
            if ui.button(format!("Clear {}", BUS_NAMES[bus as usize])).clicked() {
                self.send(Control::Clear { bus: Some(bus) });
            }
            if ui.button("Clear all").clicked() {
                self.send(Control::Clear { bus: None });
            }
        });
        if let Some(t) = &self.sent {
            ui.weak(t);
        }
        if let Some(s) = &self.step {
            let now = s.t;
            let active: Vec<String> = s
                .buses
                .iter()
                .enumerate()
                .flat_map(|(b, snap)| {
                    snap.faults.iter().filter(move |x| now < x.until).map(move |x| {
                        let what = if x.id == u32::MAX { "every frame".to_string() } else { msg_name(b as u8, x.id).to_string() };
                        let kind = match x.kind {
                            0 => "drop".to_string(),
                            1 => "CRC error".to_string(),
                            _ => format!("delay {:.0} ms", x.delay * 1000.0),
                        };
                        let until = if x.until.is_finite() { format!("{:.1} s left", x.until - now) } else { "until cleared".into() };
                        format!("{}: {what}: {kind}, p {:.2}, {until}", BUS_NAMES[b], x.p)
                    })
                })
                .collect();
            if active.is_empty() {
                ui.weak("no active faults");
            } else {
                for a in active {
                    ui.colored_label(Color32::YELLOW, a);
                }
            }
        }
    }

    fn trace_view(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.mode, TraceMode::ByMessage, "By message");
            ui.selectable_value(&mut self.mode, TraceMode::Scroll, "Trace");
            ui.separator();
            for (b, n) in BUS_NAMES.iter().enumerate() {
                ui.checkbox(&mut self.show_bus[b], *n);
            }
            ui.separator();
            ui.label("filter");
            ui.add(egui::TextEdit::singleline(&mut self.filter).desired_width(140.0).hint_text("name or hex id"));
            if self.mode == TraceMode::Scroll {
                ui.checkbox(&mut self.paused, "pause");
            }
        });
        let row_h = 16.0;
        match self.mode {
            TraceMode::ByMessage => {
                let rows: Vec<(u8, u32)> = self.msgs.keys().copied().filter(|(b, id)| self.filtered(*b, *id)).collect();
                let head = format!("{:<8}{:>5}  {:<22}{:>9}  {:>9}  {}", "bus", "id", "message", "count", "rate", "data");
                ui.label(RichText::new(head).monospace().strong());
                egui::ScrollArea::vertical().id_salt("by-msg").auto_shrink([false, false]).max_height(ui.available_height() * 0.55).show_rows(
                    ui,
                    row_h,
                    rows.len(),
                    |ui, range| {
                        for &(b, id) in &rows[range] {
                            let m = &self.msgs[&(b, id)];
                            let sel = self.selected == Some((b, id));
                            let rate = if m.dt > 0.0 { format!("{:.1} Hz", 1.0 / m.dt) } else { "-".into() };
                            let text = format!("{:<8}{:>5X}  {:<22}{:>9}  {:>9}  {}", BUS_NAMES[b as usize], id, msg_name(b, id), m.count, rate, hex(&m.data));
                            if ui.add(egui::Button::selectable(sel, RichText::new(text).monospace().color(BUS_COLORS[b as usize & 1])).truncate()).clicked() {
                                self.selected = if sel { None } else { Some((b, id)) };
                            }
                        }
                    },
                );
                ui.separator();
                match self.selected.and_then(|k| self.msgs.get(&k).map(|m| (k, m))) {
                    Some(((b, id), m)) => {
                        ui.strong(format!("{} 0x{id:X} on {}", msg_name(b, id), BUS_NAMES[b as usize]));
                        egui::ScrollArea::vertical().id_salt("signals").auto_shrink([false, false]).show(ui, |ui| {
                            egui::Grid::new("signals").num_columns(2).striped(true).show(ui, |ui| {
                                for (n, v) in decode(b, id, &m.data) {
                                    ui.monospace(n);
                                    ui.monospace(fmt_num(v));
                                    ui.end_row();
                                }
                            });
                        });
                    }
                    None => {
                        ui.weak("select a message to see its signals");
                    }
                }
            }
            TraceMode::Scroll => {
                let head = format!("{:>11}  {:<8}{:>5}  {:<22}{:<48}  {}", "time (s)", "bus", "id", "message", "data", "signals");
                ui.label(RichText::new(head).monospace().strong());
                let rows: Vec<&CanFrame> = self.trace.iter().filter(|f| self.filtered(f.bus, f.id)).collect();
                egui::ScrollArea::vertical().id_salt("trace").auto_shrink([false, false]).stick_to_bottom(!self.paused).show_rows(ui, row_h, rows.len(), |ui, range| {
                    for f in &rows[range] {
                        let text = format!(
                            "{:>11.6}  {:<8}{:>5X}  {:<22}{:<48}  {}",
                            f.t,
                            BUS_NAMES[f.bus as usize & 1],
                            f.id,
                            msg_name(f.bus, f.id),
                            hex(&f.data),
                            signals_text(&decode(f.bus, f.id, &f.data), 6)
                        );
                        ui.add(egui::Label::new(RichText::new(text).monospace().color(BUS_COLORS[f.bus as usize & 1])).truncate());
                    }
                });
            }
        }
    }
}

fn hud_text(f: u32) -> String {
    use adapter_api::hud_flags as h;
    let names = [
        (h::ACC, "ACC"),
        (h::FOLLOWING, "FOLLOW"),
        (h::LEAD, "LEAD"),
        (h::FCW, "FCW"),
        (h::AEB, "AEB"),
        (h::HOLD, "HOLD"),
        (h::OVERRIDE, "OVERRIDE"),
        (h::UNAVAILABLE, "UNAVAIL"),
        (h::AEB_OFF, "AEB-OFF"),
    ];
    let s: Vec<&str> = names.iter().filter(|(b, _)| f & b != 0).map(|(_, n)| *n).collect();
    if s.is_empty() { "-".into() } else { s.join(" ") }
}

fn adapter_text(code: u32) -> &'static str {
    adapter_api::hud_message::text(code)
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.pump();
        ui.ctx().request_repaint_after(Duration::from_millis(33));
        if let Some((path, at)) = &self.screenshot
            && Instant::now() >= *at
        {
            let path = path.clone();
            self.screenshot = None;
            ui.ctx().request_screenshot(move |img| {
                match save_png(&path, &img) {
                    Ok(()) => println!("saved {}", path.display()),
                    Err(e) => eprintln!("screenshot: {e}"),
                }
                std::process::exit(0);
            });
        }
        egui::Panel::top("status").show(ui, |ui| self.status_bar(ui));
        egui::Panel::left("side").resizable(true).default_size(380.0).show(ui, |ui| {
            egui::ScrollArea::vertical().id_salt("side-scroll").show(ui, |ui| {
                egui::CollapsingHeader::new("Vehicle and HUD").default_open(true).show(ui, |ui| self.vehicle_view(ui));
                egui::CollapsingHeader::new("Latency").default_open(true).show(ui, |ui| self.latency_view(ui));
                egui::CollapsingHeader::new("Fault injection").default_open(true).show(ui, |ui| self.fault_view(ui));
                egui::CollapsingHeader::new("Events").default_open(true).show(ui, |ui| {
                    for e in self.events.iter().rev().take(30) {
                        ui.small(e);
                    }
                });
            });
        });
        egui::CentralPanel::default().show(ui, |ui| {
            ui.strong("Bus load per network");
            self.load_plot(ui, 150.0);
            self.bus_view(ui);
            ui.separator();
            ui.strong("CAN trace");
            self.trace_view(ui);
        });
    }
}

fn main() -> eframe::Result {
    let mut port = telemetry::PANEL_PORT;
    let (mut shot, mut after) = (None, 5.0f64);
    let mut trace = false;
    let mut select: Option<(u8, u32)> = None;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--port" => port = args.next().and_then(|v| v.parse().ok()).expect("--port needs a number"),
            "--trace" => trace = true,
            "--select" => {
                // chassis:343 or adas:301 (hex id)
                let v = args.next().expect("--select needs bus:hexid");
                let (b, id) = v.split_once(':').expect("--select needs bus:hexid");
                let bus = BUS_NAMES.iter().position(|n| *n == b).expect("bus is chassis or adas") as u8;
                select = Some((bus, u32::from_str_radix(id.trim_start_matches("0x"), 16).expect("hex id")));
            }
            "--screenshot" => shot = Some(std::path::PathBuf::from(args.next().expect("--screenshot needs a path"))),
            "--after" => after = args.next().and_then(|v| v.parse().ok()).expect("--after needs seconds"),
            other => {
                eprintln!("unknown argument {other}; usage: adas-panel [--port N] [--screenshot OUT.png --after S]");
                std::process::exit(2);
            }
        }
    }
    let options = eframe::NativeOptions { viewport: egui::ViewportBuilder::default().with_inner_size([1500.0, 900.0]).with_title("ADAS engineering panel"), ..Default::default() };
    eframe::run_native(
        "ADAS engineering panel",
        options,
        Box::new(move |_cc| {
            let mut app = App::new(port);
            app.screenshot = shot.map(|p| (p, Instant::now() + Duration::from_secs_f64(after)));
            app.selected = select;
            if trace {
                app.mode = TraceMode::Scroll;
            }
            Ok(Box::new(app))
        }),
    )
}
