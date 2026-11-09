//! DBC to Rust (Phase 1): one struct per message, with typed fields and
//! straight-line shift-and-mask packing, so a message is checked by the
//! compiler instead of looked up by name at run time.
//!
//! A crate's `build.rs` parses a DBC and writes [`generate`]'s output to
//! `OUT_DIR`, then `include!`s it. The generated code scales values with
//! [`phys_to_raw`](crate::dbc::phys_to_raw) and
//! [`raw_to_phys`](crate::dbc::raw_to_phys), the same functions
//! [`Signal`](crate::Signal) uses, so both paths produce identical bytes.
//!
//! Per message, the generated code has:
//! - a struct (`STEERING_LTA` → `SteeringLta`, `STEER_ANGLE_CMD` →
//!   `steer_angle_cmd`). One-bit unsigned unscaled signals are `bool`, and the
//!   rest are `f64` in physical units. `Default` is the all-zero payload;
//! - [`CanMessage`]: `ID`, `LEN`, `NAME`, `SIGNALS`, `encode`, `decode`;
//! - `get` and `set` by signal name, for tools.
//!
//! Per file, the generated code has `MESSAGES`, `id`, `name`, `signals`,
//! `decode_signals` and `encode_signals` (by id, for traces and tests).

use std::collections::HashSet;
use std::fmt::Write;

use crate::dbc::{Dbc, Message, Signal};

/// A message type generated from a DBC.
pub trait CanMessage: Sized + Default {
    const ID: u32;
    const LEN: usize;
    const NAME: &'static str;
    const SIGNALS: &'static [&'static str];
    /// Packs every signal into `data` (at least `LEN` bytes); other bits keep
    /// their value.
    fn encode_into(&self, data: &mut [u8]);
    /// Unpacks a payload; missing bytes read as 0.
    fn decode(data: &[u8]) -> Self;
    fn encode(&self) -> Vec<u8> {
        let mut d = vec![0u8; Self::LEN];
        self.encode_into(&mut d);
        d
    }
    fn has_signal(name: &str) -> bool {
        Self::SIGNALS.contains(&name)
    }
}

const KEYWORDS: &[&str] = &[
    "as", "async", "await", "break", "const", "continue", "crate", "dyn", "else", "enum", "extern", "false", "fn", "for", "gen", "if", "impl", "in",
    "let", "loop", "match", "mod", "move", "mut", "pub", "ref", "return", "self", "static", "struct", "super", "trait", "true", "type", "unsafe",
    "use", "where", "while", "abstract", "become", "box", "do", "final", "macro", "override", "priv", "try", "typeof", "unsized", "virtual",
    "yield",
];

fn sanitize(s: &str) -> String {
    let mut out: String = s.chars().map(|c| if c.is_ascii_alphanumeric() || c == '_' { c } else { '_' }).collect();
    if out.is_empty() || out.starts_with(|c: char| c.is_ascii_digit()) {
        out.insert(0, '_');
    }
    out
}

/// `STEER_ANGLE_CMD` → `steer_angle_cmd`.
pub fn field_name(signal: &str) -> String {
    let f = sanitize(&signal.to_ascii_lowercase());
    if KEYWORDS.contains(&f.as_str()) { format!("r#{f}") } else { f }
}

/// `STEERING_LTA` → `SteeringLta`.
pub fn type_name(message: &str) -> String {
    let mut out = String::new();
    for part in message.split(|c: char| !c.is_ascii_alphanumeric()).filter(|p| !p.is_empty()) {
        let mut cs = part.chars();
        if let Some(c) = cs.next() {
            out.push(c.to_ascii_uppercase());
            out.extend(cs.map(|c| c.to_ascii_lowercase()));
        }
    }
    if out.is_empty() || out.starts_with(|c: char| c.is_ascii_digit()) {
        out.insert(0, 'M');
    }
    out
}

fn is_bool(s: &Signal) -> bool {
    s.len == 1 && !s.signed && s.factor == 1.0 && s.offset == 0.0
}

/// An `f64` literal that reads back as exactly the same value.
fn lit(v: f64) -> String {
    let s = format!("{v:?}");
    if s.contains(['.', 'e', 'E', 'n', 'N', 'i']) { format!("{s}_f64") } else { format!("{s}.0_f64") }
}

/// Generates the Rust code for every message in `dbc`. `source` names the
/// DBC in the header comment. Fails on names that collide once converted.
pub fn generate(dbc: &Dbc, source: &str) -> Result<String, String> {
    let mut o = String::new();
    // No inner attributes: the file is `include!`d into a module, which
    // carries the `#[allow]`s.
    let _ = writeln!(o, "// Generated from {source} by can::codegen. Do not edit: change the DBC instead.");
    let _ = writeln!(o, "use ::can::CanMessage;");
    let mut types = HashSet::new();
    let mut ids = HashSet::new();
    for m in &dbc.messages {
        if m.signals.iter().any(|s| s.multiplexed) {
            return Err(format!("{}: multiplexed signals are not supported by the code generator", m.name));
        }
        if !types.insert(type_name(&m.name)) {
            return Err(format!("{}: type name {} collides", m.name, type_name(&m.name)));
        }
        if !ids.insert(m.id) {
            return Err(format!("{}: id {} used twice", m.name, m.id));
        }
        message(&mut o, m)?;
    }
    // File-level lookups.
    let _ = writeln!(o, "\n/// (id, name, length) of every message.");
    let _ = writeln!(o, "pub const MESSAGES: &[(u32, &str, usize)] = &[");
    for m in &dbc.messages {
        let _ = writeln!(o, "    ({}, {:?}, {}),", m.id, m.name, m.len);
    }
    let _ = writeln!(o, "];");
    let _ = writeln!(o, "\npub fn id(name: &str) -> Option<u32> {{\n    match name {{");
    for m in &dbc.messages {
        let _ = writeln!(o, "        {:?} => Some({}),", m.name, m.id);
    }
    let _ = writeln!(o, "        _ => None,\n    }}\n}}");
    let _ = writeln!(o, "\npub fn name(id: u32) -> Option<&'static str> {{\n    match id {{");
    for m in &dbc.messages {
        let _ = writeln!(o, "        {} => Some({:?}),", m.id, m.name);
    }
    let _ = writeln!(o, "        _ => None,\n    }}\n}}");
    let _ = writeln!(o, "\n/// The signal names of message `id`.");
    let _ = writeln!(o, "pub fn signals(id: u32) -> Option<&'static [&'static str]> {{\n    match id {{");
    for m in &dbc.messages {
        let _ = writeln!(o, "        {} => Some({}::SIGNALS),", m.id, type_name(&m.name));
    }
    let _ = writeln!(o, "        _ => None,\n    }}\n}}");
    let _ = writeln!(o, "\n/// Every signal of message `id`, decoded: (name, physical value).");
    let _ = writeln!(o, "pub fn decode_signals(id: u32, data: &[u8]) -> Option<Vec<(&'static str, f64)>> {{\n    match id {{");
    for m in &dbc.messages {
        let t = type_name(&m.name);
        let _ = writeln!(o, "        {} => {{ let v = {t}::decode(data); Some({t}::SIGNALS.iter().map(|s| (*s, v.get(s).unwrap_or(0.0))).collect()) }}", m.id);
    }
    let _ = writeln!(o, "        _ => None,\n    }}\n}}");
    let _ = writeln!(o, "\n/// Encodes message `id` from (signal, physical value) pairs; other signals are raw 0.");
    let _ = writeln!(o, "pub fn encode_signals(id: u32, values: &[(&str, f64)]) -> Option<Vec<u8>> {{\n    match id {{");
    for m in &dbc.messages {
        let t = type_name(&m.name);
        let _ = writeln!(o, "        {} => {{ let mut v = {t}::default(); for (n, x) in values {{ if !v.set(n, *x) {{ return None; }} }} Some(v.encode()) }}", m.id);
    }
    let _ = writeln!(o, "        _ => None,\n    }}\n}}");
    Ok(o)
}

fn message(o: &mut String, m: &Message) -> Result<(), String> {
    let t = type_name(&m.name);
    let mut fields = HashSet::new();
    for s in &m.signals {
        if !fields.insert(field_name(&s.name)) {
            return Err(format!("{}.{}: field name {} collides", m.name, s.name, field_name(&s.name)));
        }
    }
    let _ = writeln!(o, "\n/// `{}`: id {} (0x{:X}), {} bytes, sent by {}.", m.name, m.id, m.id, m.len, m.sender);
    let _ = writeln!(o, "#[derive(Clone, Copy, Debug, PartialEq)]");
    let _ = writeln!(o, "pub struct {t} {{");
    for s in &m.signals {
        let order = if s.little_endian { "Intel" } else { "Motorola" };
        let sign = if s.signed { "signed" } else { "unsigned" };
        let unit = if s.unit.is_empty() { String::new() } else { format!(" {}", s.unit) };
        let _ = writeln!(
            o,
            "    /// `{}`: start {}, {} bits, {order}, {sign}, ×{} {:+}, [{}, {}]{unit}.",
            s.name, s.start, s.len, s.factor, s.offset, s.min, s.max
        );
        let ty = if is_bool(s) { "bool" } else { "f64" };
        let _ = writeln!(o, "    pub {}: {ty},", field_name(&s.name));
    }
    let _ = writeln!(o, "}}");

    // Default: the all-zero payload (a raw 0 is `offset` in physical units).
    let _ = writeln!(o, "\nimpl Default for {t} {{\n    fn default() -> Self {{\n        Self {{");
    for s in &m.signals {
        let v = if is_bool(s) { "false".to_string() } else { lit(s.offset) };
        let _ = writeln!(o, "            {}: {v},", field_name(&s.name));
    }
    let _ = writeln!(o, "        }}\n    }}\n}}");

    let _ = writeln!(o, "\nimpl CanMessage for {t} {{");
    let _ = writeln!(o, "    const ID: u32 = {};", m.id);
    let _ = writeln!(o, "    const LEN: usize = {};", m.len);
    let _ = writeln!(o, "    const NAME: &'static str = {:?};", m.name);
    let names: Vec<String> = m.signals.iter().map(|s| format!("{:?}", s.name)).collect();
    let _ = writeln!(o, "    const SIGNALS: &'static [&'static str] = &[{}];", names.join(", "));

    let _ = writeln!(o, "\n    fn encode_into(&self, d: &mut [u8]) {{");
    let _ = writeln!(o, "        assert!(d.len() >= {}, \"{} needs {} bytes\");", m.len, m.name, m.len);
    for s in &m.signals {
        let f = field_name(&s.name);
        if is_bool(s) {
            let _ = writeln!(o, "        let r = self.{f} as u64;");
        } else {
            let _ = writeln!(o, "        let r = ::can::dbc::phys_to_raw(self.{f}, {}, {}, {}, {});", s.len, s.signed, lit(s.factor), lit(s.offset));
        }
        for (byte, lo, w, rlo) in s.runs(m.len) {
            let field_mask: u16 = (1u16 << w) - 1;
            let byte_mask = (field_mask << lo) as u8;
            let _ = writeln!(o, "        d[{byte}] = (d[{byte}] & 0x{:02X}) | ((((r >> {rlo}) & 0x{field_mask:X}) as u8) << {lo});", !byte_mask);
        }
    }
    let _ = writeln!(o, "    }}");

    let _ = writeln!(o, "\n    fn decode(d: &[u8]) -> Self {{");
    let _ = writeln!(o, "        let mut b = [0u8; {}];", m.len);
    let _ = writeln!(o, "        let n = d.len().min({});", m.len);
    let _ = writeln!(o, "        b[..n].copy_from_slice(&d[..n]);");
    let _ = writeln!(o, "        Self {{");
    for s in &m.signals {
        let f = field_name(&s.name);
        let parts: Vec<String> = s.runs(m.len).into_iter().map(|(byte, lo, w, rlo)| format!("((((b[{byte}] >> {lo}) as u64) & 0x{:X}) << {rlo})", (1u16 << w) - 1)).collect();
        let raw = if parts.is_empty() { "0u64".to_string() } else { parts.join(" | ") };
        if is_bool(s) {
            let _ = writeln!(o, "            {f}: ({raw}) != 0,");
        } else {
            let _ = writeln!(o, "            {f}: ::can::dbc::raw_to_phys({raw}, {}, {}, {}, {}),", s.len, s.signed, lit(s.factor), lit(s.offset));
        }
    }
    let _ = writeln!(o, "        }}\n    }}\n}}");

    // Access by name, for tools.
    let _ = writeln!(o, "\nimpl {t} {{");
    let _ = writeln!(o, "    /// A signal's physical value by its DBC name (`bool` as 0 or 1).");
    let _ = writeln!(o, "    pub fn get(&self, signal: &str) -> Option<f64> {{\n        match signal {{");
    for s in &m.signals {
        let f = field_name(&s.name);
        let v = if is_bool(s) { format!("self.{f} as u8 as f64") } else { format!("self.{f}") };
        let _ = writeln!(o, "            {:?} => Some({v}),", s.name);
    }
    let _ = writeln!(o, "            _ => None,\n        }}\n    }}");
    let _ = writeln!(o, "    /// Sets a signal by its DBC name; `false` if there is no such signal.");
    let _ = writeln!(o, "    pub fn set(&mut self, signal: &str, value: f64) -> bool {{\n        match signal {{");
    for s in &m.signals {
        let f = field_name(&s.name);
        let v = if is_bool(s) { "value.round() as u64 & 1 == 1".to_string() } else { "value".to_string() };
        let _ = writeln!(o, "            {:?} => self.{f} = {v},", s.name);
    }
    let _ = writeln!(o, "            _ => return false,\n        }}\n        true\n    }}\n}}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names() {
        assert_eq!(type_name("STEERING_LTA"), "SteeringLta");
        assert_eq!(type_name("ENG2F41"), "Eng2f41");
        assert_eq!(type_name("2WD"), "M2wd");
        assert_eq!(field_name("STEER_ANGLE_CMD"), "steer_angle_cmd");
        assert_eq!(field_name("TYPE"), "r#type");
        assert_eq!(field_name("1ST"), "_1st");
    }

    #[test]
    fn literals_read_back_exactly() {
        for v in [0.0573, 1.0, -540.0, 0.01, 1e-5, 0.0, -0.0, 1.0 / 3.0] {
            let s = lit(v);
            let back: f64 = s.trim_end_matches("_f64").parse().unwrap();
            assert_eq!(back.to_bits(), v.to_bits(), "{v} -> {s}");
        }
    }

    #[test]
    fn runs_cover_every_bit_once() {
        let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../dbc/toyota_nodsu_pt_generated.dbc")).unwrap();
        let d = Dbc::parse(&text).unwrap();
        for m in &d.messages {
            for s in &m.signals {
                let runs = s.runs(m.len);
                let total: u32 = runs.iter().map(|r| r.2).sum();
                let in_range = (0..s.len).count() as u32;
                assert!(total <= in_range, "{}.{}", m.name, s.name);
                for &(byte, lo, w, _) in &runs {
                    assert!(byte < m.len && lo + w <= 8, "{}.{}: run {byte} {lo} {w}", m.name, s.name);
                }
            }
        }
        // STEERING_LTA.STEER_ANGLE_CMD: Motorola, 16 bits from bit 15: all of
        // byte 1 (raw bits 8..15), then all of byte 2 (raw bits 0..7).
        let lta = d.message("STEERING_LTA").unwrap();
        assert_eq!(lta.signal("STEER_ANGLE_CMD").unwrap().runs(8), vec![(1, 0, 8, 8), (2, 0, 8, 0)]);
        assert!(generate(&d, "tss2").is_ok());
    }
}
