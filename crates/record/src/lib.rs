//! Drive recordings (D20).
//!
//! One MCAP file per drive. Each core step writes, in order: `sample`,
//! `actors`, `ranges` (while the range fans are answered), `lanes` (while the
//! host gives lane markings), `stick` (if the test driver is running) and
//! `command`. The bytes
//! are the contract's `#[repr(C)]` layouts (encoding `adas-raw`, schema data =
//! contract version), so a recording is tied to the contract version it was
//! made with, and a replay can feed the core exactly what it saw live.
//! Metadata `adas` holds what the replay needs to rebuild the core: the
//! feature, its parameters and the initial engage-request flag.
//!
//! Between steps: `info` (adapter info), `profile` (a vehicle profile) and
//! `fault` (a bus fault injected live, from the engineering panel). Each
//! applies before the next step, live and in a replay.

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{self, BufWriter};
use std::path::Path;

use adapter_api::{ActorList, AdapterInfo, CONTRACT_VERSION, Command, Header, LaneMap, MAX_ACTORS, RangeReport, VehicleSample};

pub const ENCODING: &str = "adas-raw";

/// Types whose bytes can be recorded as they are.
///
/// # Safety
/// The type must be `#[repr(C)]`, `Copy`, have no padding the recorder would
/// read as uninitialised, and every bit pattern a valid writer produces must be
/// a valid value (the contract types are built that way: explicit padding
/// fields, enums only from known values).
pub unsafe trait Raw: Copy + Default {}
unsafe impl Raw for VehicleSample {}
unsafe impl Raw for Command {}
unsafe impl Raw for AdapterInfo {}
unsafe impl Raw for Stick3 {}
unsafe impl Raw for Profile {}
unsafe impl Raw for FaultRec {}
unsafe impl Raw for RangeReport {}
unsafe impl Raw for LaneMap {}

/// A bus fault injected (or faults cleared) between steps.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FaultRec {
    /// 1 = inject, 2 = clear.
    pub op: u32,
    /// 0 chassis, 1 adas; for clear, `u32::MAX` = both.
    pub bus: u32,
    /// Message id; `u32::MAX` = every frame.
    pub id: u32,
    /// 0 drop, 1 corrupt, 2 delay.
    pub kind: u32,
    pub delay: f64,
    pub p: f64,
    /// Active for bus times in `[from, until)` (s).
    pub from: f64,
    pub until: f64,
}

impl FaultRec {
    pub const INJECT: u32 = 1;
    pub const CLEAR: u32 = 2;
}

/// A vehicle profile as applied to the core (D19), recorded so a replay
/// applies it at the same step.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Profile {
    /// Brake gain (m/s² per unit of brake demand).
    pub brake_gain: f32,
    pub _reserved: [f32; 3],
}

/// The test driver's stick as recorded: throttle, brake, steer.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Stick3 {
    pub throttle: f32,
    pub brake: f32,
    pub steer: f32,
}

fn bytes_of<T: Raw>(v: &T) -> &[u8] {
    // SAFETY: `Raw` types are plain old data.
    unsafe { core::slice::from_raw_parts(v as *const T as *const u8, core::mem::size_of::<T>()) }
}

fn from_bytes<T: Raw>(b: &[u8]) -> Option<T> {
    if b.len() != core::mem::size_of::<T>() {
        return None;
    }
    let mut v = T::default();
    // SAFETY: as above; the length matches.
    unsafe { core::ptr::copy_nonoverlapping(b.as_ptr(), &mut v as *mut T as *mut u8, b.len()) };
    Some(v)
}

/// The actor list is recorded trimmed to `count` actors.
fn actors_bytes(a: &ActorList) -> Vec<u8> {
    let head = core::mem::size_of::<Header>() + 8;
    let n = (a.count as usize).min(MAX_ACTORS);
    let full = unsafe { core::slice::from_raw_parts(a as *const ActorList as *const u8, core::mem::size_of::<ActorList>()) };
    full[..head + n * core::mem::size_of::<adapter_api::Actor>()].to_vec()
}

fn actors_from(b: &[u8]) -> Option<ActorList> {
    let head = core::mem::size_of::<Header>() + 8;
    let one = core::mem::size_of::<adapter_api::Actor>();
    if b.len() < head || !(b.len() - head).is_multiple_of(one) || (b.len() - head) / one > MAX_ACTORS {
        return None;
    }
    let mut a = ActorList::default();
    unsafe { core::ptr::copy_nonoverlapping(b.as_ptr(), &mut a as *mut ActorList as *mut u8, b.len()) };
    if a.count as usize != (b.len() - head) / one {
        return None;
    }
    Some(a)
}

struct Channels {
    profile: u16,
    sample: u16,
    actors: u16,
    stick: u16,
    command: u16,
    info: u16,
    fault: u16,
    ranges: u16,
    lanes: u16,
}

pub struct Recorder {
    w: mcap::Writer<BufWriter<File>>,
    ch: Channels,
    seq: u32,
    steps: u32,
}

/// Steps between flushes. Each flush closes the current chunk, so a crash
/// loses at most this many steps (about a second at 72 fps).
const FLUSH_EVERY: u32 = 72;

impl Recorder {
    /// Creates a recording. `meta` goes into the `adas` metadata record.
    pub fn create(path: &Path, meta: &BTreeMap<String, String>) -> io::Result<Self> {
        let f = BufWriter::new(File::create(path)?);
        let err = |e: mcap::McapError| io::Error::other(e.to_string());
        let mut w = mcap::WriteOptions::new().compression(None).profile("adas").create(f).map_err(err)?;
        let ver = CONTRACT_VERSION.to_string();
        let mut chan = |topic: &str, schema: &str| -> io::Result<u16> {
            let s = w.add_schema(schema, ENCODING, ver.as_bytes()).map_err(err)?;
            w.add_channel(s, topic, ENCODING, &BTreeMap::new()).map_err(err)
        };
        let ch = Channels {
            profile: chan("profile", "adas.Profile")?,
            sample: chan("sample", "adas.VehicleSample")?,
            actors: chan("actors", "adas.ActorList")?,
            stick: chan("stick", "adas.Stick")?,
            command: chan("command", "adas.Command")?,
            info: chan("info", "adas.AdapterInfo")?,
            fault: chan("fault", "adas.Fault")?,
            ranges: chan("ranges", "adas.RangeReport")?,
            lanes: chan("lanes", "adas.LaneMap")?,
        };
        let mut meta = meta.clone();
        meta.insert("contract".into(), ver);
        w.write_metadata(&mcap::records::Metadata { name: "adas".into(), metadata: meta }).map_err(err)?;
        Ok(Self { w, ch, seq: 0, steps: 0 })
    }

    fn put(&mut self, channel: u16, t_ns: u64, data: &[u8]) -> io::Result<()> {
        self.seq = self.seq.wrapping_add(1);
        let h = mcap::records::MessageHeader { channel_id: channel, sequence: self.seq, log_time: t_ns, publish_time: t_ns };
        self.w.write_to_known_channel(&h, data).map_err(|e| io::Error::other(e.to_string()))
    }

    pub fn profile(&mut self, p: &Profile, t_ns: u64) -> io::Result<()> {
        self.put(self.ch.profile, t_ns, bytes_of(p))
    }

    pub fn info(&mut self, i: &AdapterInfo) -> io::Result<()> {
        self.put(self.ch.info, i.header.wall_time_ns, bytes_of(i))
    }

    /// A fault injected (or faults cleared) before the next step.
    pub fn fault(&mut self, f: &FaultRec, t_ns: u64) -> io::Result<()> {
        self.put(self.ch.fault, t_ns, bytes_of(f))
    }

    /// Records one core step.
#[allow(clippy::too_many_arguments)]
    pub fn step(&mut self, s: &VehicleSample, a: &ActorList, ranges: Option<&RangeReport>, lanes: Option<&LaneMap>, stick: Option<Stick3>, c: &Command) -> io::Result<()> {
        let t = s.header.wall_time_ns;
        self.put(self.ch.sample, t, bytes_of(s))?;
        self.put(self.ch.actors, t, &actors_bytes(a))?;
        if let Some(r) = ranges.filter(|r| r.count > 0) {
            self.put(self.ch.ranges, t, bytes_of(r))?;
        }
        if let Some(l) = lanes.filter(|l| l.count > 0) {
            self.put(self.ch.lanes, t, bytes_of(l))?;
        }
        if let Some(st) = stick {
            self.put(self.ch.stick, t, bytes_of(&st))?;
        }
        self.put(self.ch.command, c.header.wall_time_ns.max(t), bytes_of(c))?;
        self.steps += 1;
        if self.steps.is_multiple_of(FLUSH_EVERY) {
            self.w.flush().map_err(|e| io::Error::other(e.to_string()))?;
        }
        Ok(())
    }

    pub fn finish(mut self) -> io::Result<()> {
        self.w.finish().map(|_| ()).map_err(|e| io::Error::other(e.to_string()))
    }
}

/// One recorded core step.
#[derive(Clone, Debug)]
pub struct Frame {
    /// Adapter info that arrived just before this step (the core applies it
    /// before stepping).
    pub info: Option<AdapterInfo>,
    /// A vehicle profile applied just before this step.
    pub profile: Option<Profile>,
    /// Faults injected or cleared just before this step, in order.
    pub faults: Vec<FaultRec>,
    pub sample: VehicleSample,
    pub actors: ActorList,
    /// The range fans' answers, while they were active.
    pub ranges: Option<RangeReport>,
    /// The lane markings, while the host gave them.
    pub lanes: Option<Box<LaneMap>>,
    pub stick: Option<Stick3>,
    pub command: Option<Command>,
}

/// A whole recording, read into memory.
pub struct Recording {
    pub meta: BTreeMap<String, String>,
    pub info: Option<AdapterInfo>,
    pub frames: Vec<Frame>,
    /// The file ended early (the recorder was killed): `frames` holds every
    /// complete step before the cut.
    pub truncated: bool,
}

impl Recording {
    pub fn open(path: &Path) -> io::Result<Self> {
        let data = std::fs::read(path)?;
        let bad = |m: String| io::Error::new(io::ErrorKind::InvalidData, m);
        let mut meta = BTreeMap::new();
        // Metadata records are found by scanning; the file is small enough.
        for rec in mcap::read::LinearReader::new(&data).map_err(|e| bad(e.to_string()))? {
            match rec {
                Ok(mcap::records::Record::Metadata(m)) if m.name == "adas" => {
                    meta = m.metadata;
                    break;
                }
                Ok(_) => {}
                // An error doesn't advance the reader: stop rather than spin.
                Err(e) => return Err(bad(format!("reading records: {e}"))),
            }
        }
        let theirs = meta.get("contract").and_then(|v| v.parse::<u16>().ok());
        if !theirs.is_some_and(|v| adapter_api::RECORDING_COMPATIBLE.contains(&v)) {
            return Err(bad(format!("recording is contract {:?}, this build is {CONTRACT_VERSION}", meta.get("contract"))));
        }
        let mut info = None;
        let mut frames: Vec<Frame> = Vec::new();
        let mut truncated = false;
        let mut pending_info: Option<AdapterInfo> = None;
        let mut pending_profile: Option<Profile> = None;
        let mut pending_faults: Vec<FaultRec> = Vec::new();
        for m in mcap::MessageStream::new(&data).map_err(|e| bad(e.to_string()))? {
            let m = match m {
                Ok(m) => m,
                Err(e) if !frames.is_empty() => {
                    let _ = e;
                    truncated = true;
                    break;
                }
                Err(e) => return Err(bad(e.to_string())),
            };
            match m.channel.topic.as_str() {
                "sample" => frames.push(Frame {
                    info: pending_info.take(),
                    profile: pending_profile.take(),
                    faults: std::mem::take(&mut pending_faults),
                    sample: from_bytes(&m.data).ok_or_else(|| bad("bad sample".into()))?,
                    actors: ActorList::default(),
                    ranges: None,
                    lanes: None,
                    stick: None,
                    command: None,
                }),
                "actors" => {
                    if let Some(f) = frames.last_mut() {
                        f.actors = actors_from(&m.data).ok_or_else(|| bad("bad actor list".into()))?;
                    }
                }
                "stick" => {
                    if let Some(f) = frames.last_mut() {
                        f.stick = from_bytes(&m.data);
                    }
                }
                "ranges" => {
                    if let Some(f) = frames.last_mut() {
                        f.ranges = from_bytes(&m.data);
                    }
                }
                "lanes" => {
                    if let Some(f) = frames.last_mut() {
                        f.lanes = from_bytes(&m.data).map(Box::new);
                    }
                }
                "command" => {
                    if let Some(f) = frames.last_mut() {
                        f.command = from_bytes(&m.data);
                    }
                }
                "profile" => pending_profile = from_bytes(&m.data),
                "fault" => pending_faults.extend(from_bytes::<FaultRec>(&m.data)),
                "info" => {
                    let i: Option<AdapterInfo> = from_bytes(&m.data);
                    if info.is_none() {
                        info = i;
                    }
                    pending_info = i;
                }
                _ => {}
            }
        }
        // A step cut short (no command yet) can't be compared: drop it.
        if truncated && frames.last().is_some_and(|f| f.command.is_none()) {
            frames.pop();
        }
        Ok(Self { meta, info, frames, truncated })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use adapter_api::{Actor, ActorKind};

    #[test]
    fn round_trip() {
        let dir = std::env::temp_dir().join(format!("adas-rec-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("t.mcap");
        let mut meta = BTreeMap::new();
        meta.insert("feature".into(), "speed-profile".into());
        let mut rec = Recorder::create(&path, &meta).unwrap();
        let mut actors = ActorList { count: 2, ..Default::default() };
        actors.actors[0] = Actor { id: 7, kind: ActorKind::Vehicle, x: 30.0, ..Actor::ZERO };
        actors.actors[1] = Actor { id: 9, kind: ActorKind::Pedestrian, y: -3.0, ..Actor::ZERO };
        for i in 1..=5u32 {
            let mut s = VehicleSample { speed: i as f32, ..Default::default() };
            s.header.seq = i;
            s.header.wall_time_ns = i as u64 * 1000;
            let c = Command { answers_seq: i, throttle: 0.1 * i as f32, ..Default::default() };
            let stick = (i % 2 == 0).then_some(Stick3 { throttle: 0.5, brake: 0.0, steer: -0.25 });
            if i == 3 {
                let f = FaultRec { op: FaultRec::INJECT, bus: 1, id: u32::MAX, kind: 0, p: 1.0, from: 1.0, until: 3.0, ..Default::default() };
                rec.fault(&f, 2500).unwrap();
                rec.fault(&FaultRec { op: FaultRec::CLEAR, bus: u32::MAX, ..Default::default() }, 2600).unwrap();
            }
            let mut ranges = RangeReport { count: 2, ..Default::default() };
            ranges.fans[1].hits[0] = i as f32;
            let mut lanes = LaneMap { count: 1, ..Default::default() };
            lanes.lines[0].count = 2;
            lanes.lines[0].points[1] = [30.0, 1.8];
            rec.step(&s, &actors, (i == 4).then_some(&ranges), (i == 2).then_some(&lanes), stick, &c).unwrap();
        }
        rec.finish().unwrap();

        let r = Recording::open(&path).unwrap();
        assert_eq!(r.meta.get("feature").unwrap(), "speed-profile");
        assert_eq!(r.frames.len(), 5);
        assert_eq!(r.frames[2].sample.speed, 3.0);
        assert_eq!(r.frames[2].command.unwrap().answers_seq, 3);
        assert_eq!(r.frames[1].stick.unwrap().steer, -0.25);
        assert!(r.frames[0].stick.is_none());
        assert_eq!(r.frames[2].faults.len(), 2, "faults apply before the step they precede");
        assert_eq!((r.frames[2].faults[0].bus, r.frames[2].faults[0].until), (1, 3.0));
        assert_eq!(r.frames[2].faults[1].op, FaultRec::CLEAR);
        assert!(r.frames[1].faults.is_empty() && r.frames[3].faults.is_empty());
        assert_eq!(r.frames[4].actors.as_slice().len(), 2);
        assert_eq!(r.frames[4].actors.actors[1].kind, ActorKind::Pedestrian);
        assert_eq!(r.frames[3].ranges.map(|g| g.fans[1].hits[0]), Some(4.0));
        assert!(r.frames[2].ranges.is_none() && r.frames[4].ranges.is_none());
        assert_eq!(r.frames[1].lanes.as_ref().map(|l| l.lines[0].points[1]), Some([30.0, 1.8]));
        assert!(r.frames[0].lanes.is_none());
        assert!(!r.truncated);
        let _ = std::fs::remove_dir_all(dir);
    }

    #[test]
    fn a_killed_recorder_leaves_a_readable_file() {
        let dir = std::env::temp_dir().join(format!("adas-rec-trunc-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("t.mcap");
        let mut rec = Recorder::create(&path, &BTreeMap::new()).unwrap();
        let a = ActorList::default();
        for i in 1..=200u32 {
            let s = VehicleSample { speed: i as f32, ..Default::default() };
            rec.step(&s, &a, None, None, None, &Command { answers_seq: i, ..Default::default() }).unwrap();
        }
        // Simulate a crash: the writer is dropped without `finish`, after the
        // file is cut mid-way through the last (unflushed) chunk.
        std::mem::forget(rec);
        let full = std::fs::read(&path).unwrap();
        std::fs::write(&path, &full[..full.len().saturating_sub(10)]).unwrap();
        let r = Recording::open(&path).unwrap();
        assert!(r.truncated || r.frames.len() >= 144);
        assert!(r.frames.len() >= 144, "lost more than the unflushed tail: {} frames", r.frames.len());
        assert!(r.frames.iter().all(|f| f.command.is_some()));
        let _ = std::fs::remove_dir_all(dir);
    }
}
