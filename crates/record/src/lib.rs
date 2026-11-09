//! Drive recordings (D20).
//!
//! One MCAP file per drive. Each core step writes, in order: `sample`,
//! `actors`, `stick` (if the test driver is running) and `command`. The bytes
//! are the contract's `#[repr(C)]` layouts (encoding `adas-raw`, schema data =
//! contract version), so a recording is tied to the contract version it was
//! made with, and a replay can feed the core exactly what it saw live.
//! Metadata `adas` holds what the replay needs to rebuild the core: the
//! feature, its parameters and the initial engage-request flag.

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{self, BufWriter};
use std::path::Path;

use adapter_api::{ActorList, AdapterInfo, CONTRACT_VERSION, Command, Header, MAX_ACTORS, VehicleSample};

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
    sample: u16,
    actors: u16,
    stick: u16,
    command: u16,
    info: u16,
}

pub struct Recorder {
    w: mcap::Writer<BufWriter<File>>,
    ch: Channels,
    seq: u32,
}

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
            sample: chan("sample", "adas.VehicleSample")?,
            actors: chan("actors", "adas.ActorList")?,
            stick: chan("stick", "adas.Stick")?,
            command: chan("command", "adas.Command")?,
            info: chan("info", "adas.AdapterInfo")?,
        };
        let mut meta = meta.clone();
        meta.insert("contract".into(), ver);
        w.write_metadata(&mcap::records::Metadata { name: "adas".into(), metadata: meta }).map_err(err)?;
        Ok(Self { w, ch, seq: 0 })
    }

    fn put(&mut self, channel: u16, t_ns: u64, data: &[u8]) -> io::Result<()> {
        self.seq = self.seq.wrapping_add(1);
        let h = mcap::records::MessageHeader { channel_id: channel, sequence: self.seq, log_time: t_ns, publish_time: t_ns };
        self.w.write_to_known_channel(&h, data).map_err(|e| io::Error::other(e.to_string()))
    }

    pub fn info(&mut self, i: &AdapterInfo) -> io::Result<()> {
        self.put(self.ch.info, i.header.wall_time_ns, bytes_of(i))
    }

    /// Records one core step.
    pub fn step(&mut self, s: &VehicleSample, a: &ActorList, stick: Option<Stick3>, c: &Command) -> io::Result<()> {
        let t = s.header.wall_time_ns;
        self.put(self.ch.sample, t, bytes_of(s))?;
        self.put(self.ch.actors, t, &actors_bytes(a))?;
        if let Some(st) = stick {
            self.put(self.ch.stick, t, bytes_of(&st))?;
        }
        self.put(self.ch.command, c.header.wall_time_ns.max(t), bytes_of(c))
    }

    pub fn finish(mut self) -> io::Result<()> {
        self.w.finish().map(|_| ()).map_err(|e| io::Error::other(e.to_string()))
    }
}

/// One recorded core step.
#[derive(Clone, Debug)]
pub struct Frame {
    pub sample: VehicleSample,
    pub actors: ActorList,
    pub stick: Option<Stick3>,
    pub command: Option<Command>,
}

/// A whole recording, read into memory.
pub struct Recording {
    pub meta: BTreeMap<String, String>,
    pub info: Option<AdapterInfo>,
    pub frames: Vec<Frame>,
}

impl Recording {
    pub fn open(path: &Path) -> io::Result<Self> {
        let data = std::fs::read(path)?;
        let bad = |m: String| io::Error::new(io::ErrorKind::InvalidData, m);
        let mut meta = BTreeMap::new();
        // Metadata records are found by scanning; the file is small enough.
        for rec in mcap::read::LinearReader::new(&data).map_err(|e| bad(e.to_string()))? {
            if let Ok(mcap::records::Record::Metadata(m)) = rec
                && m.name == "adas"
            {
                meta = m.metadata;
            }
        }
        let ver = CONTRACT_VERSION.to_string();
        if meta.get("contract") != Some(&ver) {
            return Err(bad(format!("recording is contract {:?}, this build is {ver}", meta.get("contract"))));
        }
        let mut info = None;
        let mut frames: Vec<Frame> = Vec::new();
        for m in mcap::MessageStream::new(&data).map_err(|e| bad(e.to_string()))? {
            let m = m.map_err(|e| bad(e.to_string()))?;
            match m.channel.topic.as_str() {
                "sample" => frames.push(Frame {
                    sample: from_bytes(&m.data).ok_or_else(|| bad("bad sample".into()))?,
                    actors: ActorList::default(),
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
                "command" => {
                    if let Some(f) = frames.last_mut() {
                        f.command = from_bytes(&m.data);
                    }
                }
                "info" => info = from_bytes(&m.data),
                _ => {}
            }
        }
        Ok(Self { meta, info, frames })
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
            rec.step(&s, &actors, stick, &c).unwrap();
        }
        rec.finish().unwrap();

        let r = Recording::open(&path).unwrap();
        assert_eq!(r.meta.get("feature").unwrap(), "speed-profile");
        assert_eq!(r.frames.len(), 5);
        assert_eq!(r.frames[2].sample.speed, 3.0);
        assert_eq!(r.frames[2].command.unwrap().answers_seq, 3);
        assert_eq!(r.frames[1].stick.unwrap().steer, -0.25);
        assert!(r.frames[0].stick.is_none());
        assert_eq!(r.frames[4].actors.as_slice().len(), 2);
        assert_eq!(r.frames[4].actors.actors[1].kind, ActorKind::Pedestrian);
        let _ = std::fs::remove_dir_all(dir);
    }
}
