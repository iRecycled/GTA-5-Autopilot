//! The shared region on Windows: a named file mapping holding [`Link`], plus a
//! named auto-reset event the adapter signals after each sample so the core
//! can sleep instead of spinning.
//!
//! Either side may start first: both call [`SharedLink::open`], which creates
//! the mapping if it doesn't exist yet. Fresh mappings are zeroed, and an
//! all-zero `Link` is valid (no messages yet).

use core::ffi::c_void;
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::io;
use std::sync::OnceLock;

use crate::{ActorList, AdapterInfo, CONTRACT_VERSION, Command, FanLayout, Hud, LaneMap, Mailbox, RangeReport, SensorLayout, VehicleSample};

/// Default name of the link for one adapter ↔ core pair. It carries the
/// contract version, so a core or adapter left running from another
/// contract (its mapping laid out differently) can never share the link.
pub const DEFAULT_NAME: &str = "Local\\adas-link-c6";
const LINK_MAGIC: u64 = u64::from_le_bytes(*b"ADASLNK1");

/// The shared region's layout.
#[repr(C)]
pub struct Link {
    magic: AtomicU64,
    contract_version: AtomicU32,
    /// Incremented by the core each time it starts; a new epoch means a fresh,
    /// disengaged core (the adapter drops any engagement).
    pub epoch: AtomicU32,
    /// The core's process id, written at start-up, so the adapter can notice a
    /// crashed core at once instead of waiting for the watchdog.
    pub core_pid: AtomicU32,
    pub _reserved: u32,
    /// The adapter's frame counter, updated every frame (a heartbeat).
    pub adapter_frame: AtomicU64,
    /// adapter → core, every frame.
    pub sample: Mailbox<VehicleSample>,
    /// core → adapter, every step.
    pub command: Mailbox<Command>,
    /// adapter → core, every frame (nearest actors with line-of-sight bits).
    pub actors: Mailbox<ActorList>,
    /// core → adapter: the standing line-of-sight queries.
    pub layout: Mailbox<SensorLayout>,
    /// adapter → core: identity and capabilities.
    pub info: Mailbox<AdapterInfo>,
    /// core → adapter: what the HUD shows.
    pub hud: Mailbox<Hud>,
    /// core → adapter: the standing range fans (v4).
    pub fans: Mailbox<FanLayout>,
    /// adapter → core, every frame while the fans are active (v4).
    pub ranges: Mailbox<RangeReport>,
    /// adapter → core, every frame: the lane markings around the car (v5).
    pub lanes: Mailbox<LaneMap>,
}

#[link(name = "kernel32", kind = "raw-dylib")]
unsafe extern "system" {
    fn CreateFileMappingW(file: isize, attrs: *const c_void, protect: u32, size_hi: u32, size_lo: u32, name: *const u16) -> *mut c_void;
    fn MapViewOfFile(mapping: *mut c_void, access: u32, off_hi: u32, off_lo: u32, bytes: usize) -> *mut c_void;
    fn UnmapViewOfFile(addr: *const c_void) -> i32;
    fn CloseHandle(h: *mut c_void) -> i32;
    fn CreateEventW(attrs: *const c_void, manual_reset: i32, initial: i32, name: *const u16) -> *mut c_void;
    fn SetEvent(h: *mut c_void) -> i32;
    fn WaitForSingleObject(h: *mut c_void, ms: u32) -> u32;
    fn QueryPerformanceCounter(count: *mut i64) -> i32;
    fn QueryPerformanceFrequency(freq: *mut i64) -> i32;
}

const INVALID_HANDLE_VALUE: isize = -1;
const PAGE_READWRITE: u32 = 0x04;
const FILE_MAP_ALL_ACCESS: u32 = 0x000F_001F;
const WAIT_OBJECT_0: u32 = 0;

fn wide(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(Some(0)).collect()
}

/// QueryPerformanceCounter in ns.
pub fn qpc_ns() -> u64 {
    static FREQ: OnceLock<u64> = OnceLock::new();
    let f = *FREQ.get_or_init(|| {
        let mut f = 0i64;
        unsafe { QueryPerformanceFrequency(&mut f) };
        f.max(1) as u64
    });
    let mut c = 0i64;
    unsafe { QueryPerformanceCounter(&mut c) };
    ((c as u128) * 1_000_000_000 / f as u128) as u64
}

/// An open link. Dereferences to [`Link`].
pub struct SharedLink {
    mapping: *mut c_void,
    view: *mut Link,
    event: *mut c_void,
}

// The handles and the view may be used from any thread.
unsafe impl Send for SharedLink {}
unsafe impl Sync for SharedLink {}

impl SharedLink {
    /// Opens the link called `name`, creating it if needed. Fails if the other
    /// side was built against a different contract version.
    pub fn open(name: &str) -> io::Result<Self> {
        let size = core::mem::size_of::<Link>();
        let wname = wide(name);
        let wevent = wide(&format!("{name}-sample"));
        unsafe {
            let mapping = CreateFileMappingW(INVALID_HANDLE_VALUE, core::ptr::null(), PAGE_READWRITE, 0, size as u32, wname.as_ptr());
            if mapping.is_null() {
                return Err(io::Error::last_os_error());
            }
            let view = MapViewOfFile(mapping, FILE_MAP_ALL_ACCESS, 0, 0, size) as *mut Link;
            if view.is_null() {
                let e = io::Error::last_os_error();
                CloseHandle(mapping);
                return Err(e);
            }
            let event = CreateEventW(core::ptr::null(), 0, 0, wevent.as_ptr());
            if event.is_null() {
                let e = io::Error::last_os_error();
                UnmapViewOfFile(view as *const c_void);
                CloseHandle(mapping);
                return Err(e);
            }
            let link = Self { mapping, view, event };
            // First opener stamps the region; later openers check it.
            let l = &*view;
            if l.magic.compare_exchange(0, LINK_MAGIC, Ordering::AcqRel, Ordering::Acquire).is_ok() {
                l.contract_version.store(CONTRACT_VERSION as u32, Ordering::Release);
            } else if l.magic.load(Ordering::Acquire) != LINK_MAGIC {
                return Err(io::Error::new(io::ErrorKind::InvalidData, "shared region is not an ADAS link"));
            }
            let theirs = l.contract_version.load(Ordering::Acquire);
            if theirs != CONTRACT_VERSION as u32 {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("contract version mismatch: link has {theirs}, we are {CONTRACT_VERSION}"),
                ));
            }
            Ok(link)
        }
    }

    /// Wakes a core waiting in [`SharedLink::wait_sample`].
    pub fn signal_sample(&self) {
        unsafe { SetEvent(self.event) };
    }

    /// Sleeps until the adapter signals a sample or `timeout_ms` passes.
    /// Returns false on timeout.
    pub fn wait_sample(&self, timeout_ms: u32) -> bool {
        unsafe { WaitForSingleObject(self.event, timeout_ms) == WAIT_OBJECT_0 }
    }
}

impl core::ops::Deref for SharedLink {
    type Target = Link;
    fn deref(&self) -> &Link {
        // SAFETY: the view stays mapped until drop.
        unsafe { &*self.view }
    }
}

impl Drop for SharedLink {
    fn drop(&mut self) {
        unsafe {
            CloseHandle(self.event);
            UnmapViewOfFile(self.view as *const c_void);
            CloseHandle(self.mapping);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn link_name_carries_the_contract_version() {
        assert!(DEFAULT_NAME.ends_with(&format!("-c{CONTRACT_VERSION}")), "rename DEFAULT_NAME with the contract version");
    }

    #[test]
    fn two_opens_share_memory() {
        let name = format!("Local\\adas-link-test-{}", std::process::id());
        let a = SharedLink::open(&name).unwrap();
        let b = SharedLink::open(&name).unwrap();
        a.adapter_frame.store(42, Ordering::Release);
        assert_eq!(b.adapter_frame.load(Ordering::Acquire), 42);
        a.sample.write(&VehicleSample { speed: 3.5, ..Default::default() });
        assert_eq!(b.sample.read().1.speed, 3.5);
        a.signal_sample();
        assert!(b.wait_sample(100));
        assert!(!b.wait_sample(1), "auto-reset event fired twice");
    }
}
