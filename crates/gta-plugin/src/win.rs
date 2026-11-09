//! The few Win32 calls the adapter needs.

use core::ffi::c_void;

#[link(name = "kernel32", kind = "raw-dylib")]
unsafe extern "system" {
    pub fn DisableThreadLibraryCalls(module: *mut c_void) -> i32;
    pub fn GetModuleFileNameW(module: *mut c_void, buf: *mut u16, len: u32) -> u32;
    fn GetCurrentProcess() -> *mut c_void;
    fn ReadProcessMemory(process: *mut c_void, addr: *const c_void, buf: *mut c_void, size: usize, read: *mut usize) -> i32;
    fn WriteProcessMemory(process: *mut c_void, addr: *mut c_void, buf: *const c_void, size: usize, written: *mut usize) -> i32;
    fn OpenProcess(access: u32, inherit: i32, pid: u32) -> *mut c_void;
    fn WaitForSingleObject(h: *mut c_void, ms: u32) -> u32;
    fn CloseHandle(h: *mut c_void) -> i32;
}

#[link(name = "user32", kind = "raw-dylib")]
unsafe extern "system" {
    fn GetAsyncKeyState(vkey: i32) -> i16;
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
pub struct XGamepad {
    pub buttons: u16,
    pub left_trigger: u8,
    pub right_trigger: u8,
    pub thumb_lx: i16,
    pub thumb_ly: i16,
    pub thumb_rx: i16,
    pub thumb_ry: i16,
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct XState {
    packet: u32,
    pad: XGamepad,
}

#[link(name = "xinput1_4", kind = "raw-dylib")]
unsafe extern "system" {
    fn XInputGetState(index: u32, state: *mut XState) -> u32;
}

pub fn key_down(vk: i32) -> bool {
    unsafe { GetAsyncKeyState(vk) as u16 & 0x8000 != 0 }
}

/// The first connected XInput pad, if any.
pub fn gamepad() -> Option<XGamepad> {
    for i in 0..4 {
        let mut s = XState::default();
        if unsafe { XInputGetState(i, &mut s) } == 0 {
            return Some(s.pad);
        }
    }
    None
}

/// Reads bytes at `addr` in our own process without faulting.
pub fn read_bytes<const N: usize>(addr: usize) -> Option<[u8; N]> {
    let mut b = [0u8; N];
    let mut read = 0usize;
    let ok = unsafe { ReadProcessMemory(GetCurrentProcess(), addr as *const c_void, b.as_mut_ptr() as *mut c_void, N, &mut read) };
    (ok != 0 && read == N).then_some(b)
}

pub fn read_f32(addr: usize) -> Option<f32> {
    read_bytes::<4>(addr).map(f32::from_le_bytes)
}

pub fn read_u32(addr: usize) -> Option<u32> {
    read_bytes::<4>(addr).map(u32::from_le_bytes)
}

pub fn read_u64(addr: usize) -> Option<u64> {
    read_bytes::<8>(addr).map(u64::from_le_bytes)
}

#[repr(C)]
struct MemoryBasicInformation {
    base: *mut c_void,
    alloc_base: *mut c_void,
    alloc_protect: u32,
    partition: u16,
    region_size: usize,
    state: u32,
    protect: u32,
    kind: u32,
}

#[link(name = "kernel32", kind = "raw-dylib")]
unsafe extern "system" {
    fn VirtualQuery(addr: *const c_void, info: *mut MemoryBasicInformation, len: usize) -> usize;
}

const MEM_COMMIT: u32 = 0x1000;
const PAGE_GUARD: u32 = 0x100;
const WRITABLE: u32 = 0x04 | 0x08 | 0x40 | 0x80; // RW, WRITECOPY, EXECUTE_RW, EXECUTE_WRITECOPY

/// Writes an f32 in our own process. `WriteProcessMemory` costs about 2 ms per
/// call here (it changes page protection and flushes the instruction cache),
/// so check with `VirtualQuery` that the page is committed and writable, then
/// write directly.
pub fn write_f32(addr: usize, v: f32) -> bool {
    if addr == 0 || !addr.is_multiple_of(4) {
        return false;
    }
    let mut mbi = core::mem::MaybeUninit::<MemoryBasicInformation>::zeroed();
    let n = unsafe { VirtualQuery(addr as *const c_void, mbi.as_mut_ptr(), core::mem::size_of::<MemoryBasicInformation>()) };
    if n == 0 {
        return false;
    }
    let mbi = unsafe { mbi.assume_init() };
    let end = mbi.base as usize + mbi.region_size;
    if mbi.state != MEM_COMMIT || mbi.protect & PAGE_GUARD != 0 || mbi.protect & WRITABLE == 0 || addr + 4 > end {
        return false;
    }
    // SAFETY: the page is committed, writable and the write stays inside it.
    unsafe { core::ptr::write_volatile(addr as *mut f32, v) };
    true
}

/// The old path, kept for comparison in diagnostics.
#[allow(dead_code)]
pub fn write_f32_wpm(addr: usize, v: f32) -> bool {
    let b = v.to_le_bytes();
    let mut written = 0usize;
    let ok = unsafe { WriteProcessMemory(GetCurrentProcess(), addr as *mut c_void, b.as_ptr() as *const c_void, 4, &mut written) };
    ok != 0 && written == 4
}

/// Watches another process so a crash is noticed the same frame.
pub struct ProcessWatch {
    pid: u32,
    handle: *mut c_void,
}

const SYNCHRONIZE: u32 = 0x0010_0000;

impl ProcessWatch {
    pub fn new() -> Self {
        Self { pid: 0, handle: core::ptr::null_mut() }
    }

    /// Follows `pid` (0 = none). Returns true if the watched process has exited.
    pub fn exited(&mut self, pid: u32) -> bool {
        if pid != self.pid {
            if !self.handle.is_null() {
                unsafe { CloseHandle(self.handle) };
            }
            self.pid = pid;
            self.handle = if pid == 0 { core::ptr::null_mut() } else { unsafe { OpenProcess(SYNCHRONIZE, 0, pid) } };
        }
        // A pid we can't open is treated as gone.
        self.pid != 0 && (self.handle.is_null() || unsafe { WaitForSingleObject(self.handle, 0) } == 0)
    }
}
