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

pub fn write_f32(addr: usize, v: f32) -> bool {
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
