//! Raw bindings to Script Hook V for GTA V Enhanced.
//!
//! Script Hook V exports C++ functions, so the symbols are MSVC-mangled. The
//! names below were read from the export table of `ScriptHookV.dll`
//! v3889.0/1158.13. `raw-dylib` lets us link without the SDK's import library.
//!
//! Everything here may only be called from a script fiber registered with
//! [`scriptRegister`], except `scriptRegister`/`scriptUnregister` themselves,
//! which are called from `DllMain`.

#![allow(non_snake_case)]

use core::ffi::{c_char, c_void};

/// `HMODULE` of the plugin DLL.
pub type Hmodule = *mut c_void;

/// Script entry point. Runs on a fiber; must loop forever, calling [`scriptWait`].
pub type ScriptMain = extern "C" fn();

#[link(name = "ScriptHookV", kind = "raw-dylib")]
unsafe extern "C" {
    #[link_name = "?scriptRegister@@YAXPEAUHINSTANCE__@@P6AXXZ@Z"]
    pub fn scriptRegister(module: Hmodule, main: ScriptMain);

    #[link_name = "?scriptRegisterAdditionalThread@@YAXPEAUHINSTANCE__@@P6AXXZ@Z"]
    pub fn scriptRegisterAdditionalThread(module: Hmodule, main: ScriptMain);

    #[link_name = "?scriptUnregister@@YAXPEAUHINSTANCE__@@@Z"]
    pub fn scriptUnregister(module: Hmodule);

    /// Yields the script fiber. `0` means "until the next game frame".
    #[link_name = "?scriptWait@@YAXK@Z"]
    pub fn scriptWait(ms: u32);

    #[link_name = "?nativeInit@@YAX_K@Z"]
    pub fn nativeInit(hash: u64);

    #[link_name = "?nativePush64@@YAX_K@Z"]
    pub fn nativePush64(value: u64);

    /// Calls the native set up by `nativeInit`/`nativePush64`; returns a
    /// pointer to the result buffer.
    #[link_name = "?nativeCall@@YAPEA_KXZ"]
    pub fn nativeCall() -> *mut u64;

    #[link_name = "?nativeCanExecuteInThisContext@@YA_NXZ"]
    pub fn nativeCanExecuteInThisContext() -> bool;

    #[link_name = "?worldGetAllVehicles@@YAHPEAHH@Z"]
    pub fn worldGetAllVehicles(out: *mut i32, max: i32) -> i32;

    #[link_name = "?worldGetAllPeds@@YAHPEAHH@Z"]
    pub fn worldGetAllPeds(out: *mut i32, max: i32) -> i32;

    #[link_name = "?worldGetAllObjects@@YAHPEAHH@Z"]
    pub fn worldGetAllObjects(out: *mut i32, max: i32) -> i32;

    #[link_name = "?getScriptHandleBaseAddress@@YAPEAEH@Z"]
    pub fn getScriptHandleBaseAddress(handle: i32) -> *mut u8;

    #[link_name = "?getGlobalPtr@@YAPEA_KH@Z"]
    pub fn getGlobalPtr(index: i32) -> *mut u64;

    /// Returns Script Hook V's `eGameVersion` enum value.
    #[link_name = "?getGameVersion@@YA?AW4eGameVersion@@XZ"]
    pub fn getGameVersion() -> i32;

    #[link_name = "?scriptsAreLaunchedUsingReloading@@YA_NXZ"]
    pub fn scriptsAreLaunchedUsingReloading() -> bool;
}

/// GTA's script `Vector3`: three floats, each padded to 8 bytes.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vector3 {
    pub x: f32,
    _pad_x: u32,
    pub y: f32,
    _pad_y: u32,
    pub z: f32,
    _pad_z: u32,
}

impl Vector3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, _pad_x: 0, y, _pad_y: 0, z, _pad_z: 0 }
    }
}

/// A value that can be pushed as a native argument. Matches the SDK's
/// `nativePush`: the value is stored in the low bytes of a zeroed `u64`.
pub trait Arg {
    fn to_u64(self) -> u64;
}

impl Arg for i32 {
    fn to_u64(self) -> u64 {
        self as u32 as u64
    }
}
impl Arg for u32 {
    fn to_u64(self) -> u64 {
        self as u64
    }
}
impl Arg for u64 {
    fn to_u64(self) -> u64 {
        self
    }
}
impl Arg for bool {
    fn to_u64(self) -> u64 {
        self as u64
    }
}
impl Arg for f32 {
    fn to_u64(self) -> u64 {
        self.to_bits() as u64
    }
}
impl<T> Arg for *const T {
    fn to_u64(self) -> u64 {
        self as usize as u64
    }
}
impl<T> Arg for *mut T {
    fn to_u64(self) -> u64 {
        self as usize as u64
    }
}
impl Arg for &core::ffi::CStr {
    fn to_u64(self) -> u64 {
        self.as_ptr() as *const c_char as usize as u64
    }
}

/// A native return type, read from the result buffer.
pub trait Ret {
    /// # Safety
    /// `p` must be the pointer returned by [`nativeCall`].
    unsafe fn read(p: *const u64) -> Self;
}

impl Ret for () {
    unsafe fn read(_: *const u64) {}
}
impl Ret for i32 {
    unsafe fn read(p: *const u64) -> Self {
        unsafe { *(p as *const i32) }
    }
}
impl Ret for bool {
    unsafe fn read(p: *const u64) -> Self {
        unsafe { *(p as *const i32) != 0 }
    }
}
impl Ret for u32 {
    unsafe fn read(p: *const u64) -> Self {
        unsafe { *(p as *const u32) }
    }
}
/// Natives returning `const char*` (may be null).
impl Ret for *const c_char {
    unsafe fn read(p: *const u64) -> Self {
        unsafe { *(p as *const *const c_char) }
    }
}
impl Ret for f32 {
    unsafe fn read(p: *const u64) -> Self {
        unsafe { *(p as *const f32) }
    }
}
impl Ret for Vector3 {
    unsafe fn read(p: *const u64) -> Self {
        unsafe { *(p as *const Vector3) }
    }
}

/// Calls a game native by hash. The return type is inferred.
///
/// ```ignore
/// let ped: i32 = unsafe { invoke!(0xD80958FC74E988A6) }; // PLAYER_PED_ID
/// ```
///
/// # Safety
/// Must run on a script fiber, with argument types matching the native.
#[macro_export]
macro_rules! invoke {
    ($hash:expr $(, $arg:expr)* $(,)?) => {{
        $crate::nativeInit($hash);
        $( $crate::nativePush64($crate::Arg::to_u64($arg)); )*
        $crate::Ret::read($crate::nativeCall())
    }};
}
