//! The adapter contract (D22): versioned `#[repr(C)]` messages exchanged
//! through shared memory between a host adapter (inside the game) and the core
//! (a separate process).
//!
//! The contract is the memory layout, not a Rust trait, so an adapter can be
//! written in any language. Positions are ISO 8855 vehicle coordinates and SI
//! units (D16); each adapter converts from its game once.
//!
//! This first version carries the `sample` and `command` channels only (spike
//! S3); the others in `docs/implementation-plan.md` follow.

pub mod mailbox;
pub mod msg;
#[cfg(windows)]
pub mod shm;

pub use mailbox::Mailbox;
pub use msg::*;

/// Bumped on any change to a message layout or meaning.
pub const CONTRACT_VERSION: u16 = 1;

/// Monotonic wall clock in ns (QueryPerformanceCounter on Windows). Both sides
/// of the link read the same clock, so timestamps compare across processes.
#[cfg(windows)]
pub fn wall_time_ns() -> u64 {
    shm::qpc_ns()
}
