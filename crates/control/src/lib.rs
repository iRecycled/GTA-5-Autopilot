//! Sim-neutral vehicle controllers.
//!
//! Inputs and outputs are SI units in ISO 8855 vehicle coordinates. Nothing here
//! knows which host game is running: host quirks (pedal deadzones, how a
//! standstill hold is applied) belong to that host's adapter (D22).

pub mod lateral;
pub mod longitudinal;

pub use longitudinal::{Config, Output, SpeedController};
