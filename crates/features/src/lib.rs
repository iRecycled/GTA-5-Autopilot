//! Driver-assistance features. Each feature is a small state machine fed by
//! fused objects and the ego state; it outputs a request (a target speed, a
//! brake demand, a warning) that the core arbitrates. Nothing here knows the
//! host game or the sensor set behind the objects.

pub mod acc;
pub mod aeb;

pub use acc::{Acc, AccConfig};
pub use aeb::{Aeb, AebConfig, AebStage};
