//! Driver-assistance features. Each feature is a small state machine fed by
//! fused objects and the ego state; it outputs a request (a target speed, a
//! brake demand, a warning) that the core arbitrates. Nothing here knows the
//! host game or the sensor set behind the objects.

pub mod acc;
pub mod aeb;
pub mod bsd;
pub mod ego;
pub mod junction;
pub mod lanes;
pub mod park;
pub mod rcta;

pub use acc::{Acc, AccConfig};
pub use aeb::{Aeb, AebConfig, AebStage};
pub use bsd::{Bsd, BsdConfig, BsdOut};
pub use ego::Ego;
pub use junction::{JunctionConfig, select_crossing};
pub use lanes::{LaneConfig, LaneIssue, LaneOut, LanePath, Lanes, SteerResponse};
pub use park::{ParkAssist, ParkConfig, ParkOut};
pub use rcta::{Rcta, RctaConfig, RctaOut};
