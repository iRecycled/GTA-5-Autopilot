//! CAN for the virtual vehicle (D9, D17, D23).
//!
//! - [`dbc`]: parse DBC files, look messages and signals up by name, and pack
//!   and unpack signals (Intel and Motorola byte order, signed, scaled).
//! - [`topology`]: `config/topology.toml`: buses, ECUs and which bus each is on.
//! - [`toyota`]: the Toyota checksum and counter conventions the TSS2 DBC uses.
//! - [`bus`]: a timing model of one bus: frame length from the real bitstream
//!   (stuff bits and CRC), CAN FD's faster data phase, arbitration by id, bus
//!   load, and `reactive` or `realistic` delivery.

pub mod bus;
pub mod dbc;
pub mod topology;
pub mod toyota;

pub use bus::{Bus, BusConfig, Delivery, Frame};
pub use dbc::{Dbc, Message, Signal};
pub use topology::Topology;
