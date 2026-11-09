//! Generates the vehicle's CAN message types from its DBC files (Phase 1).

use std::path::PathBuf;

fn main() {
    let out = PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR"));
    for (dbc, file) in [("../../dbc/toyota_nodsu_pt_generated.dbc", "chassis.rs"), ("../../dbc/adas_canfd.dbc", "adas.rs")] {
        println!("cargo:rerun-if-changed={dbc}");
        let text = std::fs::read_to_string(dbc).unwrap_or_else(|e| panic!("{dbc}: {e}"));
        let parsed = can::Dbc::parse(&text).unwrap_or_else(|e| panic!("{dbc}: {e}"));
        let name = dbc.rsplit('/').next().unwrap_or(dbc);
        let code = can::codegen::generate(&parsed, name).unwrap_or_else(|e| panic!("{dbc}: {e}"));
        std::fs::write(out.join(file), code).expect("write generated messages");
    }
}
