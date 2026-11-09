//! The bus topology (D17): `config/topology.toml` names every bus (kind, bit
//! rates, DBC) and every ECU with the buses it sits on.

use serde::Deserialize;

use crate::bus::{BusConfig, Delivery};

#[derive(Debug, Deserialize, Clone, PartialEq)]
pub struct BusSpec {
    pub name: String,
    /// "classic" or "fd".
    pub kind: String,
    pub nominal_bps: u32,
    pub data_bps: Option<u32>,
    pub dbc: String,
}

#[derive(Debug, Deserialize, Clone, PartialEq)]
pub struct EcuSpec {
    pub name: String,
    pub buses: Vec<String>,
}

#[derive(Debug, Deserialize, Clone, PartialEq)]
pub struct Topology {
    #[serde(rename = "bus")]
    pub buses: Vec<BusSpec>,
    #[serde(rename = "ecu")]
    pub ecus: Vec<EcuSpec>,
}

impl Topology {
    pub fn parse(text: &str) -> Result<Self, String> {
        let t: Topology = toml::from_str(text).map_err(|e| e.to_string())?;
        for e in &t.ecus {
            for b in &e.buses {
                if !t.buses.iter().any(|x| &x.name == b) {
                    return Err(format!("ECU {} is on unknown bus {b}", e.name));
                }
            }
        }
        for b in &t.buses {
            match (b.kind.as_str(), b.data_bps) {
                ("classic", None) | ("fd", Some(_)) => {}
                _ => return Err(format!("bus {}: kind {} with data_bps {:?}", b.name, b.kind, b.data_bps)),
            }
        }
        Ok(t)
    }

    pub fn bus(&self, name: &str) -> Option<&BusSpec> {
        self.buses.iter().find(|b| b.name == name)
    }

    pub fn bus_config(&self, name: &str, delivery: Delivery) -> Option<BusConfig> {
        self.bus(name).map(|b| BusConfig { name: b.name.clone(), nominal_bps: b.nominal_bps, data_bps: b.data_bps, delivery })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_project_topology_is_valid() {
        let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../config/topology.toml")).unwrap();
        let t = Topology::parse(&text).unwrap();
        assert_eq!(t.buses.len(), 2);
        let adas = t.ecus.iter().find(|e| e.name == "adas").unwrap();
        assert_eq!(adas.buses, vec!["chassis", "adas"], "the ADAS ECU sits on both buses (D9)");
        assert_eq!(t.bus("adas").unwrap().data_bps, Some(2_000_000));
    }

    #[test]
    fn rejects_an_ecu_on_a_missing_bus() {
        let t = "[[bus]]\nname='a'\nkind='classic'\nnominal_bps=500000\ndbc='x'\n[[ecu]]\nname='e'\nbuses=['b']\n";
        assert!(Topology::parse(t).is_err());
    }
}
