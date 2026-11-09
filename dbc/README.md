# DBC files

One file per CAN bus (D9, D17).

| File | Bus | Source |
| --- | --- | --- |
| `toyota_nodsu_pt_generated.dbc` | Chassis CAN (500 kbit/s) | [commaai/opendbc](https://github.com/commaai/opendbc), `opendbc/dbc/generator/toyota/toyota_nodsu_pt.dbc` with its imports (`_toyota_2017.dbc`, `_toyota_adas_standard.dbc`) concatenated the way opendbc's `generator.py` does. MIT licence: `LICENSE.opendbc`. It carries `STEERING_LTA` (0x191), the angle-steering command D28 relies on |

| `adas_canfd.dbc` | ADAS CAN FD (500 kbit/s nominal, 2 Mbit/s data) | Our own layout. It carries the front radar's object list: one `RADAR_STATUS` per scan, then one 16-byte `RADAR_OBJECT` per object |

The bus topology (which ECU sits on which bus) is in `config/topology.toml`.

Don't edit the generated file by hand. Regenerate it from opendbc instead.
