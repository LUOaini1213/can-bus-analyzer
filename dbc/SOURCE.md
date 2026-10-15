# DBC source

- Repository: https://github.com/commaai/opendbc
- Commit: `0a1645dde38d074059bdd27fce3fda235407ca71` (master on 2026-10-14)
- Licence: MIT, Copyright (c) 2020, Comma.ai, Inc. (copy: `upstream/LICENSE.opendbc`)

Vendored verbatim (unmodified) in `upstream/`:

| file | upstream path | sha256 |
|---|---|---|
| toyota_nodsu_pt.dbc | https://github.com/commaai/opendbc/blob/0a1645dde38d074059bdd27fce3fda235407ca71/opendbc/dbc/generator/toyota/toyota_nodsu_pt.dbc | `d513d18b0c0337bbfeb6cb131f071d38dd7e47a05bdb9751b87bfdfbc6d0142d` |
| _toyota_2017.dbc | https://github.com/commaai/opendbc/blob/0a1645dde38d074059bdd27fce3fda235407ca71/opendbc/dbc/generator/toyota/_toyota_2017.dbc | `818872b1c1034983f61a51460a9b175b4bfe01f3d34f537c3d21f420991a8a48` |
| _toyota_adas_standard.dbc | https://github.com/commaai/opendbc/blob/0a1645dde38d074059bdd27fce3fda235407ca71/opendbc/dbc/generator/toyota/_toyota_adas_standard.dbc | `432ca7e0c0e42d05030042f14f0c9fd1036fed44e90f297465c51fa45032aa34` |
| LICENSE.opendbc | https://github.com/commaai/opendbc/blob/0a1645dde38d074059bdd27fce3fda235407ca71/LICENSE | `6543c4a66b46affddae1b9ae1e1992687f0b7191e74ba3a7235388b9f42a6d13` |

`toyota_nodsu_pt_generated.dbc` is what opendbc's own generator
(`opendbc/dbc/generator/generator.py`) produces from `toyota_nodsu_pt.dbc`: it
concatenates the files named in its `CM_ "IMPORT ...";` lines in front of the
template. opendbc does not commit generated files, so `scripts/build_dbc.py`
re-implements that concatenation; `tests/test_dbc_decode.py` checks the vendored
file is byte-identical to the rebuild.

sha256 of the generated file: `29ee5c92017b46b8db2fe7566e6f765fe52efcddfb3b426dff929c879d70e066`

The DBC carries no `GenMsgCycleTime` attributes; cycle times used here are
assumptions set in `canbus_analyzer/config.py`.
