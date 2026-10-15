# Run report: seed 0

Faults: `none`  
Scenario: gap0 42.06 m, lead brakes at 3.21 s with 8.32 m/s^2, ego/lead 72 km/h.  
Frames sent 2490, received over python-can virtual bus 2490.  
Estimated load at 500 kbit/s: 6.09% (no stuffing) to 7.41% (worst-case stuffing).

## Cycle time per message

| message | nominal ms | frames | mean ms | p95 ms | max ms | missing | timing violations |
|---|---:|---:|---:|---:|---:|---:|---:|
| WHEEL_SPEEDS | 10 | 900 | 10.000 | 10.000 | 10.000 | 0 | 0 |
| SPEED | 20 | 450 | 20.000 | 20.000 | 20.000 | 0 | 0 |
| BRAKE_MODULE | 25 | 360 | 25.000 | 25.000 | 25.000 | 0 | 0 |
| ACC_CONTROL | 30 | 300 | 30.000 | 30.000 | 30.000 | 0 | 0 |
| PRE_COLLISION | 30 | 300 | 30.000 | 30.000 | 30.000 | 0 | 0 |
| LEAD_INFO | 50 | 180 | 50.000 | 50.000 | 50.000 | 0 | 0 |

## Findings by kind

| kind | count |
|---|---:|
| (none) | 0 |

## Injected faults vs detected (event level)

| fault | injected | detected | recall |
|---|---:|---:|---:|
| drop | 0 | 0 | n/a |
| delay | 0 | 0 | n/a |
| range | 0 | 0 | n/a |
| dlc | 0 | 0 | n/a |
| counter | 0 | 0 | n/a |
| checksum | 0 | 0 | n/a |
| stuck | 0 | 0 | n/a |

## AEB latency

Trigger = first LEAD_INFO frame with TTC < 1.8 s (TTC = LEAD_LONG_DIST / -LEAD_REL_SPEED). Request = first PRE_COLLISION frame with PRECOLLISION_ACTIVE=1 and FORCE<0. Pressure = first BRAKE_MODULE frame with BRAKE_PRESSURE >= 200. Ground truth uses the simulator's continuous TTC crossing (1 ms resolution).

| metric | bus measured ms | ground truth ms | error ms |
|---|---:|---:|---:|
| trigger_to_request_ms | 22.000 | 41.000 | -19.000 |
| trigger_to_pressure_ms | 195.000 | 203.000 | -8.000 |

## ASC / BLF round trip (python-can writer -> python-can reader)

| format | written | read | id mismatch | data mismatch | dlc mismatch | max dt us | ok |
|---|---:|---:|---:|---:|---:|---:|---|
| asc | 2490 | 2490 | 0 | 0 | 0 | 0.212 | True |
| blf | 2490 | 2490 | 0 | 0 | 0 | 0.000 | True |

Cycle times are assumptions from `canbus_analyzer/config.py` (the DBC has no GenMsgCycleTime attributes): WHEEL_SPEEDS 10 ms, SPEED 20 ms, BRAKE_MODULE 25 ms, ACC_CONTROL 30 ms, PRE_COLLISION 30 ms, LEAD_INFO 50 ms.
