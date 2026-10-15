# Run report: seed 0

Faults: `{"drop_prob": 0.01, "jitter_ms": 1.0, "delay_n": 3, "range_n": 3, "dlc_n": 3, "counter_n": 3, "checksum_n": 3, "stuck_s": 1.0}`  
Scenario: gap0 42.06 m, lead brakes at 3.21 s with 8.32 m/s^2, ego/lead 72 km/h.  
Frames sent 2463, received over python-can virtual bus 2463.  
Estimated load at 500 kbit/s: 6.02% (no stuffing) to 7.33% (worst-case stuffing).

## Cycle time per message

| message | nominal ms | frames | mean ms | p95 ms | max ms | missing | timing violations |
|---|---:|---:|---:|---:|---:|---:|---:|
| WHEEL_SPEEDS | 10 | 889 | 10.124 | 12.456 | 21.649 | 11 | 142 |
| SPEED | 20 | 446 | 20.178 | 22.340 | 42.370 | 4 | 3 |
| BRAKE_MODULE | 25 | 356 | 25.281 | 27.312 | 50.713 | 4 | 0 |
| ACC_CONTROL | 30 | 298 | 30.204 | 32.500 | 60.525 | 2 | 0 |
| PRE_COLLISION | 30 | 298 | 30.199 | 32.388 | 62.108 | 2 | 0 |
| LEAD_INFO | 50 | 176 | 51.142 | 52.708 | 100.785 | 4 | 0 |

## Findings by kind

| kind | count |
|---|---:|
| checksum | 3 |
| counter | 6 |
| dlc | 3 |
| missing | 27 |
| range | 3 |
| timing | 145 |

## Injected faults vs detected (event level)

| fault | injected | detected | recall |
|---|---:|---:|---:|
| drop | 27 | 27 | 1.000 |
| delay | 3 | 3 | 1.000 |
| range | 3 | 3 | 1.000 |
| dlc | 3 | 3 | 1.000 |
| counter | 3 | 3 | 1.000 |
| checksum | 3 | 3 | 1.000 |
| stuck | 1 | 0 | 0.000 |

## AEB latency

Trigger = first LEAD_INFO frame with TTC < 1.8 s (TTC = LEAD_LONG_DIST / -LEAD_REL_SPEED). Request = first PRE_COLLISION frame with PRECOLLISION_ACTIVE=1 and FORCE<0. Pressure = first BRAKE_MODULE frame with BRAKE_PRESSURE >= 200. Ground truth uses the simulator's continuous TTC crossing (1 ms resolution).

| metric | bus measured ms | ground truth ms | error ms |
|---|---:|---:|---:|
| trigger_to_request_ms | 23.466 | 41.000 | -17.534 |
| trigger_to_pressure_ms | 194.486 | 203.000 | -8.514 |

## ASC / BLF round trip (python-can writer -> python-can reader)

| format | written | read | id mismatch | data mismatch | dlc mismatch | max dt us | ok |
|---|---:|---:|---:|---:|---:|---:|---|
| asc | 2463 | 2463 | 0 | 0 | 0 | 0.132 | True |
| blf | 2463 | 2463 | 0 | 0 | 0 | 0.000 | True |

Cycle times are assumptions from `canbus_analyzer/config.py` (the DBC has no GenMsgCycleTime attributes): WHEEL_SPEEDS 10 ms, SPEED 20 ms, BRAKE_MODULE 25 ms, ACC_CONTROL 30 ms, PRE_COLLISION 30 ms, LEAD_INFO 50 ms.
