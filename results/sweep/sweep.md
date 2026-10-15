# Sweep

## Detection vs fault intensity

| fault | level | injected | detected | recall | runs with detection | spurious findings (other kinds) | request-latency error mean / p95 abs (ms) |
|---|---:|---:|---:|---:|---:|---|---:|
| none | 0 | - | - | - | - | 0 | -21.2 / 43.3 |
| drop | 0.001 | 56 | 56 | 1.00 | 19/20 | 0 | -21.2 / 43.3 |
| drop | 0.005 | 240 | 240 | 1.00 | 20/20 | 0 | -21.2 / 43.3 |
| drop | 0.01 | 481 | 481 | 1.00 | 20/20 | 0 | -20.7 / 43.3 |
| drop | 0.05 | 2489 | 2479 | 1.00 | 20/20 | 0 | -21.2 / 43.3 |
| drop | 0.1 | 4933 | 4899 | 0.99 | 20/20 | 0 | -23.2 / 51.0 |
| jitter | 0.25 | - | - | - | 0/20 | 0 | -21.2 / 43.6 |
| jitter | 0.5 | - | - | - | 19/20 | 0 | -21.2 / 43.8 |
| jitter | 1 | - | - | - | 20/20 | missing 1 | -21.7 / 44.4 |
| jitter | 2 | - | - | - | 20/20 | missing 676 | -21.5 / 41.4 |
| jitter | 4 | - | - | - | 20/20 | missing 3565, stuck 1 | -22.6 / 41.5 |
| jitter | 8 | - | - | - | 20/20 | counter 55, missing 7950, stuck 3 | -21.3 / 49.3 |
| delay | 0.1 | 100 | 0 | 0.00 | 0/20 | 0 | -21.2 / 43.3 |
| delay | 0.15 | 100 | 0 | 0.00 | 0/20 | 0 | -21.2 / 43.3 |
| delay | 0.25 | 100 | 100 | 1.00 | 20/20 | 0 | -21.2 / 43.3 |
| delay | 0.4 | 100 | 100 | 1.00 | 20/20 | 0 | -21.2 / 43.3 |
| delay | 0.6 | 100 | 98 | 0.98 | 20/20 | missing 98 | -21.2 / 43.3 |
| range | 1 | 20 | 20 | 1.00 | 20/20 | 0 | -21.2 / 43.3 |
| range | 5 | 100 | 100 | 1.00 | 20/20 | 0 | -21.2 / 43.3 |
| range | 20 | 400 | 400 | 1.00 | 20/20 | 0 | -22.2 / 43.3 |
| dlc | 1 | 20 | 20 | 1.00 | 20/20 | 0 | -21.2 / 43.3 |
| dlc | 5 | 100 | 100 | 1.00 | 20/20 | 0 | -21.2 / 43.3 |
| dlc | 20 | 400 | 400 | 1.00 | 20/20 | 0 | -21.7 / 44.3 |
| counter | 1 | 20 | 20 | 1.00 | 20/20 | 0 | -21.2 / 43.3 |
| counter | 5 | 100 | 100 | 1.00 | 20/20 | 0 | -21.2 / 43.3 |
| counter | 20 | 400 | 400 | 1.00 | 20/20 | 0 | -21.2 / 43.3 |
| checksum | 1 | 20 | 20 | 1.00 | 20/20 | 0 | -21.2 / 43.3 |
| checksum | 5 | 100 | 100 | 1.00 | 20/20 | 0 | -21.2 / 43.3 |
| checksum | 20 | 400 | 400 | 1.00 | 20/20 | 0 | -19.7 / 43.3 |
| stuck | 0.1 | 20 | 0 | 0.00 (observable 0/4) | 0/20 | 0 | -21.2 / 43.3 |
| stuck | 0.25 | 20 | 4 | 0.20 (observable 4/4) | 4/20 | 0 | -21.2 / 43.3 |
| stuck | 0.5 | 20 | 10 | 0.50 (observable 10/10) | 10/20 | 0 | -21.2 / 43.3 |
| stuck | 1 | 20 | 6 | 0.30 (observable 6/6) | 6/20 | 0 | -21.2 / 43.3 |
| stuck | 2 | 20 | 15 | 0.75 (observable 15/15) | 15/20 | 0 | -21.2 / 43.3 |

20 seeds per row. Collision in 6 of 20 clean scenarios (toy plant, not tuned).

## Latency measurement vs bus jitter

| jitter sigma (ms) | req err mean | req err sd | req p95 abs | req max abs | prs err mean | prs p95 abs | req vs no-jitter p95 abs | prs vs no-jitter p95 abs | trigger lag mean | missing req |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | -21.2 | 13.7 | 43.3 | 49.0 | -8.2 | 32.1 | 0.0 | 0.0 | 21.2 | 0 |
| 0.25 | -21.2 | 13.8 | 43.6 | 49.1 | -8.3 | 32.1 | 0.8 | 0.6 | 21.2 | 0 |
| 0.5 | -21.2 | 13.8 | 43.8 | 49.3 | -8.1 | 33.0 | 1.1 | 1.2 | 21.2 | 0 |
| 1 | -21.7 | 14.1 | 44.4 | 50.3 | -8.4 | 33.6 | 2.9 | 2.2 | 21.3 | 0 |
| 2 | -21.5 | 13.2 | 41.4 | 49.0 | -8.0 | 31.6 | 3.2 | 4.6 | 21.4 | 0 |
| 4 | -22.6 | 12.8 | 41.5 | 57.2 | -8.4 | 29.6 | 8.4 | 8.3 | 21.2 | 0 |
| 8 | -21.3 | 19.4 | 49.3 | 52.7 | -7.6 | 40.4 | 23.6 | 20.3 | 20.6 | 0 |

20 seeds per row; all values in ms. err = bus-measured latency minus simulator ground truth; 'vs no-jitter' = same seed, bus-measured with minus without jitter; trigger lag = bus-visible trigger (LEAD_INFO frame) minus physical TTC crossing.
