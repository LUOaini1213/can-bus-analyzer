# Function KPIs per scenario

| scenario | v0 km/h | TTC@FCW bus / true (s) | TTC@brake bus / true (s) | TTC@full bus (s) | FCW→brake (ms) | contact | v_rel impact bus / true (km/h) | fault→FAULT bus / true (ms) | states on bus |
|---|---:|---:|---:|---:|---:|---|---:|---:|---|
| CCRs-10 | 10 | 2.60 / 2.56 | 1.59 / 1.58 | - | 982 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > STANDBY |
| CCRs-15 | 15 | 2.58 / 2.56 | 1.58 / 1.56 | - | 998 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > STANDBY |
| CCRs-20 | 20 | 2.56 / 2.54 | 1.56 / 1.53 | - | 1007 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > STANDBY |
| CCRs-25 | 25 | 2.60 / 2.58 | 1.60 / 1.58 | - | 996 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > STANDBY |
| CCRs-30 | 30 | 2.60 / 2.56 | 1.59 / 1.57 | - | 997 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > STANDBY |
| CCRs-35 | 35 | 2.57 / 2.56 | 1.57 / 1.54 | 0.90 | 1021 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRs-40 | 40 | 2.58 / 2.57 | 1.58 / 1.55 | 0.88 | 1021 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRs-45 | 45 | 2.56 / 2.54 | 1.56 / 1.55 | 0.89 | 990 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRs-50 | 50 | 2.59 / 2.58 | 1.59 / 1.57 | 0.88 | 1010 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRs-55 | 55 | 2.60 / 2.59 | 1.60 / 1.59 | 0.90 | 997 | no | - / - | - / - | STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRs-60 | 60 | 2.57 / 2.56 | 1.57 / 1.56 | 0.87 | 997 | yes | -0.2 / 4.4 | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRs-65 | 65 | 2.56 / 2.54 | 1.56 / 1.52 | 0.87 | 1011 | yes | 19.9 / 20.1 | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRs-70 | 70 | 2.57 / 2.56 | 1.57 / 1.55 | 0.88 | 1011 | yes | 26.1 / 26.2 | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRs-75 | 75 | 2.58 / 2.55 | 1.58 / 1.56 | 0.88 | 990 | yes | 32.9 / 33.0 | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRs-80 | 80 | 2.56 / 2.54 | 1.56 / 1.55 | 0.89 | 986 | yes | 39.7 / 39.2 | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRm-30 | 30 | 2.56 / 2.53 | 1.55 / 1.54 | - | 983 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > STANDBY |
| CCRm-35 | 35 | 2.56 / 2.56 | 1.57 / 1.56 | - | 991 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > STANDBY |
| CCRm-40 | 40 | 2.56 / 2.53 | 1.56 / 1.52 | - | 1006 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > STANDBY |
| CCRm-45 | 45 | 2.58 / 2.55 | 1.58 / 1.58 | - | 976 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > STANDBY |
| CCRm-50 | 50 | 2.60 / 2.58 | 1.59 / 1.57 | - | 1011 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > STANDBY |
| CCRm-55 | 55 | 2.55 / 2.53 | 1.60 / 1.57 | 0.90 | 957 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRm-60 | 60 | 2.57 / 2.56 | 1.57 / 1.55 | 0.90 | 1005 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRm-65 | 65 | 2.56 / 2.52 | 1.56 / 1.53 | 0.87 | 989 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRm-70 | 70 | 2.58 / 2.56 | 1.59 / 1.57 | 0.87 | 997 | no | - / - | - / - | STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRm-75 | 75 | 2.60 / 2.59 | 1.60 / 1.58 | 0.89 | 1005 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRm-80 | 80 | 2.57 / 2.55 | 1.57 / 1.56 | 0.88 | 986 | yes | 4.7 / 4.4 | - / - | STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRb-12m-2 | 50 | 2.52 / 2.46 | 1.51 / 1.49 | 0.87 | 491 | yes | 0.0 / 1.3 | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY > FCW > PARTIAL_BRAKE > STANDBY |
| CCRb-12m-6 | 50 | 2.43 / 2.38 | 1.59 / 1.51 | 0.86 | 267 | yes | 20.6 / 21.0 | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRb-40m-2 | 50 | 2.56 / 2.55 | 1.55 / 1.51 | 0.89 | 724 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| CCRb-40m-6 | 50 | 2.55 / 2.54 | 1.58 / 1.55 | 0.90 | 575 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| NT-01 | 60 | - / - | - / - | - | - | no | - / - | - / - | OFF > STANDBY |
| NT-02 | 70 | - / - | - / - | - | - | no | - / - | - / - | OFF > STANDBY |
| NT-03 | 50 | - / - | - / - | - | - | no | - / - | - / - | OFF > STANDBY |
| OVR-01 | 50 | - / - | - / - | - | - | yes | 50.0 / 50.0 | - / - | OFF > STANDBY |
| OVR-02 | 50 | 2.55 / 2.54 | 1.55 / 1.55 | - | 990 | yes | 46.1 / 46.1 | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > STANDBY |
| DRV-01 | 50 | 2.57 / 2.56 | 1.57 / 1.57 | 0.89 | 988 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| OFF-01 | 30 | - / - | - / - | - | - | yes | 30.0 / 30.0 | - / - | OFF |
| FLT-01 | 50 | - / - | - / - | - | - | n/a | - / 50.0 | 132 / 104 | OFF > STANDBY > FAULT |
| FLT-02 | 60 | 2.60 / 2.59 | 1.59 / 1.58 | - | 1014 | n/a | - / 56.5 | 115 / 101 | OFF > STANDBY > FCW > PARTIAL_BRAKE > FAULT |
| FLT-03 | 40 | 2.56 / 2.54 | 1.56 / 1.53 | 0.90 | 1007 | no | - / - | 29 / 27 | OFF > STANDBY > FAULT > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |
| FLT-04 | 40 | 2.59 / 2.56 | 1.59 / 1.57 | 0.88 | 992 | no | - / - | - / - | OFF > STANDBY > FCW > PARTIAL_BRAKE > FULL_BRAKE > STANDBY |

Bus minus truth: TTC at FCW 0.01 to 0.07 s (n=35); TTC at brake request 0.00 to 0.08 s (n=35); impact speed -4.42 to 0.55 km/h (n=11); fault reaction 28.0, 14.0, 2.0 ms; contact verdict differs in 0 scenario(s); not observable from the bus in FLT-01, FLT-02 (LEAD_INFO stopped; the simulator reports contact there).
