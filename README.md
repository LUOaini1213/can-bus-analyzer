# can-bus-analyzer

用真实量产车 DBC（commaai/opendbc 的 Toyota 定义）生成一个合成 AEB（自动紧急制动）场景的 CAN 报文，按周期在 python-can 的进程内虚拟总线上回放，注入常见总线/信号故障，再用 cantools 解码做周期统计、信号合法性检查和 AEB 链路时延测量；最后把录到的报文导出成 Vector 的 ASC / BLF 日志格式并读回逐帧比对。

## 范围（事实说明）

- 场景是合成的玩具模型（自车巡航、前车急刹、简化的雷达/DSU 决策与一阶制动执行器），不是整车模型，也没有接真实 ECU。
- 总线是 python-can 的 `virtual` 接口（进程内队列）。它不模拟仲裁、位定时、总线负载或错误帧；所有时序故障都由故障注入器在逻辑时钟上产生。表中的"总线负载"是按经典 CAN 2.0A 帧长公式估算的。
- 只用了经典 CAN（每帧最多 8 字节）。没有实现 CAN FD 和 FlexRay。
- ASC / BLF 文件由 python-can 的 writer 写出、python-can 的 reader 读回；**没有在 CANalyzer / CANoe 里打开过**（没有许可证）。
- opendbc 的 DBC 不含 `GenMsgCycleTime` 属性，所以各报文周期是本项目在 `canbus_analyzer/config.py` 里设定的假设值：WHEEL_SPEEDS 10 ms、SPEED 20 ms、BRAKE_MODULE 25 ms、ACC_CONTROL 30 ms、PRE_COLLISION 30 ms、LEAD_INFO 50 ms。

## 用到的 DBC 报文

DBC 来源与许可见 [dbc/SOURCE.md](dbc/SOURCE.md)。

| 报文 (ID) | 用到的信号 | 校验 |
|---|---|---|
| WHEEL_SPEEDS (0xAA) | 四轮轮速 | 无 |
| SPEED (0xB4) | SPEED | Toyota CHECKSUM |
| BRAKE_MODULE (0x226) | BRAKE_PRESSURE（原始值，无物理单位） | 无 |
| LEAD_INFO (0x2E6) | LEAD_LONG_DIST, LEAD_REL_SPEED → TTC | Toyota CHECKSUM |
| ACC_CONTROL (0x343) | ACCEL_CMD | Toyota CHECKSUM |
| PRE_COLLISION (0x283) | PRECOLLISION_ACTIVE, FORCE, STATE, _COUNTER | Toyota CHECKSUM + 8 位滚动计数 |

- Checksum 算法与 opendbc `toyota_checksum()` 相同（长度 + 地址各字节 + 除校验字节外的数据字节，取低 8 位）。
- "AEB 制动请求"沿用 openpilot carstate 的判据：`PRECOLLISION_ACTIVE == 1 且 FORCE < 0`。DBC 给 FORCE 的范围是 `[0|255]`，与该判据矛盾，所以范围检查对 FORCE 用了覆盖值 `[-10000, 0]` N（写在 config 里）。
- `_COUNTER` 在 opendbc 里不叫 `COUNTER`，openpilot 自身并不校验它；本项目在合成报文里把它当 8 位滚动计数使用。

## 检查项

| 检查 | 规则 |
|---|---|
| 周期 / 丢帧 | 相邻到达间隔 > 1.5 T 记为丢帧（按 round(Δ/T)−1 计数）；否则 \|Δ−T\| > 0.2 T 记为时序违例；给出 mean / p95 / max |
| DLC | 数据长度 ≠ DBC 定义长度 |
| Checksum | 末字节 ≠ toyota_checksum |
| 范围 | 物理值超出 DBC [min, max]（或 config 覆盖值） |
| 滚动计数 | 步长须为 1（模 256）；步长 k>1 只有当时间间隔约为 k 个周期（即丢帧）时才接受 |
| 信号卡死 | SPEED 连续 ≥ 0.2 s 不变，而同一时间窗内四轮平均轮速变化 > 1 km/h |
| AEB 时延 | 触发 = 第一帧 TTC < 1.8 s 的 LEAD_INFO；到第一帧 AEB 请求的 PRE_COLLISION；到第一帧 BRAKE_PRESSURE ≥ 200 的 BRAKE_MODULE。与仿真器真值（连续 TTC 穿越时刻，1 ms 分辨率）对比 |

DLC / Checksum 不通过的帧不解码；范围不通过的帧不参与时延和卡死判断。

## 结果

以下表格由 `scripts/readme_check.py --write` 从 `results/` 生成，`tests/test_readme_numbers.py` 校验 README 与生成文件一致。

### 单次运行（seed 0）

<!-- BEGIN:run -->
| 项目 | 无故障 (seed 0) | 全部故障 (seed 0) |
|---|---:|---:|
| 发送 / 接收帧数 | 2490 / 2490 | 2463 / 2463 |
| 发现项 `checksum` | 0 | 3 |
| 发现项 `counter` | 0 | 6 |
| 发现项 `dlc` | 0 | 3 |
| 发现项 `missing` | 0 | 27 |
| 发现项 `range` | 0 | 3 |
| 发现项 `timing` | 0 | 145 |
| 注入 `drop` 检出 / 注入 | - | 27 / 27 |
| 注入 `delay` 检出 / 注入 | - | 3 / 3 |
| 注入 `range` 检出 / 注入 | - | 3 / 3 |
| 注入 `dlc` 检出 / 注入 | - | 3 / 3 |
| 注入 `counter` 检出 / 注入 | - | 3 / 3 |
| 注入 `checksum` 检出 / 注入 | - | 3 / 3 |
| 注入 `stuck` 检出 / 注入 | - | 0 / 1 |
| 总线测得 触发→制动请求 (ms) | 22.0 | 23.5 |
| 仿真真值 触发→制动请求 (ms) | 41.0 | 41.0 |
| 总线测得 触发→制动压力≥200 (ms) | 195.0 | 194.5 |
| 仿真真值 触发→制动压力≥200 (ms) | 203.0 | 203.0 |
| 500 kbit/s 估算负载 (%) 无填充–最坏填充 | 6.09–7.41 | 6.02–7.33 |
| ASC 回读逐帧一致 (帧数, 最大时间差 µs) | True (2490, 0.212) | True (2463, 0.132) |
| BLF 回读逐帧一致 (帧数, 最大时间差 µs) | True (2490, 0.000) | True (2463, 0.000) |
<!-- END:run -->

详细报告：[results/run_seed0_none/report.md](results/run_seed0_none/report.md)、[results/run_seed0_all/report.md](results/run_seed0_all/report.md)。

### 扫描 1：故障强度 × 检出（每行 20 个种子）

"recall" 是事件级：注入的每个故障是否被对应检查在正确的报文、正确的帧（或时间窗）上报出。"spurious" 是该行故障引出的其它类别发现项。最后一列是总线测得的"触发→制动请求"时延减去仿真真值。

<!-- BEGIN:sweep -->
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
<!-- END:sweep -->

### 扫描 2：时延测量误差 × 总线抖动（每行 20 个种子）

抖动是对所有帧时间戳加高斯噪声（σ 为表中值）。"vs no-jitter" 列把同一种子无抖动时的总线测量当基准，单独看抖动带来的变化。

<!-- BEGIN:latency -->
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
<!-- END:latency -->

### 读表

- 无故障时所有检查零误报。
- 时延误差主要来自采样，不是抖动：总线上能看到的"触发"是物理 TTC 穿越之后的下一帧 LEAD_INFO（50 ms 周期），所以总线测得的"触发→请求"系统性偏短，无抖动时其均值正好是 trigger lag 均值的相反数；无抖动时最大偏差小于一个雷达周期。抖动带来的额外变化（vs no-jitter 列）随 σ 增大，到 σ 最大一档才与采样误差同量级。
- 固定的 1.5 T 丢帧阈值对 10 ms 报文不耐抖动：σ = 2 ms 起就把抖动大量误报成丢帧，最大档还引出滚动计数误报。
- σ ≥ 4 ms 时出现少量"卡死"误报，全部出现在发生碰撞的种子里：玩具模型在碰撞瞬间把自车速度直接置为前车速度（0），SPEED 阶跃到 0，而抖动把碰撞前的一帧轮速排到了 SPEED=0 之后，交叉校验看到"SPEED 不变而轮速变了"。根源是两条报文之间的时间对齐，不是 SPEED 本身卡住。
- 不超过 0.2 T 的单帧延迟按设计看不见；延迟到 0.6 T 时间隔超过 1.5 T，被归类成"丢帧"而不是"时序违例"。
- 丢帧率高时 recall 略低于 1：漏掉的都是报文开头（包括从第一帧起连续丢的几帧）或最后一帧，前后没有参照帧，间隔检查无从发现（逐条核对过 drop 0.05 和 0.1 两行的全部漏检）。
- 卡死只在冻结窗口与车速真实变化（制动段）重叠时可观测；巡航段冻结的值本来就等于真实值，仅靠总线数据无法区分。
- 范围 / DLC / Checksum / 计数的 recall 全部为 1，这说明检查实现正确，并不说明这些故障难检。

### 变异测试

`scripts/mutation_test.py` 把手写变异体逐个打到分析器源码的临时副本上，再跑整套 pytest；测试失败即为杀死。包括每个检查被整体关掉的变异体。明细在 [results/mutation.md](results/mutation.md)。

<!-- BEGIN:mutation -->
变异体 31 个；非等价变异体被测试杀死 30/30；声明为等价 1 个（M28 survived）。
<!-- END:mutation -->

## 运行

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate    Linux/macOS: source .venv/bin/activate
pip install -r requirements.txt

python -m canbus_analyzer run --seed 0 --faults all      # 单次运行 -> results/run_seed0_all/
python -m canbus_analyzer run --seed 3 --faults drop,dlc # 任选故障: drop,jitter,delay,range,dlc,counter,checksum,stuck
python -m canbus_analyzer all                            # 重新生成 results/（两次单跑 + 扫描）
python scripts/mutation_test.py                          # 变异测试 -> results/mutation.*
python scripts/readme_check.py --write                   # 用 results/ 重写 README 表格
python -m pytest -q
```

## 目录

```
canbus_analyzer/   scenario（场景）faults（故障注入）bus（虚拟总线回放）analyzer（检查）
                   logio（ASC/BLF）scoring（与注入真值比对）sweep report __main__
dbc/               opendbc 原文件 + 生成的 toyota_nodsu_pt_generated.dbc
scripts/           build_dbc.py  mutation_test.py  readme_check.py
tests/             pytest
results/           脚本生成的报告、CSV（ASC/BLF 日志不入库）
```

## English summary

Synthetic AEB scenario encoded with a real opendbc Toyota DBC, replayed over python-can's in-process virtual bus with switchable faults (drops, jitter, delays, out-of-range values, wrong DLC, rolling-counter and checksum errors, stuck signal). A cantools-based analyzer reports cycle-time statistics, signal checks and the trigger→brake-request / brake-pressure latency against simulator ground truth. Traffic is exported to ASC and BLF with python-can and read back frame by frame. Classic CAN only; no CAN FD / FlexRay; files not opened in Vector tools. A 20-seed sweep measures event-level recall per fault intensity, spurious findings caused by other faults, and how the latency measurement error depends on bus jitter: the dominant error is the 50 ms radar-frame sampling of the trigger, not jitter. Hand-written mutation testing (scripts/mutation_test.py) checks that the tests fail when any detector is broken or disabled. All README numbers are generated and checked by a test.

## License

MIT, see [LICENSE](LICENSE). Vendored DBC files are from commaai/opendbc under its own MIT licence, see [THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES).
