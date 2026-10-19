# can-bus-analyzer

新增 [CANoe AEB 接入与自动化验收](canoe/README.md)：CAPL 状态机/监督器、独立刺激节点、8 组场景和 36 个手写总线检查、ASC/BLF 导出与独立日志核验。共享算法函数已经 C++ / Python 对照；**CAPL 包装尚未在 CANoe 编译或运行**。生成命令为 `python -m canbus_analyzer canoe-build`。

两部分，都跑在 python-can 的进程内虚拟总线上，报文用真实量产车 DBC（commaai/opendbc 的 Toyota 定义）编解码：

1. **AEB 功能设计与功能测试**：写了一个 AEB 功能（OFF / STANDBY / FCW / PARTIAL_BRAKE / FULL_BRAKE / FAULT 六状态，表驱动状态机），带输入监控（超时 + checksum/DLC/范围校验）和降级策略；功能节点、被控对象（玩具车辆模型）和记录节点三方挂在同一条虚拟总线上闭环运行。按 Euro NCAP AEB Car-to-Car 协议 v3.0.2 的 CCRs / CCRm / CCRb 场景类型（参数来源逐条标注）加上无威胁、驾驶员干预、故障、主开关关闭场景组成场景矩阵；功能 KPI 只从总线日志解码得出，再与仿真真值对比；功能测试用例表由运行矩阵自动生成（含实际结果和 PASS/FAIL）。
2. **总线分析**：合成 AEB 场景回放 + 故障注入，cantools 解码做周期统计、信号合法性检查和 AEB 链路时延测量；报文导出成 Vector ASC / BLF 并读回逐帧比对。

## 范围（事实说明）

- 场景是合成的玩具模型（自车巡航、前车急刹、简化的雷达/DSU 决策与一阶制动执行器），不是整车模型，也没有接真实 ECU。
- 第 1 部分的"功能"是本项目自己写的 Python 逻辑，不是任何量产 ECU 的功能；所有阈值（TTC、车速窗口、迟滞、驻留时间、超时次数、减速度）都是本项目的设计选择，不是任何主机厂、供应商或法规的数值。被控对象是一维点质量模型（100 % 重叠、无轮胎/路面模型、雷达无噪声无视场限制）。没有 Euro NCAP 评分，也不声称符合 Euro NCAP。
- 总线是 python-can 的 `virtual` 接口（进程内队列）。它不模拟仲裁、位定时、总线负载或错误帧；所有时序故障都由故障注入器在逻辑时钟上产生。表中的"总线负载"是按经典 CAN 2.0A 帧长公式估算的。
- 只用了经典 CAN（每帧最多 8 字节）。没有实现 CAN FD 和 FlexRay。
- ASC / BLF 文件由 python-can 的 writer 写出、python-can 的 reader 读回；**没有在 CANalyzer / CANoe 里打开过**（没有许可证）。
- opendbc 的 DBC 不含 `GenMsgCycleTime` 属性，所以各报文周期是本项目在 `canbus_analyzer/config.py` 里设定的假设值：WHEEL_SPEEDS 10 ms、SPEED 20 ms、BRAKE_MODULE 25 ms、ACC_CONTROL 30 ms、PRE_COLLISION 30 ms、LEAD_INFO 50 ms。第 1 部分另外用到 GAS_PEDAL 20 ms、PCS_HUD 30 ms（同样是假设值，见 `canbus_analyzer/aeb_function.py`）。

---

# 第 1 部分：AEB 功能设计与功能测试

代码：`canbus_analyzer/aeb_function.py`（状态机 + 输入监控 + ECU 节点）、`closed_loop.py`（三节点闭环）、`function_kpi.py`（从总线日志提 KPI）、`function_matrix.py`（场景矩阵、需求、用例生成）。运行 `python -m canbus_analyzer function`，产物在 [results/function/](results/function/)。

## 功能定义

| 方向 | 报文.信号 | 用法 |
|---|---|---|
| 输入 | LEAD_INFO.LEAD_LONG_DIST / LEAD_REL_SPEED (50 ms) | TTC = 距离 / 接近速度（恒速假设），用最近一帧有效雷达帧，不外推。距离 = 300 m（DBC 上限）视为无目标（本项目约定） |
| 输入 | SPEED.SPEED (20 ms) | 自车车速，用于车速窗口和静止判断 |
| 输入 | GAS_PEDAL.GAS_PEDAL (20 ms) | 加速踏板开度，驾驶员超越 |
| 输入 | BRAKE_MODULE.BRAKE_PRESSED (25 ms) | 驾驶员制动 |
| 输出 | PRE_COLLISION.STATE / PRECOLLISION_ACTIVE / FORCE / _COUNTER (30 ms) | STATE 填功能状态（0 OFF、1 STANDBY、2 FCW、3 PARTIAL_BRAKE、4 FULL_BRAKE、5 FAULT，这是本项目的编码，不是丰田对该字段的定义）；制动状态下 ACTIVE = 1，FORCE = −整车质量 × 请求减速度 |
| 输出 | PCS_HUD.FCW / PCS_INDICATOR (30 ms) | FCW、PARTIAL、FULL 时 FCW = 1；PCS_INDICATOR 按 DBC 的取值表：2 = 故障、1 = 用户关闭、0 = 启用 |

opendbc 的 PCS_HUD 里有互相重叠的占位信号（如 SET_ME_X10 39\|8 覆盖了 PCS_TEMP / PCS_DUST），cantools 对这条报文编码、解码都会失败（按 10 字节打包）。所以这两个信号按 DBC 给出的起始位和长度直接读写字节（`dbcutil.pack_byte_signals`），位置由测试从 DBC 定义核对。

## 状态机

转移表就是代码里的 `TRANSITIONS`，下表由它直接渲染。每个 10 ms 任务从上往下检查，第一条成立的转移生效；同一任务内只走一步。

<!-- BEGIN:fsm -->
| rule | from | to | condition (rows checked top to bottom, first match wins) |
|---|---|---|---|
| R01 | STANDBY, FCW, PARTIAL_BRAKE, FULL_BRAKE, FAULT | OFF | main switch off |
| R02 | OFF | FAULT | main switch on AND input fault (see input_fault) |
| R03 | OFF | STANDBY | main switch on AND all inputs received and valid |
| R04 | STANDBY, FCW, PARTIAL_BRAKE, FULL_BRAKE | FAULT | input timeout (> N cycles) OR E2E error (checksum/DLC/range) on SPEED, LEAD_INFO, GAS_PEDAL or BRAKE_MODULE |
| R05 | FAULT | STANDBY | inputs valid continuously >= fault_recover_s |
| R06 | FCW, PARTIAL_BRAKE, FULL_BRAKE | STANDBY | accelerator pedal >= gas_override_pct (driver override) |
| R07 | FCW | STANDBY | driver brake pedal pressed (driver is reacting) |
| R08 | PARTIAL_BRAKE, FULL_BRAKE | STANDBY | ego speed < v_stop_kph (standstill reached) |
| R09 | PARTIAL_BRAKE, FULL_BRAKE | STANDBY | dwell >= brake_min_dwell_s AND no TTC (no target / not closing); [variant brake_release_ttc_hyst: OR TTC >= ttc_pb_s + ttc_hyst_s] |
| R10 | FCW | STANDBY | dwell >= fcw_min_dwell_s AND (no TTC OR TTC >= ttc_fcw_s + ttc_hyst_s) |
| R11 | STANDBY, FCW, PARTIAL_BRAKE | FULL_BRAKE | armed AND TTC < ttc_fb_s |
| R12 | STANDBY, FCW | PARTIAL_BRAKE | armed AND TTC < ttc_pb_s |
| R13 | STANDBY | FCW | armed AND brake pedal not pressed AND TTC < ttc_fcw_s |

armed = TTC defined AND v_min_kph <= ego speed <= v_max_kph AND accelerator < gas_override_pct.

| parameter (project choice) | value |
|---|---:|
| `ttc_fcw_s` | 2.6 |
| `ttc_pb_s` | 1.6 |
| `ttc_fb_s` | 0.9 |
| `ttc_hyst_s` | 0.3 |
| `v_min_kph` | 8.0 |
| `v_max_kph` | 130.0 |
| `v_stop_kph` | 0.5 |
| `gas_override_pct` | 40.0 |
| `fcw_min_dwell_s` | 0.8 |
| `brake_min_dwell_s` | 0.5 |
| `brake_release_ttc_hyst` | False |
| `fault_recover_s` | 1.0 |
| `partial_decel_mps2` | 4.0 |
| `full_decel_mps2` | 9.0 |
| `vehicle_mass_kg` | 1500.0 |
| `task_s` | 0.01 |
<!-- END:fsm -->

设计要点（都是本项目的选择）：

- **升级可跳级**：STANDBY 在一个任务内可直接进 PARTIAL 或 FULL（如目标突然出现），不强制先经过 FCW。
- **迟滞与驻留**：FCW 退出要求 TTC 回到阈值 + 0.3 s 以上且已驻留 0.8 s；制动状态驻留至少 0.5 s。FULL 不降回 PARTIAL。
- **制动释放**：默认只在"不再接近（TTC 无定义）"或"已静止"时释放。曾考虑的另一种规则是 TTC 回到 1.6 + 0.3 s 以上就释放；但制动中自车减速，恒速 TTC 会自己变大，这条规则会过早松开。两种规则都在矩阵上跑过，对比见下方测试用例小结（`brake_release_ttc_hyst` 参数）。
- **驾驶员**：加速踏板 ≥ 40 % 时禁止激活，并中止 FCW 和制动；制动踏板只取消 FCW（认为驾驶员已在反应），不取消自动制动。

## 总线监控与降级策略

- 每个输入报文：超过 3 个周期没有收到**有效**帧即超时；连续 2 帧无效（DLC 错、Toyota checksum 错、超出 DBC 物理范围）即 E2E 错误。单帧错误只丢弃该帧，不降级。
- 这些输入报文在 opendbc DBC 里都没有滚动计数器，所以对输入**做不了**计数器检查；计数器只出现在功能自己发出的 PRE_COLLISION._COUNTER 上。
- 任一输入超时或 E2E 错误 → FAULT：PCS_INDICATOR = 2，FCW 和自动制动全部抑制，**包括正在进行中的制动**（fail-silent）。这样做的代价在矩阵里量出来了：FLT-02 在部分制动中丢失雷达，制动被中止，仿真真值里以较高速度相撞（数字见 KPI 表）。
- FAULT 锁存：输入连续有效 1.0 s 后才回到 STANDBY。

## 系统需求（本项目定义的通过准则）

<!-- BEGIN:reqs -->
| id | requirement (project-defined pass criterion) |
|---|---|
| REQ-01 | Threat scenario: FCW is on the bus while the bus TTC is still >= 2.0 s. |
| REQ-02 | Threat scenario: a brake request is on the bus before contact, and not before the FCW. |
| REQ-03 | Threat scenario: no contact if the initial closing speed is <= 50 km/h; otherwise the relative impact speed is >= 20 km/h below the initial closing speed. |
| REQ-04 | No-threat scenario: no FCW and no brake request (false activation). |
| REQ-05 | Accelerator >= override threshold: no brake request later than 150 ms after the pedal frame. |
| REQ-06 | Driver brakes during FCW: the FCW is cancelled within 200 ms of BRAKE_PRESSED. |
| REQ-07 | Input timeout / E2E error: FAULT (PCS_INDICATOR = 2) on the bus within 200 ms of the bus-observed fault onset, and no brake request while FAULT is shown. |
| REQ-08 | FAULT heals only after the inputs have been valid for >= 1.0 s. |
| REQ-09 | Main switch off: no FCW, no brake request, PCS_INDICATOR = 1 (off). |
| REQ-10 | The state sequence on the bus contains the expected states in order and none of the forbidden ones. |
<!-- END:reqs -->

## 场景矩阵与参数来源

读过的文档：Euro NCAP *TEST PROTOCOL – AEB Car-to-Car systems*, Version 3.0.2, July 2019（[PDF](https://cdn.euroncap.com/cars/assets/euro_ncap_aeb_c2c_test_protocol_v302_30dae6950b.pdf)；原来的 `cdn.euroncap.com/media/56143/...` 链接已 404，这是 Euro NCAP 网站现在给出的地址）。用 pypdf 抽取文本后逐条读取，下表"来源"一列写明哪些参数是从文中读到的。

| 族 | 本项目参数 | 来源 |
|---|---|---|
| CCRs | 自车 10–80 km/h，5 km/h 一档（15 例）；目标静止 | 协议 §8.2.3：AEB 10–50 km/h、FCW 30–80 km/h，5 km/h 步长。**协议还要求 −50 %…+50 % 重叠率，本项目只做 100 %** |
| CCRm | 自车 30–80 km/h，5 km/h 一档（11 例）；目标 20 km/h | 自车车速范围来自 §8.2.4。目标车速在协议的图 8-2 里，抽取文本中没有，**20 km/h 按项目设定处理，不算"从协议读到"** |
| CCRb | 两车 50 km/h；目标减速度 −2 / −6 m/s²；车距 12 / 40 m；全组合（4 例） | §8.2.5（车速、减速度、车距全部读自正文）。协议允许目标在 1.0 s 内达到目标减速度，本项目用阶跃（项目设定） |
| T0 | CCRs/CCRm 在 TTC = 4 s 处；CCRb 在目标开始减速时；之前有 2 s 稳态行驶 | T0 定义读自 §4.2.1；2 s 预行驶是项目设定 |
| DRV-01 | 驾驶员在 FCW 帧后 1.2 s 开始 4 m/s² 制动 | 1.2 s 反应时间读自 §8.4.6（协议用于 FCW 测试的制动机器人）；4 m/s² 是项目设定 |
| NT / OVR / OFF / FLT | 无威胁（前车加速驶离、切出、稳定跟车）、加速踏板超越、主开关关闭、输入超时 / checksum 错误 | 全部是项目设定 |

## 功能 KPI（每个场景，全部从总线日志解码得出）

列说明：TTC@FCW / TTC@brake 的"bus"是该事件帧之前最近一帧有效 LEAD_INFO 算出的 TTC，"true"是仿真器在该帧时刻的物理 TTC；碰撞与碰撞速度的总线估计 = 最后一帧非零距离的雷达帧按恒速外推到距离为 0 的时刻，取该时刻之前最近的 SPEED；fault→FAULT 的总线值 = 从总线上看到的故障起点（第一帧坏帧，或第一个错过的周期）到第一帧 PCS_INDICATOR = 2，真值 = 从第一帧被注入的帧到 ECU 内部进入 FAULT。

<!-- BEGIN:kpi -->
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
<!-- END:kpi -->

## 功能测试用例（运行矩阵生成）

完整用例表（ID、场景、关联需求、前置条件、激励、期望状态序列、通过准则、实际结果、PASS/FAIL）：[results/function/testcases.md](results/function/testcases.md) / [testcases.csv](results/function/testcases.csv)。期望序列与通过准则在跑之前写在 `function_matrix.expectation()` 里，实际结果和判定由运行结果填入。

<!-- BEGIN:testcases -->
41 test cases generated by running the matrix: 38 PASS, 3 FAIL.

| family | cases | pass | contact | FCW issued | brake requested |
|---|---:|---:|---:|---:|---:|
| CCRs | 15 | 15 | 5 | 15 | 15 |
| CCRm | 11 | 11 | 1 | 11 | 11 |
| CCRb | 4 | 2 | 2 | 4 | 4 |
| NoThreat | 3 | 3 | 0 | 0 | 0 |
| Override | 3 | 2 | 2 | 2 | 2 |
| Off | 1 | 1 | 1 | 0 | 0 |
| Fault | 4 | 4 | 0 | 3 | 3 |

Failed: TC-027 CCRb-12m-2 (REQ-03); TC-028 CCRb-12m-6 (REQ-03); TC-036 DRV-01 (REQ-06).

Brake-release A/B over the 30 CCR cases: contacts with the default rule (release only when no longer closing, or at standstill) 8; with the TTC-hysteresis release variant 12 (extra contacts: CCRs-10, CCRs-15, CCRs-20, CCRs-25).
<!-- END:testcases -->

## 读表（第 1 部分）

- **CCRb 12 m 两例相撞（REQ-03 不通过）。** −6 m/s² 时 TTC 下降太快，FCW 到制动请求只有 267 ms，仿真真值相对碰撞速度 21.0 km/h。−2 m/s² 时总线上的状态序列是 FULL_BRAKE → STANDBY → FCW → PARTIAL_BRAKE → STANDBY：自车减到与目标同速后"不再接近"，按释放规则松开制动，而目标还在继续减速，于是再次接近、重新激活，第二次只到 PARTIAL，最后以 1.3 km/h 相撞。说明"不再接近就释放"对持续减速的目标偏早；可能的改法（释放前看目标加速度、释放后保持一段时间）没有实现。
- **DRV-01 不通过 REQ-06，这是阈值之间的矛盾，不是代码错误。** FCW（TTC 2.6 s）到部分制动（TTC 1.6 s）在恒定接近速度下只隔约 1 s（CCRs / CCRm 各例 FCW→制动请求 957–1021 ms），而 Euro NCAP v3.0.2 §8.4.6 对 FCW 测试假定驾驶员 1.2 s 后才制动。驾驶员踩下制动时功能已经在部分制动里，按设计制动踏板只取消 FCW 状态，所以 FCW 没有被取消（这一例最终没有相撞）。要满足 1.2 s 反应时间，FCW 阈值和部分制动阈值至少要拉开 1.2 s 以上。
- CCRs 10–55 km/h 全部避免碰撞；60–80 km/h 相撞，但相对碰撞速度都比初始速度低 20 km/h 以上（REQ-03 的减速档）。CCRm 只有 80 km/h 一例相撞。
- **制动释放规则 A/B：** 换成"TTC 回到 1.9 s 以上就释放"后，CCR 30 例的碰撞从 8 例增加到 12 例，多出来的是 CCRs 10–25 km/h：部分制动把恒速 TTC 抬过 1.9 s 后制动松开，此时自车已低于 8 km/h 的激活车速下限，功能无法再次介入，自车低速蹭上目标。
- **降级时间：** LEAD_INFO 停发后，总线上 132 ms（FLT-01）/ 115 ms（FLT-02）出现 PCS_INDICATOR = 2；ECU 内部进入 FAULT 是 104 / 101 ms（3 × 50 ms 超时，从第一个错过的周期算起约 100 ms，加 10 ms 任务周期）。总线值多出的部分来自 PCS_HUD 的 30 ms 发送周期。SPEED 连续 checksum 错误 29 ms 降级（2 帧防抖），输入恢复 1.03 s 后解除。
- **fail-silent 的代价：** FLT-02 在部分制动中丢失雷达，制动被中止，仿真真值里以 56.5 km/h 相撞（初速 60 km/h）。这次碰撞在总线上看不到（雷达帧已停），所以总线 KPI 把它记为"不可观测"，不是"无碰撞"；FLT-01 同理。
- **误激活：** 3 个无威胁场景 0 次 FCW / 制动。这只是很弱的证据：雷达是无噪声理想值，没有鬼影、相邻车道目标、弯道或静止路边物体。
- **总线 KPI 与真值的偏差：** 总线 TTC 比同一时刻的物理 TTC 系统性偏大（FCW 0.005–0.066 s，制动请求 0.003–0.076 s），因为功能用最近一帧雷达（最多 50 ms 前）且不外推，告警比阈值名义值略晚。碰撞速度的总线估计误差 −4.42 到 +0.55 km/h，最差的是 CCRs-60（真值 4.4 km/h，总线估计 −0.2 km/h）：重制动下按恒速外推接触时刻会偏晚。
- REQ-10 的状态序列检查是"按顺序包含期望状态、且不出现禁止状态"，中间允许插入其他状态，比逐帧比对宽松。

---

# 第 2 部分：总线分析

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


---

# 测试与变异测试

`python -m pytest -q` 跑全部测试：状态机转移表每一条边（13 条规则、27 条"源状态→目标状态"边）都有触发用例，阈值、迟滞、驻留、FAULT 锁存、超时边界都有临界值用例；KPI 提取既在闭环运行上测，也在手工构造的日志上测（KPI 是否取对帧、坏帧是否被排除）；用例生成器的 PASS/FAIL 判定单独测。

`scripts/mutation_test.py` 把手写变异体逐个打到源码的临时副本上，再跑整套 pytest（README 一致性测试除外）；测试失败即为杀死。M 开头的是第 2 部分（分析器）的变异体，N 开头的是第 1 部分（状态机、输入监控、KPI 提取、用例判定）的变异体，比如阈值比较写错、去掉迟滞、FAULT 不锁存、KPI 取错帧、坏帧没排除。明细在 [results/mutation.md](results/mutation.md)。

<!-- BEGIN:mutation -->
变异体 59 个；非等价变异体被测试杀死 58/58；声明为等价 1 个（M28 survived）。
<!-- END:mutation -->

这个变异集合针对 Python 分析器与 AEB 功能，不覆盖新增 CAPL 源码。CAPL 共享算法的跨语言对照和真实 CANoe 验证状态分别记录在 [接入说明](canoe/README.md) 中。

## 运行

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate    Linux/macOS: source .venv/bin/activate
pip install -r requirements.txt

python -m canbus_analyzer run --seed 0 --faults all      # 单次运行 -> results/run_seed0_all/
python -m canbus_analyzer run --seed 3 --faults drop,dlc # 任选故障: drop,jitter,delay,range,dlc,counter,checksum,stuck
python -m canbus_analyzer function                       # AEB 功能场景矩阵 -> results/function/
python -m canbus_analyzer all                            # 重新生成 results/（两次单跑 + 扫描 + 功能矩阵）
python scripts/mutation_test.py                          # 变异测试 -> results/mutation.*
python scripts/readme_check.py --write                   # 用 results/ 重写 README 表格
python -m pytest -q
```

## 目录

```
canbus_analyzer/   第 1 部分：aeb_function（状态机、输入监控、ECU 节点）closed_loop（三节点闭环）
                             function_kpi（从总线日志提 KPI）function_matrix（场景矩阵、需求、用例）
                   第 2 部分：scenario（场景）faults（故障注入）bus（虚拟总线回放）analyzer（检查）
                             logio（ASC/BLF）scoring（与注入真值比对）sweep report
                   __main__ dbcutil config
dbc/               opendbc 原文件 + 生成的 toyota_nodsu_pt_generated.dbc
scripts/           build_dbc.py  mutation_test.py  readme_check.py
tests/             pytest
results/           脚本生成的报告、CSV（ASC/BLF 日志不入库）
```

## English summary

**Part 1 – AEB function design and functional test.** A project-defined AEB function (states OFF / STANDBY / FCW / PARTIAL_BRAKE / FULL_BRAKE / FAULT) with a table-driven state machine (TTC thresholds, ego-speed window, accelerator/brake-pedal override, hysteresis and minimum dwell; all thresholds are project design choices, not vendor values), input supervision (timeout > 3 cycles, 2 consecutive DLC/checksum/range errors) and a fail-silent degradation strategy (FAULT inhibits warning and automatic braking, latched until the inputs have been valid for 1 s). The function, a toy point-mass plant and a logger are three nodes on one python-can virtual bus in closed loop. The scenario matrix uses the CCRs / CCRm / CCRb scenario types; VUT speed ranges, the CCRb speed / target deceleration / headway combinations and the T0 definition were read from the Euro NCAP AEB Car-to-Car test protocol v3.0.2 (sections cited above), while the CCRm target speed, 100 % overlap, the no-threat / override / fault / main-switch-off scenarios and every pass criterion are project settings. No Euro NCAP score or conformance is claimed. KPIs (TTC at FCW / brake request / full brake, FCW-to-brake delay, contact and impact speed, false activations, fault-to-degradation time) are extracted from the decoded bus log only and compared with simulator truth. A functional test-case catalogue (ID, requirement, preconditions, stimulus, expected state sequence, pass criteria, actual result, PASS/FAIL) is generated by running the matrix: 38 of 41 pass. The failures are reported, not tuned away: both CCRb 12 m cases collide (in the −2 m/s² case the brake is released when ego matches the decelerating target and re-engages too late), and the 1.2 s driver reaction used in Euro NCAP FCW tests is longer than the ~1 s between this design's FCW and partial-brake thresholds. An A/B run shows that releasing the brake on TTC hysteresis would add four low-speed CCRs collisions. Bus-only KPIs cannot see a collision once the radar message is lost. CAN FD is not implemented.

**Part 2 – bus analysis.** Synthetic AEB scenario encoded with a real opendbc Toyota DBC, replayed over python-can's in-process virtual bus with switchable faults (drops, jitter, delays, out-of-range values, wrong DLC, rolling-counter and checksum errors, stuck signal). A cantools-based analyzer reports cycle-time statistics, signal checks and the trigger→brake-request / brake-pressure latency against simulator ground truth. Traffic is exported to ASC and BLF with python-can and read back frame by frame. Classic CAN only; no CAN FD / FlexRay; files not opened in Vector tools. A 20-seed sweep measures event-level recall per fault intensity, spurious findings caused by other faults, and how the latency measurement error depends on bus jitter: the dominant error is the 50 ms radar-frame sampling of the trigger, not jitter. Hand-written mutation testing (scripts/mutation_test.py) checks that the tests fail when any detector is broken or disabled. All README numbers are generated and checked by a test.

## License

MIT, see [LICENSE](LICENSE). Vendored DBC files are from commaai/opendbc under its own MIT licence, see [THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES).
