# AEB CANoe 接入与自动化验收

本目录把已有 Python AEB 功能移植为 CAPL 源码，并提供独立刺激节点、手写预期和 ASC/BLF 日志核验。当前交付状态是**源码与离线验证完成，CANoe 编译和测量未执行**。不提供假造的 `.cfg`、CANoe 截图或通过报告。

## 已交付的工作

| 文件 | 作用 |
|---|---|
| `aeb_core.cin` | Motorola 位编解码、Toyota checksum、输入监督、六状态及 13 条优先规则、输出编码 |
| `aeb_ecu.can` | CAN1 仿真节点；10 ms 计算、30 ms 输出，调用共享核心 |
| `regression_driver.can` | 独立输入发送器和观察器；定时发送测试报文，检查收到的真实输出并打印结果 |
| `../canbus_analyzer/canoe_suite.py` | 8 组手写场景、36 个预期检查、生成 CAPL 数据、ASC/BLF、清单、日志独立核验 |
| `../scripts/check_capl_core.py` | 把相同 `.cin` 函数交给 C++ 编译，与原 Python ECU 对照；它不是 CAPL 编译器 |
| `../tests/test_canoe_suite.py` | 通过改变日志、丢失输出、改状态、校验和、通道和文件校验值，检验验收器确实会拒绝错误 |

控制器的阈值、报文周期和状态码沿用本项目默认配置，都是项目设计值。使用公开 Toyota DBC 不意味着复现丰田量产 AEB。此次增加的是**开环输入刺激与控制器回归**；原来的 Python 车辆闭环仍在 `closed_loop.py`，本 CAPL 包没有新增车辆动力学节点，也不测避撞率。

## 本地生成和检查

在仓库根目录，使用已安装 `requirements.txt` 的 Python：

```powershell
.\.venv\Scripts\python.exe -m canbus_analyzer canoe-build
.\.venv\Scripts\python.exe -m canbus_analyzer canoe-verify artifacts\canoe_aeb\python_reference.asc --reference --report results\canoe_bridge\reference_asc_check.json
.\.venv\Scripts\python.exe -m canbus_analyzer canoe-verify artifacts\canoe_aeb\python_reference.blf --reference --report results\canoe_bridge\reference_blf_check.json
.\.venv\Scripts\python.exe scripts\check_capl_core.py --out results\canoe_bridge\portable_core_check.json
.\.venv\Scripts\python.exe -m pytest -q
```

共享函数的 C++ 检查需要 g++；Windows 脚本使用本机 WSL 内的 `/usr/bin/g++`，Linux CI 直接使用 g++。只去掉 CAPL 的 `variables {}` 声明包装并映射整数类型，函数源码保持原样。该检查覆盖 827 个对照步骤，其中 91 个有额外手写预期，覆盖 13 条规则和全部 27 条来源状态转移边；核对状态、监督历史、解码值和 PRE/HUD 的每个输出字节。CAPL 事件处理器、CANoe 编译器、总线调度和配置没有通过这项检查得到验证。

生成目录默认是 `artifacts/canoe_aeb/`：

```text
manifest.json                       文件及源代码 SHA-256、明确的未执行状态
contract.json                       输入帧与手写预期
python_reference.asc / .blf          Python 参考日志，不能作为 CANoe 运行记录
canoe/aeb_core.cin
canoe/aeb_ecu.can
canoe/regression_driver.can
canoe/generated/regression_data.cin  由 contract.json 生成的数组
dbc/toyota_nodsu_pt_generated.dbc
```

`canoe-build --out <独立目录>` 可生成另一份便携包。构建不会改写原 41 个车辆闭环场景的结果。

## 场景与需求对应

| 用例 | 重点 | 手写检查数 |
|---|---|---:|
| 1 activation_ladder | 待机→预警→部分制动→完全制动→静止释放 | 5 |
| 2 strict_ttc_thresholds | 2.6 / 1.6 / 0.9 s 等于与略小于阈值 | 6 |
| 3 gas_override | 加速踏板 40% 抑制、39.5% 可重新激活 | 4 |
| 4 checksum_and_recovery | 单帧坏校验和、连续两帧错误、故障锁存与恢复 | 4 |
| 5 radar_timeout_and_recovery | 雷达停发、超过三周期、连续有效 1 s 后恢复 | 4 |
| 6 dlc_and_recovery | 单帧短 DLC、连续两帧短 DLC、恢复 | 4 |
| 7 range_and_unknown_id | 超范围速度、未知 ID 忽略、恢复 | 4 |
| 8 driver_brake_and_switch | 刹车踏板取消预警、主开关关闭优先、重新开启 | 5 |

这 36 个总线检查均选在状态稳定的观察窗口；浮点与严格超时的更细边界由共享函数的手写向量核对。两种测试各有作用，不能把稳定窗口检查说成测得了微秒级实时响应。

## 在有权限的 CANoe 环境运行

1. 使用允许加载自定义 DBC/CAPL 的 CANoe 桌面环境，记录完整版本、edition/options 和使用权限。菜单名称因版本而异，以下步骤未在本机执行。
2. 新建 Classic CAN 的单网络配置，逻辑通道 CAN1，选择适用的仿真模式。教学位速率设为 500 kbit/s；本包无硬件、CAN FD、实车或诊断依赖。
3. 给 CAN1 添加便携包里的 DBC。确认 SPEED=0xB4、LEAD_INFO=0x2E6、GAS_PEDAL=0x2C1、BRAKE_MODULE=0x226、PRE_COLLISION=0x283、PCS_HUD=0x411。
4. 在 **Simulation Setup** 添加两个网络仿真节点：`AebEcu` 指向 `canoe/aeb_ecu.can`；`RegressionDriver` 指向 `canoe/regression_driver.can`。保持相对 include 目录，分别编译并保存完整诊断。当前两文件尚未通过 CANoe 编译，遇到 API/语法问题以安装版本 Help 为准。
5. 关闭 Interactive Generator、Replay 和其它相同 ID 的发送器。只有 Driver 发送输入和测试控制帧，只有 ECU 发送 0x283/0x411。不要播放 `python_reference.asc` 来冒充被测节点响应。
6. 开启 Trace、Graphics、Write 和 CAN1 logging block，记录**单份完整 CAN1** ASC 或 BLF；不要把各节点 Tx/Rx 导出合并造成重复。Graphics 选择 SPEED、距离、相对速度、STATE、FORCE、FCW 和 PCS_INDICATOR。
7. 开始测量，运行至少 24 秒。Driver 自动执行 8 例并在 Write 中打印 36 条 `AEB_CHECK`。完整结束必须出现 `AEB_SUITE pass=36 fail=0 expected=36 complete=1`。提前停止没有完整结束记录；仅 Write 文本不能代替总线证据。
8. 保存 CANoe 自己生成的 `.cfg`、原始日志、编译诊断、Write 导出和 Trace/Graphics 截图。重新打开保存的 `.cfg`，检查相对路径仍能解析。把真实日志复制到独立位置，运行：

```powershell
.\.venv\Scripts\python.exe -m canbus_analyzer canoe-verify C:\path\actual_canoe.asc --report results\canoe_bridge\actual_capture_check.json
```

日志核验检查包文件 SHA、各 ID 内的刺激载荷/顺序、每条刺激相对时刻 ±5 ms、Classic CAN1、8 个用例重置标记、两路输出分别从前 35 ms 覆盖到结束前 35 ms、30±5 ms 输出周期、PRE 输出计数器连续性，以及每个检查前 65 ms 内两路输出的全部样本。用例重置处允许计数器复位；无关 ID 的迟到报文不能代替任何一路输出的结束证据。允许同刻不同 ID 因仲裁重排，不允许同 ID 载荷、数量或时间变化。输出值、校验和或 DLC 错误、缺一类输出、日志截断都会失败。真实 CANoe 调度若超出时间容差，先保留失败日志定位原因，不得直接改大容差掩盖问题。

核验成功只表示这份日志符合契约；日志文件不能独立证明生产者身份，因此 JSON 永远不自动设置 `canoe_execution_confirmed=true`。要认定真实 CANoe 完成，还须同时核对版本、编译诊断、实际配置和测量证据。

## 时间与测试控制协议

- CAN1 使用 Classic CAN 标准 11-bit ID。SPEED/GAS 的刺激周期 20 ms，LEAD 50 ms，BRAKE 名义 25 ms。输入从每例 +15 ms 开始；BRAKE 若碰到 10 ms 计算点则延后 1 ms，实际间隔 24/26 ms。监督器的 BRAKE 周期仍为 25 ms、严格超时阈值 75 ms。
- 每例占用 3000 ms 槽位，前 2300 ms 有刺激。初始控制帧在 +5 ms，检查在稳定窗口。ECU 每 10 ms 计算，PRE/HUD 每 30 ms 输出；同刻的内部回调次序不被假定，真实接收与发送时刻以日志为准。
- 测试专用 0x7E0，DLC8，前四字节为 `41 45 42 01`，byte4=用例编号，byte5=主开关0/1，byte6=0重置或1仅切换开关，byte7=0。它是本测试台自己的协议，不来自 Toyota DBC。
- 所有控制/故障帧仅用于隔离仿真。DLC/应用校验和错误不代表物理 CAN CRC、ACK 或错误帧注入。
- `timeNow()` 为 32-bit 10-us 时间，本教学包限定短测量，建议每次不超过 10 分钟，不处理回绕或长期运行。
- 核心只支持默认参数；没有移植原 Python 的替代制动释放策略。输入报文无可用滚动计数器，只检查长度、已有 checksum 和物理范围。

## 官方环境与学习入口

Vector 是公司，CANoe 是其产品。当前官方在线演示用于运行内置样例；FAQ 明确不能上传自定义文件/数据库，结束会话丢弃更改，因此不能用它编译和验收本包。公开安装包不等同使用授权。可使用学校有权限的实验室，或联系 Vector 了解自定义数据库评估环境。

- [CANoe 官方产品页](https://www.vector.com/en/product/canoe/)
- [在线演示 FAQ](https://portal.vector.com/de/web/help/canoe-demo)
- [Getting Started 免费学习目录](https://elearning.vector.com/tag/index.php?lang=en&tag=Getting+Started&tc=6)
- [CAPL WarmUp](https://elearning.vector.com/course/view.php?id=158&section=1)
- [Vector Test Feature Set 教程](https://support.vector.com/sys_attachment.do?sys_id=b1d992aa1bcca094d2cd97de1d4bcb4b)
- [CAPL 定时器/节点官方例](https://support.vector.com/sys_attachment.do?sys_id=27ee721387020a18a0460fe40cbb35d3)

官方资料查阅日期：2026-10-15。这里的步骤是待执行说明；“有源码”“共享函数通过 C++ 核对”“真实 CANoe 编译”“真实测量”“原始日志通过验收”是分别记录的状态。

## 面试讲法与当前边界

可说明“在 Python AEB 仿真的基础上，准备了 CAPL 状态机和输入监督实现，设计了独立刺激、36 个手写总线检查及日志验收；共享核心经过跨语言对照”。本人具体贡献按亲自完成的设计、修改和验证表述。

当前不能说“熟练使用 CANoe”“完成 CANoe/HIL 验证”“实车避撞通过”或把 Python 参考日志当成 CANoe 日志。取得真实运行证据后再补充具体版本、操作和发现。
