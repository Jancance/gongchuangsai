# 2026-11-20 第二、三阶段开发与现场证据

本包记录本轮候选的演进、离线验证和现场读回。`inventory.json` 逐项列出包内该文件的相对路径、会话 `work/` 下的原始路径、字节数、SHA-256 和证据类别。历史候选的配置与日志按版本保留；判断当前行为时，以对应版本的配置和记录配对为准。

| 证据 | 可以确认的内容 | 限制 |
| --- | --- | --- |
| `stage3_modules_validation.*` | Source、Rough、Stack 的 Begin/Poll/Stop 合约前置模型，72 场景。涵盖运行归属、停止确认、映射、堆叠单次扣 24000 虚拟脉冲与终态顺序。 | MATLAB 模型不证明实际固件接口或实体运动。模块候选正式源码与构建凭据随主交付另行发布。 |
| `stage3_directhandoff_validation.*`、`stage3_directhandoff_20261120/directhandoff-final-review.json` | 已锁定末件深度上抬 40 mm；依次等真实 Z、DM、X 到位；X 直接奔首回收环绝对值；视觉补偿独立核算。含 234 个模型场景及该候选 C 验证和双目标构建证据。 | DirectHandoff 记录明确未声明实车验收。 |
| `stage3_serialhandoff_validation.*`、`stage3_serialhandoff_20261120/serialhandoff-final-review.json`、`serialhandoff-postflash-snapshot.json` | 更早的 Z→X0→DM 串行候选，198 个模型场景及历史板上读回。 | 快照中的 `request_completed` 只对应其请求，不等于完整三放三收；此交接已被 DirectHandoff 替代。 |
| `stage3_standalone_validation.*`、`standalone-postflash-full.json`、`stage3_standalone_20261120/pi/pi-startup-readonly.json` | 16 个按键和上电状态模型场景；另保留当时 MCU 烧录后读回与 Pi 自启状态。 | 读回不是重复整轮的实体动作验收。PC13 在新的模块架构中归整车；旧区域单机入口仅作历史证据。 |
| `stage3-anticipatory-finalzero-watch-20261120-025811-347599.json` | 该历史镜像一次现场记录 `completed: true`，保留原始状态与计时。 | 文件仍标记 `physical_acceptance: false`；不能替代后续候选验收。 |
| `stage3-anticipatory-handoff40-watch-20261120-031627-245027.json` | 历史采集保留命令、快照和首个 `USBTimeoutError(10060)`。 | `completed: false`，不能从超时推断完成或净空。 |

当前继承的视觉对齐配置：平移 2 mm、航向 1°；Pi 前 13 mm、右 5 mm。`stage3_modules_20261120/stage3-vision-13-5.json` 中的目标像素为 `(992.9757080078125, 553.7538970947265)`，不再运行对齐后重复补偿。视觉配置 SHA-256 为 `1214c865c0deb2d47accf36297b9486781a29e19279d77474b9be8e21c6fc501`，配套 `ring_row_alignment.py` SHA-256 为 `38e0e323bb182928612e0cdbd370956b4c64ae06271218f80534482179214569`。此处哈希标识本包文件，不声明 Pi 当前部署状态。

主要新增成果及复核入口见 [SOURCE_INDEX.md](SOURCE_INDEX.md)。历史 `stage3_*_202611xx/project` 只是经过选取的测试依赖，不包含完整 Keil 工程。该包不包含凭据、远端部署或连接脚本、Flash 镜像及工具依赖。
