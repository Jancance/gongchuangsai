# 本轮新增成果与源码索引

以下是本聊天形成、需要随正式工程合并或随证据保留的成果。历史副本保留自己的版本路径，不应覆盖正式工程。这里只列源码与产物的逻辑归属，不宣称已检查远端 Git 跟踪状态。

| 成果 | 当前证据路径 | 正式工程交接范围 |
| --- | --- | --- |
| 三任务模块 | `stage3_modules_validation.m` 及同名 JSON/日志 | Source 消费上层锁定颜色；粗加工、堆叠接收同一运行描述；模块完成与停止确认；新入口、相关 owner/视觉桥、上层派发及工程构建配置由主交付合并。 |
| 第三件放置后的直接交接 | `stage3_directhandoff_20261120/project/CHASSIS/USER/{stage3_cycle.inc,integration10_complete.inc,source_parallel_flow.c,source_parallel_flow.h,vehicle_source_parallel.inc}` | Z→DM→实际 X 目标、完成与 STOP 所有权；检查最终模块候选中相同逻辑仍存在。 |
| 最后一件回槽归零与槽口松爪间隔 | `stage3_anticipatory_finalzero_20261120/project/CHASSIS/USER/{stage3_cycle.inc,integration10_complete.inc}`；`stage3_finalzero_validation.m` | 100 ms 从 PWM 成功后计时；末件 DM 0° 实际完成后发布终态。 |
| 早期夹爪预闭合、带料提早启动、时序和原点诊断 | 根目录 `stage3_anticipatory_cycle_validation.m`、`stage3_origin_recheck_fix_validation.m`、`stage3_carryrise1000_validation.m`、`stage3_carrysettle_validation.m` 及其结果 | 属于历史回归。按最新正式候选实际保留的动作与参数判定，不能按旧文件名恢复动作。 |
| 单机启动和串行折返对照 | `stage3_standalone_20261120/project`、`stage3_serialhandoff_20261120/project` 下选定文件，及同名模型/夹具 | 历史版本基线；PC13 的正式分工以整车模块版为准。 |
| Pi 视觉补偿及抓放参数 | `stage3_modules_20261120/{ring_row_alignment.py,stage3-vision-13-5.json,placement-profile.json,recovery-profile-candidate.json,recover-placed-template.json}` | 13/5 补偿和对应坐标锁定；源码哈希用于与部署包配对。 |
| 验证夹具及脚本 | `stage3_*_fixture.c`、`verify_stage3_{standalone,serialhandoff,directhandoff}.py`、`verify_directhandoff_compensation.py` | 主机端验证源及历史输入；与相应模块副本配对。 |

在证据包目录中，可按顺序运行 MATLAB 的 `stage3_finalzero_validation.m`、`stage3_handoff40_validation.m`、`stage3_standalone_validation.m`、`stage3_serialhandoff_validation.m`、`stage3_directhandoff_validation.m`、`stage3_modules_validation.m`。复跑会重写对应输出 JSON；如要保留清单哈希，使用独立副本。C 验证可用 `python verify_stage3_directhandoff.py`、`python verify_stage3_serialhandoff.py` 和 `python verify_stage3_standalone.py`，需要主机 `gcc`。这些夹具会生成临时构建产物。
