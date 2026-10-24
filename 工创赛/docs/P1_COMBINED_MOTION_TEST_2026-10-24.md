# 组合动作测试：整组供电（2026-10-24）

当前源码APP_TEST_MODE=3。入口App_CombinedTestStep由既有CAN任务调用，未新增RTOS任务；RC1的ARM/START/KEEP/STOP复用现有串口任务。固件上电默认闲置、VM关闭；主机显式run-once才启动，不能只改Watch绕过私有许可。

## 路线与供电

固定路线：前进8秒 → 停稳 → 左移8秒 → 停稳 → 相对逆时针+90度 → 停稳 → 沿新车头前进8秒 → 停稳关VM。平移基础15RPM、修正±2RPM、反馈限30RPM；转向按剩余误差>25度5RPM、>8度3RPM、其余1RPM，1度到位容差和原角速度/稳定条件保留。HWT只保持航向，不提供精确距离/XY控制。

整组只在开始时上电准备一次。中间段零速、停帧mask15、零速核验成功后继续保持VM和电机使能，至少间隔1秒。仅匹配成功会话可释放到下一段；后续跳到状态/零速复核，不再发送重复失能/使能或重复断电上电。段间仍监督数据/健康；最后结束才请求VM0并确认PE2关闭。PE2是控制状态，不是实际电压测量。任何异常/取消/失联仍停止并断VM，首因锁存不清。

## 操作

模式1为独立无线收发、模式2为单次串口平移、模式0为原手动测试；只有模式3为整组。方向选择APP_TRANSLATION_HOLD_MODE不决定组合私有路线。改变模式须编译下载；不用进入Debug。命令行下载后须检查Verify OK/Application running，并确认新会话实时健康遥测，不能只凭日志发车。

```powershell
D:/python/python.exe 'C:/Users/lenovo/Desktop/工创赛/工创赛/scripts/p1_serial_motion.py' --port COM17 --action monitor --mode 3 --seconds 3
D:/python/python.exe 'C:/Users/lenovo/Desktop/工创赛/工创赛/scripts/p1_serial_motion.py' --port COM17 --action run-once --mode 3 --seconds 120
```

第二条仅在现场准备好且发车前提醒用户后执行。PC13或串口stop可取消整组，包括段间；主机200ms KEEP、主控1秒失联取消；主机600ms无新数据尝试STOP。每次启动只一组，不自动重试。准备约3m×3m平整空间，车放中间并确认原朝向，现场能随时关VM。

## 遥测与时限

TM1在组合模式为55字段：原48字段+sequence_state/index/completed及CAN guard原因/异常轮位、准备状态/后稳定mask。主机兼容48/51/55字段。phase是整组状态，单段完成不会导致主机提前退出；只有completed4、成功停帧15、VM0/PE20才软件完成。转向阶段correction_rpm字段为转向命令轮速，勿当平移PD修正；peak_error字段仍来自平移前端局部数据，完整分析需按sequence_index分段。起点误差90度是转向目标，非平移偏差。日志raw/JSONL/summary默认保存tmp/serial_motion_captures。

最新代码模式3转向会话限30秒，其他模式20秒；模式3引用APP_HEADING_RUN_MS的其他独立兜底也随宏更新。保留35秒每段、120秒整组、停止超时、原电机/源/故障/看门狗保护，到期失败不能冒充成功。

## 实测进度

30秒转向上限版本已于2026-10-24真实下载并完成一轮四段组合，总时长56.304秒，用户确认顺序/方向/段间保电/最终停稳和VM灯熄灭均符合。四段完成误差记录约+0.510、-0.214、+0.922、+0.340度。三个段间均采到VM请求1/PE21，最终VM0/四轮反馈0/停止mask15，无CAN/RTOS/Safety故障或失联。转向最后几度按1RPM细调，真实转向阶段含停稳约24.3秒。

证据同日combined_four_stage_pass.raw/jsonl/summary.json、combined_four_stage_observation.json和combined_turn30s_download.log。主机ARM/START各一次，无自动重试。一个结束边界在途KEEP被拒绝，不代表动作失败；旧DAP缓存仍可能补传，主机保留时序门禁。该轮不代表精确XY/距离、负载/高速鲁棒性或完整故障注入已通过。早期换段与20秒超时失败记录保留于根交接和历史证据。

新成果仅本地，未再次推送。R RUN-01/PWR-01及E LOC-03/RUN-06/MOT-01/02已核，官方Q不变。
