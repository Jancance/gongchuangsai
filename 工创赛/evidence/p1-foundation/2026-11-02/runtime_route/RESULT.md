# 运行时分段路线：使用及验证记录（2026-11-02）

已在当前正式工程整合X协议及运行时路线队列。首版支持1～8段，每段按当时车头方向解释；前进/左右横移10～210cm，速度1.3～20cm/s，转向限±90°、30°/s。暂不支持倒车、任意角度或行进中插队。正式工程默认为VM关闭、不自动发车；离线工程保持硬件运动封锁。

## 后续绕地图怎么测

路线文件：`工创赛/config/p1_runtime_perimeter.json`。保持之前8段动作：前进181.44cm、右移180cm、右转90°两次、前进181.44cm、左转90°两次、左移180cm。两段前进包含+0.8%试验补偿；这只是路线距离，不是轮径或里程计标定。

后续首次使用需要下载新的正式工程镜像；本次没有下载、没有发车。下载后核对TMQ/BTQ/HF3身份及健康状态。以后只改变此文件内合法的路线不必重新编译。例子以下均在仓库根目录运行，命令只是说明，本轮未执行：

```powershell
# 只监听
D:/python/python.exe 工创赛/scripts/p1_runtime_route.py --action monitor --port COM17 --seconds 10 --output 工创赛/tmp/route_monitor
# 仅装入路线，不发车
D:/python/python.exe 工创赛/scripts/p1_runtime_route.py --action upload --port COM17 --route 工创赛/config/p1_runtime_perimeter.json --output 工创赛/tmp/route_upload
# 现场重新确认车轮紧固、同起点/载物、完整路径及转向和停车余量清空、可随时断电后，单次上传并运行
D:/python/python.exe 工创赛/scripts/p1_runtime_route.py --action run-once --port COM17 --route 工创赛/config/p1_runtime_perimeter.json --output 工创赛/tmp/route_once
# 停止
D:/python/python.exe 工创赛/scripts/p1_runtime_route.py --action stop --port COM17 --seconds 10 --output 工创赛/tmp/route_stop
```

客户端默认只监听；upload不会ARM或GO。run-once上传成功后才准备并单次GO，没有自动重跑。每次新路线须使用新ID，默认按当前时间生成；同一秒重复命令可稍等或明确递增--route-id。执行结束即消耗该ID，重复GO不能重跑；正常停止且四轮零速/VM关闭才能装下一条路线。故障或取消不能用上传清除，须保留首故障、排查后重新建立健康会话。

## 协议与停止边界

TMQ健康遥测与RC1 ARM/KEEP/STOP沿用原安全链。RQ1依次BEGIN/ROW/COMMIT，CRC16-CCITT及完整FNV1a摘要通过才安装；GO绑定路线ID、摘要及ARM令牌。RA1/RS1仅报告，不续命。断线KEEP超时、姿态过期、反转/超速、VM、故障及看门狗保护保留。旧TMX/TM9工具会拒绝新身份，须使用新客户端。

平移报文单位0.1mm、速度mm/s；转角mdeg、速度mdeg/s。上传窗口10秒，重复包不延长；每段超时1～75秒且不得低于规划所需时间，总预算仍不超过300秒。最多8段；更长比赛路径后续另做有序批次，当前不在运动中追加。RS1记录路线及段进度；XD1只保留最后一个后端会话，不能把它当作整圈四轮连续记录。

运行时里程计以本路线起点(0,0,0)建立局部坐标，仅用于轮速里程计/航向诊断，不代替雷达定位或真实横向闭环。本协议当前用于调试，不等同于正式雷达通信接口。

## 完成和证据

- 正式与离线Keil工程均0错误、0警告；日志build_main_final.log、build_offline_final.log，镜像哈希见release_review.json。
- 实际C解析、命令准入、调度器及后端成功重装：420项通过，含电脑端报文逐字节输入、1～8段、重复报文、第二路线、超时/失联/故障拒绝；c_checks_release/report.json。平台与动作完成为桩。
- 25项电脑端测试通过，覆盖协议身份、上传重试和故障/结果处理。
- 40个运动模型、1,122,413项断言通过：实际后端、航向会话、里程计与当前适配器；包含1～8段、负载/延迟和异常保护。序列驱动、CAN/VM/RTOS和物理车辆是模型。models_release/report.json记录断言数与源码哈希，不构成实车验收。
- SAFETY/WATCHDOG源文件相对任务前快照逐字节相同；未关闭保护，未清硬件故障。队列重装仅放行成功结束且四轮零速、VM关、无故障的会话；保留诊断历史。

保持X驱动、P=1/D=0.25/死区0.3°/I关闭。低速反号诊断沿用主工程整合方案：仅末端10mm内、目标≤2RPM且反向≤0.5RPM时允许单次新样本，连续反号或超时仍停止；START与RUN均检查。此调整只有离线证据。

最新已下载记录仍为旧隔离X镜像6c8284e52e54bc6bda859a827a7b667ba491574fc889338edbae979f70353098（TM9/BT9），上次整圈第二段反号中止，现场最终停稳/灯灭确认未补齐。本次不能据此宣称已升级或实车跑通。规则核对R FIELD-01/02/06、RUN-01、PWR-01；E LOC-03/05、MOT-02/04、RUN-06，未改变规则事实。

仅本地完成，无提交或推送。下一步在现场条件重新确认后下载、先静态核对，再单次绕图测量闭环误差。
