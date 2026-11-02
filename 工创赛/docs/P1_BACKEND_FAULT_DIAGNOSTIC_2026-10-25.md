# 保电换段原因码3分支诊断（2026-10-25）

本轮补充诊断，未修改运行判定、保护条件或截止值，不称换段故障已修复。没有连接串口、下载、复位或发车。板上状态仅沿用上次已知的A组失败态，本轮没有实读确认。

上一轮故障HF1在第二段VERIFY_ON附近捕获source_valid1、帧龄17ms且HWT错误计数0。原因3同时用于多个保护分支，因此不能从该记录唯一判断UART、电机通信、命令邮箱或位置读取根因。保持VM的换段逻辑不改。

## 后端首故障锁存

新增g_chassis_auto_fault_diagnostic和ChassisAuto_ReadFaultDiagnostic。Auto_CutDetailLocked在原有切故障动作之前，只锁存首次进入的分支、原state、IO结果/电机地址、命令龄期、状态/速度有效mask、最大反馈年龄及tick；后续Abort/STOP不覆盖。初始化或接受新后端会话时只重新准备诊断，不清源/运动故障来接受失败会话。

分支reason：0其它或无后端快照；1航向命令邮箱过期；2状态/速度缺失或过期；3起/终点批次过期；4起/终点读取失败；5运行中位置读取失败；6状态/速度查询失败；7距离起点mask不完整；8距离保护（具体原因仍看TM1 distance_fault_reason）。IO失败分支4/5/6的io_result/address才表示本次查询结果/地址，非IO分支io_result0并非成功，地址2分支表示首个不新鲜轮位；其它分支地址0表示不适用。

feedback_age_ms为状态/速度缓存中最旧tick年龄，不是有效样本年龄保证；mask缺失时可能是空缓存的uptime，须先看status_mask/speed_mask。command_age_ms仅对对应航向会话有意义。0ms或0字段不可自动解释为已测健康。

## HF2及兼容

主前端第一次平移trip仍锁存原29字段，在其后追加9个后端字段，共38个uint32。独立帧升级HF2，完整顺序见P1_SERIAL_MOTION_SCHEMA_2026-10-24.json/backend_fault_diagnostic。CRC规则同HF1。原TM1/PM1/PE2及控制协议不改；主机兼容旧HF1，不给旧帧编造backend_fault_reason。HF2仍故障后约1Hz发送，不参与令牌/健康/KEEP。主机故障后一次STOP，再额外收1.5秒；HF1和HF2均不导致新ARM/START。

## 本地验证及下一步

70项主机测试通过，包含HF2原因/地址/原状态、uint32年龄、非法标志/CRC/协议隔离及故障后收HF2只发送ARM/START/STOP各一次的模拟串口流程。默认、A组三段、B组转向+右移、串口单段、手动、无线、位置诊断关闭七种完整Rebuild均0错0警告；源与隔离构建逐字节一致，GBK/CRLF与差异检查通过。默认Code57268/RO788/RW2004/ZI32460。所有chassis_auto_test.h定义及主工程默认动作表与此前一致。证据同日backend_fault_diag_*_build.log和backend_fault_diag_review.json。

MCU锁存、额外栈/故障临界区开销及HF2实收未测试。先前1067ms源失效及减速cause8仍未闭环；本次也没有确认上一轮cause3的唯一根因。下一次用户确认现场后，可下载backend_fault_diag_route_a_20261025复现原左/前/右三段，只一次START。若失败保存HF2，按具体分支查初始化/查询，再做针对性修复；不放宽100ms命令/姿态、400ms位置、120秒整组、VM/停止/看门狗保护，不自动连续重跑。B组尚未执行，回位仍未验证。

核R RUN-01/PWR-01，E LOC-03/05/07、RUN-06、MOT-02/04，官方Q不变。仅本地完成，未提交推送。
