# GitHub工创赛代码筛选（2026-11-02）

查询工创赛、GongChuang、智能物流、物流搬运STM32、ZDT_X42。先使用GitHub连接器查库及读取目录，连接器随后连续网络错误，改用公开Git只读稀疏获取源码。以下结论均基于实际源码，不以星数、仓库名字或作者比赛成绩自述代替代码验证；没有运行外部代码，没有把外部代码复制进固件。

## 1. 最值得读：Wuyanzu-wuhu/gc-stm32F4-

来源 https://github.com/Wuyanzu-wuhu/gc-stm32F4- ，固定提交a3476a5a309e8676bf9935f51710491cf8c8b057。阅读Hardware/MC.c、PID.c、K230.c及control接口。

和我们同为STM32F407、标准外设库、HWT101、四麦轮、视觉辅助，所以结构最接近。MC.c:309-321实际UART速度报文为地址F6方向速度高速度低一字节加速度同步6B，共8字节，是Emm式而非X九字节串口报文。325-333有广播00 FF66 6B触发；580-588四轮预置同步位1后再触发，确实有同步启动方案，与之前pinksakura的同步位0逐轮立即执行不同。但我们既有A/B未见同步改善侧偏，不据此再次改同步或切回Emm。

可借鉴：MC.c把运动分为加速/巡航/减速/归位/转向；场地速度经角度变换为车体速度，再叠加yaw修正。PID状态按动作切换清零；K230.c:292-323分别以dx、dy产生二维修正，并独立保持航向。真正对横移偏差有意义的是视觉位置误差，不是只提高航向P。转向有连续5次角度达标确认，但应结合真实新鲜样本和停止反馈，不能只靠调用次数。

不宜照搬：MC.c:144-166在DMA发送/关闭处while无超时；ZDT_Speed_Control使用同一个static cmd并在调用下一次发送前就修改，存在DMA完成前写共享发送缓冲的风险（1ms延时不等于完成证明，8字节115200bps约0.694ms且受调度影响）。PID.c的I/D按调用次数累加/作差，没有dt；其1.6/0/1与我们的P/D定义、轮组几何、速度单位和周期不等价。K230所读dx/dy路径未形成我们所需的完整时效保护。借鉴分层和视觉闭环，保留我们有界交易、ACK配对、S曲线和故障保护。

## 2. 2025整车结构参考：Thirtynine3939/2025GongChuang_AGV

来源 https://github.com/Thirtynine3939/2025GongChuang_AGV ，固定提交f1fbab3605f4941fdc14a13d9380aa48700a4b87。下位机源码在程序/下位机/Chassis_v2.0.zip，已只读检查Drive/Source/car.c、motor.c、task.c、pid.c、jetson.c，选定文本保存在agv_archive_selected。

可借鉴：car.c:15-33场地到车体速度变换；task.c:345-387视觉位置偏差分解到两个方向；动作到上位机的完成反馈和分层任务接口。

限制：motor.c是TIM脉冲产生/方向引脚控制，不是X固件CAN。car.c:85-108的Absolute_Pos以last_x/last_y保存上一目标并立刻将last_x/last_y设为新目标，移动由时间和预定速度曲线推进，不是可靠外部定位反馈下的绝对位置闭环；不能因函数名叫Absolute_Pos就认为它能测出轮滑或真实轨迹。PID I/D无dt。我们不退回定时开环距离控制，也不照搬其1.02补偿常数。

## 3. 老方案概念参考：aystmjz/Stepping-Motor-Car

来源 https://github.com/aystmjz/Stepping-Motor-Car ，固定提交c3464f6641df56be37d2b35e98ec723bb7a4fafa，查看src/HARDWARE/PID.c、SMOTOR.c及定时器入口。README称国赛最终版在Finals_Code分支，但本次远程fetch提示该ref不存在；只审查已取得的main，不称读过决赛版，也不验证作者获奖自述。

名字含Stepping不代表底盘采用张大头X。PID.c存在两轮位置/速度误差控制与TIM1 PWM输出；SMOTOR.c为机构STEP/DIR脉冲控制，架构与我们四轮X内部闭环不同。只借鉴误差区间切换与任务/机构分离的概念；不能在X驱动外再不加辨别套一层该速度PID。KI默认为0，本身也不支持我们一定要加I的结论。

## 4. 补充协议资料：yu1745/zdt_x42s_can

来源 https://github.com/yu1745/zdt_x42s_can ，固定提交064711671b6cdfd713fb46af05556496cff87089。README及components/zdt_can/zdt_can.c显示它主要是ESP-IDF TWAI生命周期/接收封装，命令构建交给外部libzdt；示例明确配置EMM固件。X42S是型号，不等于使用X固件。其关于原生CAN数据含地址的描述与本车已验证的封装不一致，不能直接替换我们CAN编解码；帧格式应继续以厂家手册与本车回读/实测为准。

## 对当前工程的建议

保留X。先统一实际编译入口（本轮已完成），验证低速收尾保护（本轮已完成离线模型，尚未实车），再以新鲜、坐标明确且有限幅的X/Y或视觉偏差实现位置修正。继续保留P1/D0.25/死区0.3/I关闭；不是看到别人的Ki或同步位就改当前参数。同步只能解决下发时序，不能单独消除地面横向位置偏差。

此次没有从这些仓库直接引入代码；R FIELD-01/06 RUN-01 PWR-01及E LOC-03/05 MOT-02/04 RUN-06核对不变，比赛规则和Q未变。外部仓库仅工程参考，不是本车验证证据。
