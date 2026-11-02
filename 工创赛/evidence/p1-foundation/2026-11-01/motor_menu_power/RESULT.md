## 2026-11-01 用户授权恢复菜单供电，VM控制已开

用户明确要求打开电机电源以操作驱动菜单。fetch成功，HEAD与origin/main一致；已有未提交成果保留，未修改根工程或X候选。恢复既有独立motor_menu_power_20261031镜像，SHA256 b69ee8ffc2ad052b3362a567225be20bff6f29b3f7168ffaed3cbdb24be423bf，Flash回读42452字节一致，目标RUNNING。读得g_vm_test_request=1、safety_ready=1、safety_fault/rtos_fault/CAN故障/auto_active均0，PE2输出高。此为控制信号证据，现场VM实际电压/屏幕亮起尚待用户观察。

菜单版禁止CAN驱动使能和非零速度，串口运动/航向运行关闭；Safety_Update及看门狗保留，按键可取消供电，启动后600秒超时关闭。没有执行运动或Cal_MFL校准，未声称轮子机械不动或驱动已切X。当前主控已替换为菜单供电版，不再是此前轮速诊断版；不可按旧记录直接发车。新下载脚本flash_motor_menu_power_20261101.py保留10-31证据。待办：现场四台菜单核对、X校准条件及协议静态验证；X运动候选仍默认禁动。

核对R FIELD-01/02/06、RUN-01、PWR-01及E LOC-02/03/05、MOT-02/04、RUN-06；此次为调试供电，不改变R/Q或比赛运行入口。证据：工创赛/evidence/p1-foundation/2026-11-01/motor_menu_power/download_and_readback.json。仅本地，未提交推送。


### 校准不转只读排查

用户已确认进入Cal_MFL并按Enter，但屏幕正常、轴不转。只读回读Flash与菜单版一致，两次1秒间隔RAM：VM请求1/PE2高，安全/RTOS/CAN故障均0，四台状态轮询valid_mask=15/protect_mask=0；stop/disable/enable/zero/jog请求及结果均0，auto_active=0，自动停止state=0。源路径核对：菜单版拒绝由STM32发出的enable1和非零速度，返回错误前不发送；此门禁不能直接拦截驱动按键内部Cal_MFL。当前默认后台只读0x3A状态，不循环下发停止/失能/零速；未发现代码打断校准的证据。未复位/写RAM/下载/清保护。不能凭此排除校准瞬间供电异常或驱动内部条件，仍需屏幕实际提示/型号信息；不盲目开启力矩或删除保护。

证据calibration_code_audit_readonly.json。以上为采样时状态，不代表已监测整个按键校准过程；采样间启动计数较前次变小，是否发生过用户复位/电源重启未确认。

