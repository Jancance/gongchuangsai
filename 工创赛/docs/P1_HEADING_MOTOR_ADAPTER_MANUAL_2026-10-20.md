# 航向执行层第一步：用户手动加入四轮指令转换

本步承接已完成的真实航向静态预览。实际固件由用户修改，助手只准备说明和隔离候选。不修改main、现有CAN任务或Keil文件，不开启VM，不下载。

## 1. 文件与声明位置

退出Debug。打开当前Keil工程中的USER/chassis_kinematics.h，在最后一个`#endif`前添加：

```c
/* 航向控制轮速转四轮指令：正值逆时针、负值顺时针，不发送CAN。 */
uint32_t Chassis_HeadingCommands(int32_t command_rpm,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT]);
```

此处command_rpm表示纯旋转时每个轮子的目标RPM幅值及旋转方向，不是车体角速度单位。

## 2. 实现位置与代码

打开USER/chassis_kinematics.c，在文件末尾、现有Chassis_BasicTestCommands函数结束的大括号后追加：

```c
/* 纯指令转换：正值逆时针、负值顺时针，不发送CAN。 */
uint32_t Chassis_HeadingCommands(int32_t command_rpm,
    ChassisMotorCommand commands[CHASSIS_WHEEL_COUNT])
{
    ChassisWheelSpeeds logical = {{0.0f, 0.0f, 0.0f, 0.0f}};
    if (commands == 0) return 0U;
    /* 先检查有符号范围再取反，避免最小负整数溢出。 */
    if ((command_rpm < -5) || (command_rpm > 5))
    {
        (void)Chassis_EncodeEmm(&logical, 5U, commands);
        return 0U;
    }
    /* 轮序：左后、右后、左前、右前；逆时针时左轮后退。 */
    logical.rpm[CHASSIS_LEFT_REAR] = -(float)command_rpm;
    logical.rpm[CHASSIS_RIGHT_REAR] = (float)command_rpm;
    logical.rpm[CHASSIS_LEFT_FRONT] = -(float)command_rpm;
    logical.rpm[CHASSIS_RIGHT_FRONT] = (float)command_rpm;
    return Chassis_EncodeEmm(&logical, 5U, commands);
}
```

保留已有函数和源文件编码。tmp下候选及检查文件不用加入Keil。

## 3. 含义与预期

先构造逻辑轮速，再复用现有Chassis_EncodeEmm和已确认的正向0/1/0/1表，避免混淆车体转向与电机原始方向。地址顺序保持1左后、2右后、3左前、4右前。

| 输入RPM | 四轮逻辑RPM | 四轮驱动方向 | 四轮speed_raw |
|---|---|---|---|
| +5 | -5,+5,-5,+5 | 1,1,1,1 | 5,5,5,5 |
| +3/+1 | 同上符号 | 1,1,1,1 | 对应3或1 |
| -5 | +5,-5,+5,-5 | 0,0,0,0 | 5,5,5,5 |
| -3/-1 | 同上符号 | 0,0,0,0 | 对应3或1 |
| 0 | 0,0,0,0 | 0,1,0,1 | 0,0,0,0 |
| 超出-5至+5 | 返回0 | 默认方向 | 全部清零 |

0速时方向不代表实际转动。转换成功不代表有运动许可；返回0时未来调用方必须走故障/停止流程，不能忽略返回值。5RPM为本阶段工程候选上限，不是官方限制。以后修改控制器速度参数时必须同步审阅该上限，不能静默截断。

## 4. 本轮唯一验收动作

保存这两个文件，完整Rebuild，把日志发来。保持APP_HWT_STATIC_TEST=1、APP_HWT_TURN_TEST=0，不在main中调用新函数、不把数组直接发送到CAN、不设置VM请求。本轮无需下载，因为此函数尚未接入运行路径。

后续再设计动态指令提交/四轮更新、旧限时模式互斥、正常停车反馈确认、反向切换、超时/取消/传感器失效及VM兜底。现有固定10秒序列不能仅通过把5改成变量就作为目标角闭环。

## 隔离验证与范围

候选同段代码见tmp/heading_motor_adapter_candidate.c。使用当前chassis_kinematics.c和独立检查程序以GCC -std=c99 -Wall -Wextra -Werror构建并运行成功：全部11个整数输入-5至+5；以编码后原始方向/速度还原逻辑轮速，再用正解核对无平移且旋转符号正确；-6/+6/INT_MIN/INT_MAX拒绝且清除旧目标；空输出指针拒绝。ARMCC V5候选独立编译退出0，无诊断。这不是用户修改后的完整Rebuild，也不证明CAN帧、同步更新或实机运动通过。

R检查：RUN-01/PWR-01；E检查：LOC-03/05、RUN-06、MOT-01/02/04。官方Q仍待技术答疑确认，未修改规则。真实转角闭环、停车、运行期失效保护和实时X/Y均未闭环。
