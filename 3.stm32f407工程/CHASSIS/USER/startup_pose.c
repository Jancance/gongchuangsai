#include "startup_pose.h"
#include "app_config.h"
#include "../../USER/startup_pose_config.h"
#include "gripper.h"
#include "carrier_pwm.h"
#include "carrier_config.h"
volatile uint32_t g_startup_gripper_result=0U,g_startup_carrier_result=0U;
void StartupPose_Init(void)
{
#if !APP_MOTOR_OFFLINE_ONLY
    float angle=STARTUP_CARRIER_ANGLE_DEG;uint32_t pulse;
    Gripper_Init();g_startup_gripper_result=jiazhua(STARTUP_GRIPPER_ANGLE_DEG);
    /* 未配置时不强制转0度；初始化引脚为低，不发舵机位置脉冲。 */
    if(CarrierPwm_Init()!=1U){g_startup_carrier_result=2U;return;}
    if(angle==-1.0f)return;
    if(!(angle>=0.0f && angle<=CARRIER_ANGLE_MAX_DEG)){g_startup_carrier_result=3U;return;}
    pulse=CARRIER_PULSE_MIN_US+(uint32_t)(angle*(CARRIER_PULSE_MAX_US-CARRIER_PULSE_MIN_US)/CARRIER_ANGLE_MAX_DEG+0.5f);
    g_startup_carrier_result=CarrierPwm_Write(pulse)?1U:4U;
#endif
}
