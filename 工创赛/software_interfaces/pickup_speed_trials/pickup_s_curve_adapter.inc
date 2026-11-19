/* Included in VSP after its query/position helpers. Same CAN owner, no new
 * timer/task. Optional FB direct-position references; default remains FD.
 * Trajectory reference is S-shaped; real following/latency must be measured. */
#include "pickup_s_curve.h"
static PickupSCurve vsp_scurve;
static uint32_t vsp_scurve_active,vsp_scurve_begin,vsp_scurve_last,vsp_scurve_issued;
static uint8_t vsp_scurve_frame[11];
volatile uint32_t g_pick_scurve_frames,g_pick_scurve_last_gap_ms,g_pick_scurve_max_gap_ms;
volatile float g_pick_scurve_reference_deg;
static uint32_t VSP_SCurveFrame(uint32_t now)
{
 float x,v,deg,scale;uint32_t raw,speed;VSP_Axis *a=&vsp_axis[1];
 if(!vsp_scurve_active)return 0U;
 PickupSCurve_Eval(&vsp_scurve,(float)(now-vsp_scurve_begin)*.001f,&x,&v);(void)v;
 deg=a->before+(a->goal>=a->before?1.0f:-1.0f)*x*360.0f;
 scale=(a->options&0x80U)?100.0f:10.0f;
 if(!(fabsf(deg)<=(2147483000.0f/scale)))return 0U;
 raw=(uint32_t)(fabsf(deg)*scale+.5f);speed=(uint32_t)(vsp_scurve.rpm*10.0f+.5f);
 /* Position reference follows the jerk-limited path. A fixed speed CAP avoids
  * creeping to the endpoint at1RPM if the real drive has tracking lag. This
  * is NOT an assertion that measured motor velocity/jerk follows the model. */
 memset(vsp_scurve_frame,0,sizeof(vsp_scurve_frame));
 vsp_scurve_frame[0]=0xfbU;vsp_scurve_frame[1]=deg<0.0f;
 vsp_scurve_frame[2]=(uint8_t)(speed>>8);vsp_scurve_frame[3]=(uint8_t)speed;
 vsp_scurve_frame[4]=(uint8_t)(raw>>24);vsp_scurve_frame[5]=(uint8_t)(raw>>16);
 vsp_scurve_frame[6]=(uint8_t)(raw>>8);vsp_scurve_frame[7]=(uint8_t)raw;
 vsp_scurve_frame[8]=1U;vsp_scurve_frame[10]=0x6bU;
 g_pick_scurve_reference_deg=deg;return 1U;
}
static uint32_t VSP_SCurveSend(uint32_t now)
{
 uint32_t r,gap=now-vsp_scurve_last;
 if(!VSP_SCurveFrame(now))return 0U;
 vsp_scurve_last=now;g_pick_scurve_last_gap_ms=gap;
 if(vsp_scurve_issued && gap>g_pick_scurve_max_gap_ms)g_pick_scurve_max_gap_ms=gap;
 r=VehicleArm_Exchange(5U,vsp_scurve_frame,11U,3U,&s_rx);
 if(r!=1U || (s_rx.Data[1]!=2U && s_rx.Data[1]!=0x9fU))return 0U;
 vsp_scurve_issued=1U;g_pick_scurve_frames++;return 1U;
}
static uint32_t VSP_SCurveStart(uint32_t now)
{
 VSP_Axis *a=&vsp_axis[1];
 if(!PickupSCurve_Begin(&vsp_scurve,fabsf(a->goal-a->before)/360.0f,
  vsp_group.command.lift_rpm,45055.0f,.020f))return 0U;
 vsp_scurve_active=1U;vsp_scurve_begin=vsp_scurve_last=now;vsp_scurve_issued=0U;
 return VSP_SCurveSend(now);
}
static uint32_t VSP_SCurveService(void)
{
 uint32_t now=(uint32_t)xTaskGetTickCount();
 if(!vsp_scurve_active || vsp_group.state==4U || (vsp_group.done&SPF_LIFT))return 1U;
 if((uint32_t)(now-vsp_scurve_last)<20U)return 1U;
 return VSP_SCurveSend(now);
}
