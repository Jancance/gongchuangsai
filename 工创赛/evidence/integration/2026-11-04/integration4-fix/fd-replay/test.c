
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#define VEHICLE_SOURCE_PREP_EXTENSION_RPM 30
#define VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S 60
static float sp_goal,s_current,sp_origin,sp_before;static uint32_t sp_samples,sp_target_pending,s_options;
static uint32_t g_source_prepare_command_units,g_source_prepare_command_options;static int32_t g_source_prepare_command_before_mdeg,g_source_prepare_command_goal_mdeg;
static struct{uint8_t Data[8];}s_rx;
static uint8_t sent[15];static int32_t SourcePrep_Mdeg(float v){return v*1000;}
static uint32_t SourcePrep_Exchange(const uint8_t*q,uint8_t n,uint8_t rn){for(unsigned i=0;i<n;i++)sent[i]=q[i];s_rx.Data[1]=2;return 1;}
static uint32_t SourcePrep_Move(void)
{
 uint32_t units,speed;float delta=sp_goal-s_current;uint8_t move[15]={0};
 if(!(fabsf(delta)<=180.0f && fabsf(sp_goal-sp_origin)<=252.1f))return 0;
 /* Ruler + independent 0x31 on 2026-11-05 proves node6 Scale-enabled
  * command AND 0x33/0x36 feedback use 100 raw units per degree. */
 units=(uint32_t)(fabsf(delta)*((s_options&0x80U)?100.0f:10.0f)+.5f);
 speed=(uint32_t)(VEHICLE_SOURCE_PREP_EXTENSION_RPM*10.0f+.5f);
 move[0]=0xfd;move[1]=delta<0;move[2]=move[4]=VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S>>8;
 move[3]=move[5]=VEHICLE_SOURCE_PREP_EXTENSION_ACCEL_RPM_S&255;
 move[6]=speed>>8;move[7]=speed&255;move[8]=units>>24;move[9]=units>>16;
 move[10]=units>>8;move[11]=units;move[12]=2;move[14]=0x6b;
 sp_before=s_current;sp_samples=0;sp_target_pending=1U;
 g_source_prepare_command_units=units;g_source_prepare_command_options=s_options;
 g_source_prepare_command_before_mdeg=SourcePrep_Mdeg(s_current);g_source_prepare_command_goal_mdeg=SourcePrep_Mdeg(sp_goal);
 return SourcePrep_Exchange(move,15,3)==1 && (s_rx.Data[1]==2 || s_rx.Data[1]==0x9f);
}

int main(void){unsigned fail=0,checks=0;
 sp_origin=-.7f;s_current=-1.6f;sp_goal=-126.7f;s_options=0x84;
 unsigned ok=SourcePrep_Move();uint32_t raw=((uint32_t)sent[8]<<24)|((uint32_t)sent[9]<<16)|((uint32_t)sent[10]<<8)|sent[11];
 checks++;if(!ok || raw!=12510 || sent[12]!=2 || sent[1]!=1)fail++;
 float target=s_current-(float)raw*.01f;checks++;if(fabsf(target-sp_goal)>.11f)fail++;
 /* Independent 0x31 and ruler prove feedback /100, not the old /10. */
 checks++;if(fabsf(1249*.01f-12.491455f)>.01f)fail++;
 checks++;if(fabsf((-.16f-12510*.01f)-(-125.26f))>.01f)fail++;
 printf("FD replay checks=%u failures=%u command=%u predicted_target=%.3f\n",checks,fail,raw,target);return fail?1:0;
}
