"""复用实际CAN测试框架，验证整车节点6回零读参数/控制白名单与通知。"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import check_emm_migration as runner

runner.FW=runner.ROOT/'3.stm32f407工程/CHASSIS'
runner.OUT=runner.ROOT/'工创赛/evidence/integration/2026-11-03/startup_extension_home/can_checks'
runner.OUT.mkdir(parents=True,exist_ok=True)
unit='#include <stdint.h>\nextern volatile uint32_t g_rtos_fault;\n'+runner.UNIT
unit=unit.replace('uint32_t SystemCoreClock=168000000U;', '''uint32_t SystemCoreClock=168000000U;
volatile uint32_t g_startup_axes_state=2,g_startup_axes_vm_request=1;
volatile uint32_t g_startup_extension_home_active,g_startup_extension_home_notice;
volatile uint32_t g_safety_fault,g_rtos_fault;
''')
unit=unit.replace('if(f==0x43)queue_system(a);', '''if(f==0x43)queue_system(a);
 else if(f==0x22){
  uint8_t bytes[17]={0x22,2,0,0,30,0,0,0x27,0x10,1,0x2c,3,0x20,0,60,0,0x6b};
  uint8_t at=1,packet=0,j;
  while(at<17){CanRxMsg *q=&rx_queue[rx_count++];memset(q,0,sizeof(*q));q->IDE=CAN_Id_Extended;q->RTR=CAN_RTR_Data;
   q->ExtId=((uint32_t)a<<8)|packet;if(bad_sequence && packet==1)q->ExtId++;
   q->Data[0]=0x22;j=1;while(j<8 && at<17)q->Data[j++]=bytes[at++];q->DLC=j;packet++;
  }
 }''')
needle=' reset_fixture();CK(CAN1_SetEmmSpeedRaw(1,0,0,10,&reply)==1 && nonzero_count==0);'
assert needle in unit
unit=unit.replace(needle,needle+r'''
 {
 uint8_t params[17],home[4]={0x9a,2,0,0x6b},abort_home[3]={0x9c,0x48,0x6b};
 CanRxMsg notice={0};
 reset_fixture();r=StartupAxis_ReadHomeParameters(params);
#if APP_MOTOR_OFFLINE_ONLY
 CK(r==9 && tx_count==0);
#else
 CK(r==1 && tx_count==1 && params[0]==0x22 && params[4]==30 && params[16]==0x6b);
 reset_fixture();bad_sequence=1;CK(StartupAxis_ReadHomeParameters(params)==6);
#endif
 reset_fixture();g_startup_axes_vm_request=1;g_startup_axes_state=2;g_safety_fault=g_rtos_fault=0;
 g_startup_extension_home_active=1;g_startup_extension_home_notice=1;
 r=StartupAxis_Exchange(home,4,3,&reply);
#if APP_MOTOR_OFFLINE_ONLY
 CK(r==9 && tx_count==0);
#else
 CK(r==1 && last_tx.ExtId==0x600 && last_tx.DLC==4 && last_tx.Data[1]==2 && !g_startup_extension_home_notice);
 reset_fixture();g_startup_axes_state=3;CK(StartupAxis_Exchange(home,4,3,&reply)==7 && !tx_count);
 reset_fixture();g_startup_axes_state=2;g_safety_fault=1;CK(StartupAxis_Exchange(home,4,3,&reply)==9 && !tx_count);g_safety_fault=0;
#endif
 reset_fixture();g_startup_axes_vm_request=0;g_startup_axes_state=5;
 CK(StartupAxis_Exchange(abort_home,3,3,&reply)==1 && last_tx.Data[0]==0x9c);
 notice.IDE=CAN_Id_Extended;notice.RTR=CAN_RTR_Data;notice.ExtId=0x600;notice.DLC=3;
 notice.Data[0]=0x9a;notice.Data[1]=0x9f;notice.Data[2]=0x6b;
 g_startup_extension_home_notice=0;g_startup_extension_home_active=0;StartupHome_Capture(&notice);CK(!g_startup_extension_home_notice);
 g_startup_extension_home_active=1;notice.ExtId=0x500;StartupHome_Capture(&notice);CK(!g_startup_extension_home_notice);
 notice.ExtId=0x600;StartupHome_Capture(&notice);CK(g_startup_extension_home_notice==1);
 g_startup_extension_home_active=0;
 }
''')
runner.UNIT=unit
if __name__=='__main__':runner.main()
