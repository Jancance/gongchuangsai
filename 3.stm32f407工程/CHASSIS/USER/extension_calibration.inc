volatile uint32_t g_extension_calibration_only=VEHICLE_EXTENSION_CALIBRATION_ONLY;
/* Raw-unit ruler measurement. No millimetre conversion or automatic repeat. */
volatile uint32_t g_extension_cal_state=0,g_extension_cal_cause=0,g_extension_cal_samples=0;
volatile uint32_t g_extension_cal_options=0,g_extension_cal_encoder_before=0,g_extension_cal_encoder_after=0;
volatile int32_t g_extension_cal_position_before=0,g_extension_cal_position_after=0,g_extension_cal_target_raw=0;
#if !APP_MOTOR_OFFLINE_ONLY
static uint32_t ec_phase,ec_begin,ec_settled,ec_flags,ec_speed,ec_wait_begin,ec_wait;
static uint32_t ec_query(uint8_t c,uint8_t n){
 uint8_t q[2]={c,0x6b};uint32_t now=(uint32_t)xTaskGetTickCount();
 if(VehicleArm_Exchange(6,q,2,n,&s_rx)==1U){ec_wait=0;return 1;}
 if(!ec_wait){ec_wait=1;ec_wait_begin=now;}
 if(now-ec_wait_begin>=400U){g_extension_cal_cause=3;g_extension_cal_state=4;arm_fail(3);}
 return 0;
}
static int32_t ec_position(void){uint32_t r=((uint32_t)s_rx.Data[2]<<24)|((uint32_t)s_rx.Data[3]<<16)|((uint32_t)s_rx.Data[4]<<8)|s_rx.Data[5];return s_rx.Data[1]?-(int32_t)r:(int32_t)r;}
static void ec_fail(uint32_t c){g_extension_cal_state=4;g_extension_cal_cause=c;arm_fail(c);}
static void ExtensionCalibration_Service(void){
 uint32_t now=(uint32_t)xTaskGetTickCount();uint8_t en[5]={0xf3,0xab,1,0,0x6b};
 uint8_t move[15]={0xfd,1,0,60,0,60,0,50,0,0,4,0xe2,2,0,0x6b}; /* 1250 raw FD units, 5RPM */
 if(!StartupAxes_Holding() || now-ec_begin>=10000U){ec_fail(2);return;}
 g_extension_cal_state=2;
 switch(ec_phase){
 case 0:if(!ec_query(0x1a,4))return;if(s_rx.Data[2]&3U){ec_fail(4);return;}g_extension_cal_options=s_rx.Data[2];break;
 case 1:if(!ec_query(0x31,4))return;g_extension_cal_encoder_before=((uint32_t)s_rx.Data[1]<<8)|s_rx.Data[2];break;
 case 2:if(!ec_query(0x36,7))return;if(s_rx.Data[1]>1){ec_fail(4);return;}g_extension_cal_position_before=ec_position();break;
 case 3:if(VehicleArm_Exchange(6,en,5,3,&s_rx)!=1 || s_rx.Data[1]!=2){ec_fail(5);return;}break;
 case 4:if(!ec_query(0x3a,3))return;if(!(s_rx.Data[1]&1) || (s_rx.Data[1]&12)){ec_fail(6);return;}break;
 case 5:if(!ec_query(0x35,5))return;if(s_rx.Data[2] || s_rx.Data[3]){ec_settled=0;ec_phase=4;return;}if(++ec_settled<3){ec_phase=4;return;}ec_settled=0;break;
 case 6:if(VehicleArm_Exchange(6,move,15,3,&s_rx)!=1 || (s_rx.Data[1]!=2 && s_rx.Data[1]!=0x9f)){ec_fail(5);return;}break;
 case 7:if(!ec_query(0x33,7))return;if(s_rx.Data[1]>1){ec_fail(4);return;}g_extension_cal_target_raw=ec_position();break;
 case 8:if(!ec_query(0x3a,3))return;ec_flags=s_rx.Data[1];if(!(ec_flags&1) || (ec_flags&12)){ec_fail(6);return;}break;
 case 9:if(!ec_query(0x36,7))return;if(s_rx.Data[1]>1){ec_fail(4);return;}g_extension_cal_position_after=ec_position();break;
 case 10:if(!ec_query(0x35,5))return;ec_speed=((uint32_t)s_rx.Data[2]<<8)|s_rx.Data[3];break;
 default:if(!ec_query(0x31,4))return;g_extension_cal_encoder_after=((uint32_t)s_rx.Data[1]<<8)|s_rx.Data[2];
  {int32_t diff=(int32_t)g_extension_cal_encoder_after-(int32_t)g_extension_cal_encoder_before;if(diff>32767)diff-=65536;if(diff< -32768)diff+=65536;
   if(diff>3000 || diff< -26000){ec_fail(7);return;}}
  if((ec_flags&2) && !ec_speed)ec_settled++;else ec_settled=0;
  if(ec_settled>=3){g_extension_cal_samples++;g_extension_cal_state=3;g_vehicle_arm_state=2;g_vehicle_arm_axis_owned=0;return;}
  ec_phase=8;return;
 }ec_phase++;
}
#endif
uint32_t VehicleArm_BeginExtensionCalibration(void){
#if APP_MOTOR_OFFLINE_ONLY
 return 0;
#else
 ChassisAutoStatus s;
 if(!VEHICLE_EXTENSION_CALIBRATION_ONLY || VehicleArm_Busy() || g_extension_cal_state==4 || g_extension_cal_samples>=2 || !StartupAxes_Holding())return 0;
 ChassisAuto_ReadStatus(&s);if(s.active)return 0;
 ec_phase=ec_settled=ec_wait=0;ec_begin=(uint32_t)xTaskGetTickCount();g_extension_cal_cause=0;
 g_vehicle_arm_axis_owned=6;g_extension_cal_state=1;g_vehicle_arm_state=1;return 1;
#endif
}
