static uint32_t clock_ms, pwm_counts, pwm_failed, pwm_slot, s3_low_handoff, g_stage3_fault_flags;
static uint32_t g_stage3_fast_timing_mask,g_stage3_fast_align_tick,g_stage3_fast_place_tick,g_stage3_fast_recover_tick;
static uint32_t g_stage3_tray_prefetch_writes;
static uint32_t g_vehicle_lift_feedback_valid=1;
#define S3_FAST_HANDOFF_OFFSET_PULSES 8000.0f
static uint32_t xTaskGetTickCount(void){return clock_ms;}
static uint32_t Stage3_Owns(void){return 1U;}
static uint32_t CarrierPwm_Write(uint32_t pulse){
 pwm_counts++;
 if(pwm_failed)return 0;
 assert(pulse==500 || pulse==1388 || pulse==2277);
 pwm_slot=pulse==500?0:pulse==1388?1:2;return 1;
}
// INSERT_STAGE3_FAST

static void setup(uint32_t mode){
 uint32_t i;
 static const float place[3]={61802.22f,60795.02f,63002.22f}, rxz[3]={61802.22f,61595.02f,63002.22f};
 static const float px[3]={43,0,70.5f};
 g_stage3_cycle_mode=mode;g_stage3_task_batch=g_stage3_task_run=1;
 g_stage3_first_mapping_valid=s3_pending=g_it_request=fault=g_stage3_fault=g_stage3_fault_flags=0;
 g_stage3_rejected=s3_tray_valid=clock_ms=pwm_counts=pwm_failed=0;
 g_stage3_fast_timing_mask=1;g_stage3_fast_align_tick=clock_ms;
 g_stage3_fast_place_tick=g_stage3_fast_recover_tick=0;top=1;
 for(i=0;i<3;i++){
  g_stage3_target_depth[i]=place[i];g_stage3_recovery_depth[i]=rxz[i];g_stage3_target_x[i]=px[i];
 }
 assert(S3_Config());S3_LockConfig();
}

static void tick(void){
 clock_ms+=25;
 if(!s3_pending){service(clock_ms);return;}
 if(g_stage3_phase==5 || g_stage3_phase==9 || g_stage3_phase==19)top=0;
 if(g_it_request==4 || (g_stage3_phase==12 && g_it_request==3 && !s3_low_handoff) ||
    (g_stage3_phase==21 && g_it_request==3))top=1;
 if(g_stage3_phase==12 && s3_low_handoff==1)s3_low_handoff=2; /* Stub the backend's XZ completion. */
 complete=1;service(clock_ms);complete=0;
}

int main(void){
 uint32_t n,placed_xz=0,low_edges=0,return_xyz=0,recovery_pwm=0,releases=0,last_pwm=0;
 static const uint32_t slots[3]={2,0,1};
 setup(1);
 for(n=0;n<1500 && g_stage3_state==5;n++){
  tick();
  if(!s3_pending)continue;
  if(g_stage3_phase==12){
   assert(S3_SyncPlaceOwned());
   assert(g_it_axes==(SPF_EXT|SPF_LIFT) && g_it_extension_mm==0.0f && g_it_request==3);
   if(s3_low_handoff){assert(g_stage3_item==2 && g_it_lift_pulses==s3_depth[1]-8000.0f);low_edges++;}
   else assert(g_it_lift_pulses==S3_SafeTopPulses());
   placed_xz++;
  }
  assert(g_stage3_phase!=29);
  if(g_stage3_phase==17 || g_stage3_phase==18){
   assert(g_stage3_placed_count==3 && g_stage3_item<3);
   assert(S3_LowHandoffOwned()==(g_stage3_item==0));
   if(g_stage3_item==0){
    float *goal=g_stage3_phase==17?&g_it_dm_deg:&g_it_extension_mm;
    *goal+=1;assert(!S3_LowHandoffOwned());*goal-=1;
    g_stage3_locked_run++;assert(!S3_LowHandoffOwned());g_stage3_locked_run--;
   }
  }
  if(g_stage3_phase==19){
   assert(!s3_low_handoff && pwm_counts>last_pwm && pwm_slot==slots[g_stage3_item]);
   last_pwm=pwm_counts;recovery_pwm++;
  }
  if(g_stage3_phase==21){
   assert(S3_FastRecoveryOwned() && g_it_axes==(SPF_DM|SPF_EXT|SPF_LIFT) && g_it_request==3);
   assert(g_it_dm_deg==s3_carrier && g_it_extension_mm==0 && g_it_lift_pulses==S3_SafeTopPulses());return_xyz++;
  }
  if(g_stage3_phase==26){
   assert(clock_ms-s3_tray_tick>=700 && top && g_it_request==17 && pwm_counts==last_pwm);releases++;
  }
  assert(g_stage3_phase!=27 || g_it_request!=4);
 }
 assert(!fault && g_stage3_state==6 && g_stage3_placed_count==3 && g_stage3_recovered_count==3);
 assert(placed_xz==3 && low_edges==1 && return_xyz==3 && recovery_pwm==3 && releases==3);
 assert(g_stage3_fast_timing_mask==7 && g_stage3_fast_place_tick>g_stage3_fast_align_tick &&
        g_stage3_fast_recover_tick>g_stage3_fast_place_tick);

 /* A profile with a shallower final place must use the calibrated safe top. */
 setup(1);g_stage3_phase=12;g_stage3_item=2;
 s3_positions[0]=2;s3_positions[2]=3;s3_depth[2]=50000;
 service(clock_ms);assert(!s3_low_handoff && g_it_lift_pulses==S3_SafeTopPulses());
 setup(1);g_stage3_phase=27;top=0;service(clock_ms);assert(g_it_request==4);

 setup(0);
 for(n=0;n<1500 && g_stage3_state==5;n++)tick();
 assert(g_stage3_state==6 && g_stage3_placed_count==3 && !g_stage3_recovered_count && !s3_low_handoff);
 s3_tray_valid=1;g_stage3_tray_prefetch_item=0;
 top=0;request32(32);assert(g_stage3_state==6 && g_stage3_rejected==1);
 top=1;request32(32);assert(g_stage3_state==5 && !s3_low_handoff && !s3_tray_valid);
 pwm_failed=1;
 for(n=0;n<20 && g_stage3_state==5;n++)tick();
 assert(fault==9 && g_stage3_state==9 && !s3_pending && !g_stage3_recovered_count);

 setup(0);g_stage3_cycle_mode=2;g_stage3_task_batch=2;
 g_stage3_first_mapping_valid=1;
 for(n=0;n<3;n++){g_stage3_first_colors[n]=g_stage3_colors[n];g_stage3_first_positions[n]=g_stage3_positions[n];}
 assert(S3_Config());S3_LockConfig();
 for(n=0;n<3;n++)assert(s3_depth[n]==g_stage3_target_depth[n]-24000.0f);
 for(n=0;n<1500 && g_stage3_state==5;n++)tick();
 assert(g_stage3_state==6 && g_stage3_placed_count==3 && !g_stage3_recovered_count && !s3_low_handoff);
 puts("PASS actual Stage3 switch: mode1 XZ returns, one low handoff, original-slot PWM, 700 ms, XYZ request, timing; mode0/2, request32/PWM fault");
 return 0;
}
