/* Appended to the existing fixture, after renaming its main(). */
static void geometry_run(uint32_t recovery)
{
 uint32_t tick,item,phase,ring,placed=0,recovered=0;
 static const uint32_t recovery_actions[12]={3,3,3,5,4,3,3,10,0,17,4,0};
 for(tick=0;tick<1000 && g_stage3_state==5;tick++){
  if(!s3_pending)service(tick*1000);
  if(g_stage3_state!=5)break;
  item=g_stage3_item;phase=g_stage3_phase;
  if(!s3_pending)continue;
  assert(item<3);
  ring=s3_positions[item]-1;
  if(phase==9){
   assert(g_it_request==3 && g_it_axes==(SPF_EXT|SPF_LIFT));
   assert(g_it_extension_mm==candidate_x[ring] && g_it_lift_pulses==candidate_place_z[ring]);
   placed|=1U<<ring;
  }
  if(phase>=17 && phase<=28){
   assert(recovery && g_stage3_placed_count==3 && g_it_request==recovery_actions[phase-17]);
   switch(phase){
   case 18: assert(g_it_axes==SPF_EXT && g_it_extension_mm==candidate_x[ring]);break;
   case 19: assert(g_it_axes==SPF_LIFT && g_it_lift_pulses==candidate_recover_z[ring]);break;
   case 20: assert(!top);break;
   case 22: assert(top && g_it_axes==SPF_EXT && g_it_extension_mm==0);break;
   case 24: assert(g_it_carrier_deg/120==1U+(item==0)-(item==1));break;
   case 26: assert(top);recovered|=1U<<ring;break;
   default: break;
   }
  }
  service(tick*1000);assert(s3_pending && g_stage3_phase==phase);
  if(phase==5 || phase==9 || phase==19)top=0;
  if(phase==12 && ring==1)top=1;
  completed(tick*1000);
 }
 assert(g_stage3_state==6 && g_stage3_placed_count==3 && !fault);
 assert(g_stage3_recovered_count==(recovery?3U:0U));
 assert(recovered==(recovery?7U:0U));
 if(recovery!=2)assert(placed==7);
}

int main(void)
{
 uint32_t n,mode;
 assert(baseline_main()==0);
 for(mode=0;mode<2;mode++){
  top=1;fault=g_stage3_fault=g_stage3_rejected=g_it_request=s3_pending=0;
  g_stage3_task_run=g_stage3_task_batch=1;
  g_stage3_cycle_mode=mode;
  g_stage3_colors[0]=5;g_stage3_colors[1]=1;g_stage3_colors[2]=2;
  g_stage3_positions[0]=3;g_stage3_positions[1]=1;g_stage3_positions[2]=2;
  g_stage3_slot_colors[0]=1;g_stage3_slot_colors[1]=2;g_stage3_slot_colors[2]=5;
  for(n=0;n<3;n++){
   g_stage3_target_x[n]=candidate_x[n];
   g_stage3_target_depth[n]=candidate_place_z[n];
   g_stage3_recovery_depth[n]=mode?candidate_recover_z[n]:-1.0f;
  }
  assert(S3_Config());S3_LockConfig();
  geometry_run(mode);
  if(!mode){
   request32(32);assert(g_stage3_state==6 && g_stage3_rejected==1);
   for(n=0;n<3;n++)g_stage3_recovery_depth[n]=candidate_recover_z[n];
   g_stage3_target_x[2]=99.0f;g_stage3_positions[0]=2;g_stage3_slot_colors[2]=1;
   request32(32);assert(g_stage3_state==5 && g_stage3_phase==17);
   geometry_run(2);
   request32(32);assert(g_stage3_state==6 && g_stage3_rejected==2);
  }
 }
 puts("PASS actual geometry: shared X, separate Z, original slots, top before retract/release, request 32 locks X/mapping");
 return 0;
}
