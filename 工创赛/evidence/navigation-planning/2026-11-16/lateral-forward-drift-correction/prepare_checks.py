from pathlib import Path
E=Path(__file__).parent;O=E.parent/'lateral-field-run1-prep'
s=(O/'build_candidate.py').read_text(encoding='utf8')
s=s.replace("T=R/'工创赛/tmp/lateral-field-run1-build-20261116'","T=R/'工创赛/tmp/lateral-drift-trim-build-20261116'").replace("name='left210_40'","name='left210_trim4'").replace("defines='APP_BASELINE40_ENABLE=1,","defines='APP_LEFT40_DRIFT_TRIM_ENABLE=1,APP_BASELINE40_ENABLE=1,")
(E/'build_candidate.py').write_text(s,encoding='utf8')
s=(O/'check_actual_c.py').read_text(encoding='utf8')
s=s.replace("('active',1),('inactive',0)","('trim_on',1),('trim_off',1),('inactive',0)")
s=s.replace("'-DAPP_BASELINE40_ENABLE='+str(flag),","'-DAPP_LEFT40_DRIFT_TRIM_ENABLE='+str(int(mode=='trim_on')),'-DAPP_BASELINE40_ENABLE='+str(flag),")
s=s.replace('CK(MotionDistanceAcceleration(1,102,2100)==150 && MotionDistanceJerk(1,102,2100)==300);','CK(MotionDistanceAcceleration(1,102,2100)==(APP_LEFT40_DRIFT_TRIM_ENABLE ? 150.0f/1.04f : 150.0f));\n CK(MotionDistanceJerk(1,102,2100)==(APP_LEFT40_DRIFT_TRIM_ENABLE ? 300.0f/1.04f : 300.0f));\n CK(MotionDistanceAcceleration(0,100,2100)==150 && MotionDistanceJerk(0,100,2100)==300);')
needle=' CK(!Chassis_TranslationProfileCommands(1,103,0,cmd));'
extra=r'''
 for(b=0;b<=102;b++)for(z=0;z<3;z++){
  ChassisMotorCommand old[4];int32_t logical[4];float h=(float)((int32_t)z*2-2);uint32_t trim=(uint32_t)(.04f*b+.5f);
  CK(Chassis_Left40DriftCommands(b,h,cmd));
  CK(Chassis_TranslationProfileCommands(1,b,h,old));
  for(i=0;i<4;i++){
   logical[i]=cmd[i].direction ? -(int32_t)cmd[i].speed_raw : (int32_t)cmd[i].speed_raw;
   if(i==1 || i==3)logical[i]=-logical[i];
   CK(cmd[i].speed_raw<=1080);
  }
  CK(logical[0]+logical[1]+logical[2]+logical[3]==-(int32_t)trim*40);
  CK(logical[0]-logical[1]-logical[2]+logical[3]==(int32_t)b*40);
  if(!trim)CK(!memcmp(cmd,old,sizeof(cmd)));
 }
 CK(!Chassis_Left40DriftCommands(103,0,cmd));
 CK(!Chassis_Left40DriftCommands(102,3,cmd));
 CK(!Chassis_Left40DriftCommands(102,0,0));
'''
assert needle in s;s=s.replace(needle,extra+needle)
(E/'check_actual_c.py').write_text(s,encoding='utf8')
