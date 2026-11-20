"""Narrow paired ARM serial replay; exact firmware bodies, controlled external I/O."""
import binascii,re,struct,subprocess,sys

def replay(c,old,serial,pp,opts,label,fixture):
    oldlines=old.splitlines(True);block=''.join(oldlines[180:851]);send=''.join(oldlines[108:117])
    oldbody=send+block+'\n#if APP_SERIAL_MOTION_CONTROL\nvoid AppSerial_Init(void){\n'+''.join(oldlines[1195:1197])+'}\nvoid AppSerial_Telemetry(void){\n'+''.join(oldlines[1254:1274])+'}\n#endif\n'
    newbody=re.sub(r'^#include .*\n','',serial,flags=re.M)
    functions=c.functions(serial);owned=sorted(set(re.findall(r'\bs_serial_\w+|\bg_serial_motion_\w+',block)))
    renamed=owned+list(functions)+['AppSerial_Live']
    def rename(s):return re.sub(r'\b('+'|'.join(renamed)+r')\b',lambda m:'old_'+m[0],s)
    includes=''.join(re.findall(r'^#include .*\n',serial,re.M))
    # Use actual header declarations to define only externally owned observed variables.
    used=set(re.findall(r'\bg_\w+',serial));defs={}
    for m in re.finditer(r'^extern\s+([^;]+);',pp,re.M):
        decl=m[1];names=re.findall(r'\bg_\w+',decl)
        if names and any(n in used for n in names) and not any(n in owned for n in names):
            assert '(' not in decl,decl
            defs[names[0]]=decl+';'
    globals_text='\n'.join(defs.values())
    shared=sorted(set(n for d in defs.values() for n in re.findall(r'\bg_\w+',d)))
    prefix=includes+'\n#include <string.h>\n'+globals_text+'\n'
    if fixture:prefix+='\n#undef VEHICLE_INTEGRATED_ENABLE\n#define VEHICLE_INTEGRATED_ENABLE 0U\n#undef VEHICLE_STAGE1_ENABLE\n#define VEHICLE_STAGE1_ENABLE 0U\n'
    prefix+='#define TEST_PROTOCOL_FIXTURE '+str(int(fixture))+'U\n'
    stubs=r'''
uint32_t clock_ms,depth,trace[2048],trace_count,send_ok,field_local,settings_ok,health_ok,exclusive_ok,endpoint_outcome;
uint32_t sent_used,sent_count,send_attempts,send_failures,stop_calls,route_calls,command_cases,safety_cases,telemetry_cases;
uint32_t live_count=2U,live_session,live_has,live_turn,live_mode,live_rpm=6U,live_begin,live_end,live_stage;
float live_distance=250.0f,live_reference;MotionAction live_actions[8];float live_targets[9][3];
AppMotionObservation live={&live_count,&live_session,&live_has,&live_turn,&live_mode,&live_rpm,&live_distance,&live_reference,&live_begin,&live_end,&live_stage,live_actions,(const float (*)[3])live_targets};
RuntimeRoute route;ChassisAutoStatus backend;PowerCommandStatus power;AppHeadingSource heading;
char sent[8192],samples[8][1024];uint32_t sample_mask;
void event(uint32_t v){if(trace_count<2048U)trace[trace_count++]=v;}
void enter(void){event(100U+depth++);}void leave(void){event(200U+--depth);}
#undef taskENTER_CRITICAL
#undef taskEXIT_CRITICAL
#define taskENTER_CRITICAL() enter()
#define taskEXIT_CRITICAL() leave()
TickType_t xTaskGetTickCount(void){event(1U);return clock_ms;}
uint8_t USART1_SendString(const char *s){uint32_t n=strlen(s),i;const char *ids[8]={"TMQ,","PE3,","OP1,","OD1,","WF1,","PM1,","BTQ,","XD1,"};event(2U);event(n);event(send_ok);send_attempts++;if(!send_ok)send_failures++;
 if(sent_used+n+1U<sizeof(sent)){memcpy(sent+sent_used,s,n+1U);sent_used+=n+1U;}sent_count++;
 for(i=0;i<8U;i++)if(!(sample_mask&(1U<<i))&&!strncmp(s,ids[i],4U)&&n<1024U){memcpy(samples[i],s,n+1U);sample_mask|=1U<<i;}return (uint8_t)send_ok;}
uint32_t FieldMap_LocalMode(void){event(3U);return field_local;}
uint32_t CAN1_EmmSettingsConfirmed(void){event(4U);return settings_ok;}
uint32_t ChassisAuto_ReadStatus(ChassisAutoStatus *s){event(5U);*s=backend;return 1U;}
void Power_ReadCommandStatus(PowerCommandStatus *s){event(6U);*s=power;}
uint32_t App_HeadingSourceRead(AppHeadingSource *s){event(7U);*s=heading;return 1U;}
uint32_t StartupAxes_Holding(void){event(8U);return 1U;}
uint32_t App_HeadingRuntimeSource(uint32_t n){event(9U);event(n);return health_ok;}
uint32_t App_HeadingRuntimeExclusive(void){event(10U);return exclusive_ok;}
const AppMotionObservation *AppMotion_Live(void){event(11U);return &live;}
const RuntimeRoute *AppRuntimeRoute_Live(void){event(12U);return &route;}
void AppRuntimeRoute_Init(void){event(13U);memset(&route,0,sizeof(route));}
uint32_t AppMotion_SetStartPermit(uint32_t permit){event(14U);event(permit);return permit;}
void App_CombinedTestStop(uint32_t cause){event(15U);event(cause);stop_calls++;g_combined_test.cause=cause;g_combined_test.state=5U;}
uint32_t App_DebugSelectPlan(uint32_t id){event(16U);event(id);return id<=DEBUG_PLAN_MAX;}
void App_RuntimeRouteCommand(const char *s){uint32_t i;event(17U);route_calls++;for(i=0;s[i];i++)event((uint32_t)s[i]);}
uint32_t MotionGetState(void){event(18U);return MOTION_DONE;}
float MotionRpmToCmPerSec(float rpm){event(19U);return rpm*0.4f;}
void AppMotion_Stage1Telemetry(void){event(20U);}void AppMotion_Stage1CodeTelemetry(void){event(21U);}
void App_RuntimeRouteTelemetry(void){event(22U);}void App_MotionFaultTelemetry(void){event(23U);}void App_MotionTimingTelemetry(void){event(24U);}
uint32_t ChassisAuto_ReadPositionEndpoints(uint32_t index,ChassisPositionEndpoints *e){uint32_t i;event(25U);event(index);memset(e,0,sizeof(*e));e->session=7U;e->outcome=endpoint_outcome;e->start_mask=e->end_mask=15U;for(i=0;i<4U;i++){e->start_raw[i]=100U+i;e->end_raw[i]=200U+i;e->end_ms[i]=clock_ms;}return index<8U;}
uint32_t ChassisAuto_ReadPositions(ChassisPositionSnapshot *s){uint32_t i;event(26U);memset(s,0,sizeof(*s));s->session=5U;s->sequence=8U;s->valid_mask=15U;for(i=0;i<4U;i++){s->raw[i]=12345U+i;s->sample_ms[i]=clock_ms-i;}return 1U;}
uint32_t ChassisAuto_ReadWheelLog(ChassisWheelLog *s){uint32_t i;event(27U);memset(s,0,sizeof(*s));s->session=7U;s->active=1U;s->tick_ms=clock_ms;for(i=0;i<4U;i++){s->acknowledged_decirpm[i]=30+i;s->measured_decirpm[i]=29+i;}return 1U;}
uint32_t ChassisAuto_ReadBatchDiagnostic(uint32_t index,ChassisBatchDiagnostic *r,ChassisBatchStatistics *s){event(28U);event(index);memset(r,0,sizeof(*r));memset(s,0,sizeof(*s));r->session=7U;r->batch_id=index;r->correction_rpm=-0.3f;r->pending_rpm=1.2f;r->base_rpm=5U;return index<8U;}
uint32_t ChassisLoad_Read(uint32_t index,ChassisLoadRecord *r,ChassisLoadStatistics *s){event(29U);event(index);memset(r,0,sizeof(*r));memset(s,0,sizeof(*s));s->session=clock_ms/10000U;s->count=8U;return index<8U;}
'''
    harness=prefix+stubs+newbody+'\n'+rename(oldbody)
    values=[n for n in owned if n!='s_serial_observation']
    harness+='\nvolatile uint32_t assertions,cases;\n#define CHECK(x) do{assertions++;if(!(x))return __LINE__;}while(0)\nuint32_t equal(void){\n'
    for n in values:harness+=f'CHECK(memcmp((void*)&{n},(void*)&old_{n},sizeof({n}))==0);\n'
    harness+='return 0U;}\nvoid sync(void){\n'
    for n in values:harness+=f'memcpy((void*)&old_{n},(void*)&{n},sizeof({n}));\n'
    harness+='}\nuint32_t saved_external[4096];\nvoid save_external(void){uint32_t i=0U;\n'
    for n in shared:harness+=f'memcpy(saved_external+i,(void*)&{n},sizeof({n}));i+=(sizeof({n})+3U)/4U;\n'
    harness+='}\nuint32_t equal_external(void){uint32_t i=0U;\n'
    for n in shared:harness+=f'CHECK(memcmp(saved_external+i,(void*)&{n},sizeof({n}))==0);i+=(sizeof({n})+3U)/4U;\n'
    harness+='return 0U;}\nvoid environment(uint32_t n){trace_count=depth=sent_used=sent_count=0U;settings_ok=health_ok=exclusive_ok=1U;field_local=0U;endpoint_outcome=n%4U==0U?2U:1U;send_ok=n%3U!=0U;\n'
    for n in shared:harness+=f'memset((void*)&{n},0,sizeof({n}));\n'
    harness+=r'''
 memset(&route,0,sizeof(route));route.ready=1U;memset(&backend,0,sizeof(backend));memset(&power,0,sizeof(power));memset(&heading,0,sizeof(heading));
 heading.yaw_deg=-12.345f;heading.rate_dps=0.2f;heading.healthy=1U;heading.sample_ms=clock_ms-4U;
 g_motion_plan_valid=g_translation_hold_selftest=1U;g_forward_hold.target_deg=1.5f;g_forward_hold.error_deg=-2.25f;
 g_forward_hold.correction_rpm=-0.4f;g_heading_exec_rpm=0.5f;g_heading_exec_target_deg=90.0f;g_heap_min_bytes=12345U;
 live_turn=n%2U;live_mode=n%4U;live_has=n%5U==0U;live_session=n%3U==0U;live_begin=clock_ms-100U;live_end=clock_ms-10U;
 live_actions[0]=(MotionAction){MOTION_FORWARD_DISTANCE,6U,0U,0.0f,250.0f,0.0f};
 g_chassis_odometry.x_mm=-12.5f;g_chassis_odometry.y_mm=83.25f;g_chassis_odometry.heading_deg=-90.0f;
}
char command[512];uint32_t input_case;
void make_command(uint32_t n){uint32_t kind=n%16U,seq=9U,tick=clock_ms-10U;char head[400];const char *verb="ARM";
 input_case=kind;
 switch(kind){case 3:verb="ARM";break;case 4:tick=clock_ms-500U;break;case 5:seq=1U;break;case 6:verb="START";break;
 case 7:case 8:case 9:verb="KEEP";break;case 10:verb="STOP";tick=0U;seq=0U;break;case 11:verb="PLAN0";break;case 14:verb="KEEP,EXTRA";break;}
 if(kind==3U)sprintf(head,"RC1,4294967296,%lu,ARM,",(unsigned long)tick);else sprintf(head,"RC1,%lu,%lu,%s,",(unsigned long)seq,(unsigned long)tick,verb);
 sprintf(command,"%s%04X\n",head,App_SerialCrc(head,strlen(head)));
 if(kind==1U)command[strlen(command)-2U]=command[strlen(command)-2U]=='0'?'1':'0';
 if(kind==2U)strcpy(command,"BAD,9,10,ARM,0000\n");
 if(kind==12U)strcpy(command,"RQ1,9,9990,77,GO,4660,ABCD\n");
 if(kind==13U)command[strlen(command)-2U]='g';
 if(kind==15U){memset(command,'X',RQ_LINE_MAX+5U);command[RQ_LINE_MAX+5U]='\n';command[RQ_LINE_MAX+6U]=0;}
}
void input_environment(uint32_t n){environment(n);if(n/16U==1U)field_local=1U;if(n/16U==2U)settings_ok=0U;
 if(input_case==8U){g_combined_test.state=3U;g_combined_test.completed=live_count;backend.result=1U;backend.rotation_stop_mask=15U;}}
void input(uint32_t old){uint32_t i,j;for(j=0;j<((input_case==7U||input_case==12U)?2U:1U);j++)for(i=0;command[i];i++){
 if(i==5U&&(input_case==16U||input_case==17U))clock_ms+=input_case==16U?199U:200U;
 if(i==5U&&(input_case==18U||input_case==19U)){uint8_t b=input_case==18U?31U:'\r';if(old)old_App_SerialMotionByte(b);else App_SerialMotionByte(b);}
 if(old)old_App_SerialMotionByte((uint8_t)command[i]);else App_SerialMotionByte((uint8_t)command[i]);}}
void telemetry(uint32_t old,uint32_t op){
 if(op==0U){if(old)old_App_SerialMotionTelemetry();else App_SerialMotionTelemetry();}
 if(op==1U){if(old)old_App_PositionEndpointTelemetry();else App_PositionEndpointTelemetry();}
 if(op==2U){if(old)old_App_PerimeterTelemetry();else App_PerimeterTelemetry();}
 if(op==3U){if(old)old_App_OdometryTelemetry();else App_OdometryTelemetry();}
 if(op==4U){if(old)old_App_WheelLogTelemetry();else App_WheelLogTelemetry();}
 if(op==5U){if(old)old_App_PositionTelemetry();else App_PositionTelemetry();}
 if(op==6U){if(old)old_App_BatchDiagnosticTelemetry();else App_BatchDiagnosticTelemetry();}
 if(op==7U){if(old)old_App_ChassisLoadTelemetry();else App_ChassisLoadTelemetry();}
 if(op==8U){if(old)old_AppSerial_Telemetry();else AppSerial_Telemetry();}
}
void telemetry_environment(uint32_t n,uint32_t op){environment(n);g_combined_test.state=3U;if(op==4U)g_chassis_auto_active=1U;}
uint32_t saved_trace[2048],saved_num,saved_sent_used,saved_sent_count;char saved_sent[8192];
void save(void){saved_num=trace_count;memcpy(saved_trace,trace,trace_count*4U);saved_sent_used=sent_used;saved_sent_count=sent_count;memcpy(saved_sent,sent,sent_used);save_external();}
uint32_t compare(void){uint32_t r;CHECK(depth==0U);CHECK(saved_num==trace_count);CHECK(memcmp(saved_trace,trace,trace_count*4U)==0);CHECK(saved_sent_used==sent_used);CHECK(saved_sent_count==sent_count);CHECK(memcmp(saved_sent,sent,sent_used)==0);r=equal();CHECK(!r);r=equal_external();CHECK(!r);cases++;return 0U;}
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){uint32_t n,op,r,oldtick,oldseq,olddrop,first_sent;
 CHECK(App_SerialCrc("123456789",9U)==0x29B1U);CHECK(old_App_SerialCrc("123456789",9U)==0x29B1U);
 clock_ms=1000U;environment(0U);AppSerial_Init();save();environment(0U);old_AppSerial_Init();CHECK(compare()==0U);
 CHECK(AppSerial_Live()->token_seq==&s_serial_token_seq&&AppSerial_Live()->tx_seq==&s_serial_tx_seq);
 for(n=0U;n<64U;n++){
  clock_ms=10000U+n*1000U;make_command(n);g_serial_motion_state=input_case==7U?2U:(input_case==8U?3U:0U);g_serial_motion_result=input_case==8U?2U:0U;
  g_serial_motion_accepted=g_serial_motion_rejected=g_serial_motion_link_lost=0U;s_serial_proto_ok=1U;s_serial_tx_seq=10U;
  s_serial_token_seq=9U;s_serial_token_tick=clock_ms-10U;s_serial_arm_ms=clock_ms-100U;s_serial_keepalive_ms=clock_ms-100U;sync();
  input_environment(n);input(0U);CHECK(depth==0U);if(!TEST_PROTOCOL_FIXTURE&&input_case!=15U)CHECK(g_serial_motion_accepted==0U&&g_serial_motion_rejected==0U);
  if(TEST_PROTOCOL_FIXTURE&&n<16U){
   if(n==0U)CHECK(g_serial_motion_state==1U&&g_serial_motion_accepted==1U&&s_serial_token_seq==9U&&s_serial_token_tick==clock_ms-10U);
   if(n==1U||n==2U||n==3U||n==13U||n==14U||n==15U)CHECK(g_serial_motion_result==4U&&g_serial_motion_rejected==1U);
   if(n==4U||n==5U)CHECK(g_serial_motion_result==5U&&g_serial_motion_rejected==1U);
   if(n==6U||n==9U)CHECK(g_serial_motion_result==6U&&g_serial_motion_rejected==1U);
   if(n==7U)CHECK(g_serial_motion_accepted==2U&&s_serial_keepalive_ms==clock_ms);
   if(n==8U||n==11U)CHECK(g_serial_motion_accepted==1U);
   if(n==10U)CHECK(g_serial_motion_state==3U&&g_serial_motion_result==3U&&g_combined_test.state==5U);
  }save();
  input_environment(n);input(1U);r=compare();CHECK(!r);command_cases++;
 }
 for(n=16U;n<20U;n++){
  clock_ms=80000U+(n-16U)*1000U;make_command(0U);input_case=n;g_serial_motion_state=g_serial_motion_result=g_serial_motion_accepted=g_serial_motion_rejected=0U;
  s_serial_proto_ok=1U;s_serial_tx_seq=10U;s_serial_token_seq=9U;s_serial_token_tick=clock_ms-10U;sync();
  input_environment(0U);input(0U);if(n==18U)CHECK(g_serial_motion_rejected==1U);
  if(TEST_PROTOCOL_FIXTURE&&(n==16U||n==19U))CHECK(g_serial_motion_accepted==1U);
  if(TEST_PROTOCOL_FIXTURE&&n==17U)CHECK(g_serial_motion_rejected==1U);save();
  clock_ms=80000U+(n-16U)*1000U;input_environment(0U);input(1U);r=compare();CHECK(!r);command_cases++;
 }
 for(n=0U;n<48U;n++){
  clock_ms=90000U+n*1000U;g_serial_motion_state=(n%3U)+1U;g_serial_motion_result=2U;g_serial_motion_link_lost=0U;
  s_serial_arm_ms=clock_ms-(n%2U?2000U:1999U);s_serial_keepalive_ms=clock_ms-(n%2U?1000U:999U);sync();
  environment(n);g_combined_test.state=n%4U;App_SerialMotionSafety();save();environment(n);g_combined_test.state=n%4U;old_App_SerialMotionSafety();r=compare();CHECK(!r);safety_cases++;
 }
 for(n=0U;n<32U;n++)for(op=0U;op<9U;op++){
  clock_ms=200000U+n*2000U+op*150U;s_serial_tx_seq=200U+n;s_serial_tx_valid=n%7U!=0U;
  s_serial_tx_tick=clock_ms-(op==0U?101U:80U);s_serial_dropped=4U;g_serial_motion_state=3U;g_serial_motion_result=2U;sync();
  oldtick=s_serial_tx_tick;oldseq=s_serial_tx_seq;olddrop=s_serial_dropped;
  telemetry_environment(n,op);telemetry(0U,op);CHECK(depth==0U);
  if(op==0U){CHECK(s_serial_tx_seq==oldseq+1U);CHECK(s_serial_tx_tick==(send_ok?clock_ms:oldtick));CHECK(s_serial_dropped==olddrop+(send_ok?0U:1U));}
  if(op!=0U&&op!=8U&&!s_serial_tx_valid)CHECK(sent_count==0U);
  if(!send_ok){first_sent=sent_count;telemetry(0U,op);if(op==4U||op==5U)CHECK(sent_count==first_sent);else if(op!=8U&&first_sent)CHECK(sent_count==2U*first_sent);}
  save();telemetry_environment(n,op);telemetry(1U,op);if(!send_ok)telemetry(1U,op);r=compare();CHECK(!r);telemetry_cases++;
 }
 CHECK(send_attempts>0U&&send_failures>0U);if(TEST_PROTOCOL_FIXTURE)CHECK(stop_calls>0U&&route_calls>0U);return 0U;
}
'''
    out=c.T/('unit_'+label);out.mkdir(exist_ok=True);(out/'unit.c').write_bytes(harness.encode('gbk'))
    (out/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n')
    simple=[x for x in opts if x not in ['--cpu','Cortex-M4.fp.sp']]
    commands=[[str(c.B/'armcc.exe'),'--cpu','Cortex-M3','--gnu']+simple+['--split_sections','-O1','-c','unit.c','-o','unit.o'],[str(c.B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','--library_type=microlib','-o','unit.axf']]
    for cmd in commands:
        p=subprocess.run(cmd,cwd=out,capture_output=True,timeout=60);assert p.returncode==0,(p.stdout+p.stderr).decode(errors='replace')
    sys.path.insert(0,str(c.R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
    from unicorn import arm_const as reg
    blob=(out/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',blob);m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000)
    # Keil scatter places initialized RAM in the ROM LOAD segment. Load ALLOC
    # sections at their execution addresses, including initialized RW data.
    initialized_rw_bytes=0
    for i in range(elf[12]):
        _,kind,flags,addr,off,size,_,_,_,_=struct.unpack_from('<IIIIIIIIII',blob,elf[6]+i*elf[11])
        if flags&2 and kind==1 and size:m.mem_write(addr,blob[off:off+size])
        if flags&3==3 and kind==1:initialized_rw_bytes+=size
    stop=0x080FF000;m.reg_write(reg.UC_ARM_REG_SP,0x2001FFF0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(elf[4]|1,stop,timeout=60000000,count=300000000)
    assert m.reg_read(reg.UC_ARM_REG_PC)==stop;assert m.reg_read(reg.UC_ARM_REG_R0)==0,f'C check line {m.reg_read(reg.UC_ARM_REG_R0)}'
    symbols=subprocess.check_output([str(c.B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out).decode(errors='replace')
    def address(n):return int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
    def value(n):return struct.unpack('<I',m.mem_read(address(n),4))[0]
    entry_values=[struct.unpack('<I',m.mem_read(int(a,16),4))[0] for a in re.findall(r'\bsource_parameter_entry\s+(0x[0-9a-fA-F]+)',symbols)]
    assert len(entry_values)==2 and entry_values==([0,0] if fixture else [1,1]),entry_values
    samples={}
    for i in range(8):
        if value('sample_mask')&(1<<i):
            b=bytes(m.mem_read(address('samples')+i*1024,1024)).split(b'\0')[0];prefix,crc=b.rsplit(b',',1);assert int(crc.strip(),16)==binascii.crc_hqx(prefix+b',',0xFFFF)
            samples[b[:3].decode()]=b.decode('ascii')
    assert 'TMQ' in samples and len(samples)>=5
    result={n:value(n) for n in ['cases','assertions','command_cases','safety_cases','telemetry_cases','send_attempts','send_failures','stop_calls','route_calls']}
    result.update(frame_samples=samples,python_crc_verified=len(samples),initialized_rw_bytes=initialized_rw_bytes,integrated_entry_values=entry_values,unit_sha256=c.sha(out/'unit.axf'))
    return result
