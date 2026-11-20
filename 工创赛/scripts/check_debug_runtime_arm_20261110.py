"""Paired original/new debug C with real heading controller and wheel encoding.

Only the isolated fixture enables dormant UART/static bodies. It supplies the
already-undefined legacy `value` input; this does not repair a product profile.
"""
import json,re,struct,subprocess,sys

def replay(c,old,debug,pp,opts,label,fixture):
 watch,heading,uart,observe=c.blocks(old)
 oldbody=watch+heading+'#if APP_HWT_STATIC_TEST && !APP_HWT_TURN_TEST\nvoid AppDebug_HeadingObserve(void){\n'+observe+'}\n#endif\n'+uart
 newbody=re.sub(r'^#include .*\n','',debug,flags=re.M)
 owned=sorted(set(re.findall(r'\bg_heading_(?:bus|publish|dry|live)_\w+|\bs_heading_live_\w+|\bg_uart_link_\w+|\bs_link_\w+',watch+heading+uart)))
 active=owned if fixture else [n for n in owned if n.startswith('g_heading_dry_')]
 rename_names=owned+list(c.functions(debug))
 def rename(s):return re.sub(r'\b('+'|'.join(rename_names)+r')\b',lambda m:'old_'+m[0],s)
 includes=''.join(re.findall(r'^#include .*\n',debug,re.M))
 used=set(re.findall(r'\bg_\w+',debug));defs={}
 for m in re.finditer(r'^extern\s+([^;]+);',pp,re.M):
  decl=m[1];names=re.findall(r'\bg_\w+',decl)
  if names and any(n in used for n in names) and not any(n in owned or n=='g_heading_controller_default_config' for n in names):
   assert '(' not in decl;defs[names[0]]=decl+';'
 prefix=includes+'\n'+'\n'.join(defs.values())+'\n'
 if fixture:prefix+='\n#undef APP_HWT_STATIC_TEST\n#define APP_HWT_STATIC_TEST 1U\n#undef APP_HWT_TURN_TEST\n#define APP_HWT_TURN_TEST 0U\n#undef APP_UART_LINK_TEST\n#define APP_UART_LINK_TEST 1U\nAppHeadingSource value={0}; /* Fixture only: missing historical input. */\n'
 prefix+='#define TEST_DEBUG_FIXTURE '+str(int(fixture))+'U\n'
 stubs=r'''
uint32_t clock_ms,depth,trace[512],trace_count,send_ok=1U;
uint32_t sent_used,send_attempts,send_failures,publish_calls,observe_calls,read_calls;
char sent[2048];
ChassisHeadingRequest bus={0,1U,0U};uint32_t bus_valid=1U;
const HeadingControllerConfig *fixture_config=&g_heading_controller_default_config;
volatile uint32_t assertions,cases,uart_cases,heading_cases,priority_cases;
void event(uint32_t n){if(trace_count<512U)trace[trace_count++]=n;}
void enter(void){event(100U+depth++);}void leave(void){event(200U+--depth);}
#undef taskENTER_CRITICAL
#undef taskEXIT_CRITICAL
#define taskENTER_CRITICAL() enter()
#define taskEXIT_CRITICAL() leave()
TickType_t xTaskGetTickCount(void){event(1U);return clock_ms;}
uint8_t App_DebugSendString(const char *s){uint32_t n=strlen(s);event(2U);event(n);event(send_ok);send_attempts++;if(!send_ok)send_failures++;
 if(sent_used+n+1U<sizeof(sent)){memcpy(sent+sent_used,s,n+1U);sent_used+=n+1U;}return (uint8_t)send_ok;}
void App_HeadingSourceObserve(void){event(3U);observe_calls++;}
uint32_t ChassisAuto_HeadingPublish(float command_rpm,uint32_t stop_required){event(4U);publish_calls++;bus.command_rpm=command_rpm;bus.stop_required=stop_required;bus.sequence++;return 1U;}
uint32_t ChassisAuto_HeadingRead(ChassisHeadingRequest *r){event(5U);read_calls++;*r=bus;return bus_valid;}
#define CHECK(x) do{assertions++;if(!(x))return __LINE__;}while(0)
'''
 harness=prefix+stubs+newbody+'\n'+rename(oldbody)
 harness+='\nuint32_t equal(void){\n'
 for n in active:harness+=f'CHECK(memcmp((void*)&{n},(void*)&old_{n},sizeof({n}))==0);\n'
 harness+='return 0U;}\nvoid sync(void){\n'
 for n in active:harness+=f'memcpy((void*)&old_{n},(void*)&{n},sizeof({n}));\n'
 harness+='}\n'
 harness+=r'''
uint32_t saved_trace[512],saved_trace_count,saved_sent_used;char saved_sent[2048];
ChassisHeadingRequest before_bus,after_bus;
void clear_trace(void){trace_count=depth=sent_used=0U;memset(sent,0,sizeof(sent));}
void save(void){saved_trace_count=trace_count;memcpy(saved_trace,trace,sizeof(trace));saved_sent_used=sent_used;memcpy(saved_sent,sent,sizeof(sent));after_bus=bus;}
uint32_t compare(void){CHECK(depth==0U);CHECK(saved_trace_count==trace_count);CHECK(memcmp(saved_trace,trace,trace_count*4U)==0);CHECK(saved_sent_used==sent_used);CHECK(memcmp(saved_sent,sent,sent_used)==0);CHECK(memcmp(&after_bus,&bus,sizeof(bus))==0);CHECK(equal()==0U);cases++;return 0U;}
#if TEST_DEBUG_FIXTURE
const char *input;uint32_t input_length,input_gap,input_split,operation;
void action(uint32_t old){uint32_t i;
 if(operation==0U){for(i=0;i<input_length;i++){if(i==input_split)clock_ms+=input_gap;if(old)old_App_UartLinkByte((uint8_t)input[i]);else App_UartLinkByte((uint8_t)input[i]);}}
 else if(operation==1U){if(old)old_App_UartLinkPoll();else App_UartLinkPoll();}
 else if(operation==2U){if(old)old_App_HeadingLiveStep();else App_HeadingLiveStep();}
 else if(operation==3U){if(old)old_AppDebug_HeadingObserve();else AppDebug_HeadingObserve();}
}
uint32_t pair(uint32_t op){uint32_t now=clock_ms;operation=op;before_bus=bus;sync();clear_trace();action(0U);save();bus=before_bus;clock_ms=now;clear_trace();action(1U);CHECK(compare()==0U);return 0U;}
void uart_reset(void){
'''
 for n in active:
  if n.startswith(('g_uart_link_','s_link_')):harness+=f'memset((void*)&{n},0,sizeof({n}));\n'
 harness+=r'''
 g_uart_link_test_value=41U;clock_ms=1000U;input_gap=0U;input_split=0xFFFFFFFFU;send_ok=1U;
}
uint32_t command(const char *s,const char *expected,uint32_t errors,uint32_t value_out){
 uart_reset();input=s;input_length=strlen(s);CHECK(pair(0U)==0U);CHECK(g_uart_link_errors==errors);CHECK(g_uart_link_test_value==value_out);
 CHECK(s_link_pending==(expected!=0));CHECK(pair(1U)==0U);if(expected)CHECK(!strcmp(sent,expected));else CHECK(!strncmp(sent,"LINK,HB,",8U));uart_cases++;return 0U;
}
void heading_reset(void){
'''
 for n in active:
  if n.startswith(('g_heading_live_','s_heading_live_','g_heading_bus_','g_heading_publish_')):harness+=f'memset((void*)&{n},0,sizeof({n}));\n'
 harness+=r'''
 clock_ms=1000U;g_vm_test_request=g_rtos_fault=g_safety_fault=0U;
 g_hwt_version_result=1U;value.valid_mask=3U;g_hwt_angle_age_ms=g_hwt_rate_age_ms=0U;g_hwt_update_tick=clock_ms;
 g_hwt_cal_active=g_hwt_cal_request=g_hwt_cal_end_tx_result=g_hwt_version_state=g_hwt_version_request=g_hwt_a7_request=0U;
 g_hwt_yaw_deg=g_hwt_wz_dps=0.0f;g_heading_live_relative_deg=45.0f;bus=(ChassisHeadingRequest){0,1U,0U};bus_valid=1U;
}
uint32_t heading_step(void){CHECK(pair(2U)==0U);CHECK(g_vm_test_request==0U);heading_cases++;return 0U;}
#endif
__attribute__((section("i.test_entry"))) uint32_t test_entry(void){uint32_t n;const int32_t ccw[5]={5,3,1,0,0},cw[5]={-5,-3,-1,0,0};
 CHECK(fixture_config==&g_heading_controller_default_config);CHECK(fixture_config->coarse_rpm==5U);
 CHECK(send_ok==1U&&bus_valid==1U&&bus.stop_required==1U);CHECK(assertions>0U&&cases==0U&&send_attempts==0U&&publish_calls==0U);
 CHECK(g_heading_dry_result==0U&&old_g_heading_dry_result==0U);CHECK(equal()==0U);
 for(n=0U;n<2U;n++){
  clear_trace();before_bus=bus;App_HeadingDryRun();save();bus=before_bus;clear_trace();old_App_HeadingDryRun();CHECK(compare()==0U);
  CHECK(g_heading_dry_result==1U&&g_heading_dry_case==0U);CHECK(memcmp((void*)g_heading_dry_ccw,ccw,sizeof(ccw))==0);CHECK(memcmp((void*)g_heading_dry_cw,cw,sizeof(cw))==0);CHECK(g_heading_dry_cross_error==2.0f);
 }
#if TEST_DEBUG_FIXTURE
 CHECK(g_heading_bus_stop==1U&&old_g_heading_bus_stop==1U);CHECK(s_heading_live_used==0U&&s_link_pending==0U);
 CHECK(command("PING,x\n","PONG,x\r\n",0U,41U)==0U);
 CHECK(command("PING,12345678901234567890123456789012\n","PONG,12345678901234567890123456789012\r\n",0U,41U)==0U);
 CHECK(command("PING,\n","ERR,COMMAND\r\n",1U,41U)==0U);
 CHECK(command("PING,123456789012345678901234567890123\n","ERR,COMMAND\r\n",1U,41U)==0U);
 CHECK(command("ECHO, a  b \n","ECHO, a  b \r\n",0U,41U)==0U);
 CHECK(command("ECHO,\n","ECHO,\r\n",0U,41U)==0U);
 CHECK(command("ECHO,123456789012345678901234567890123456789012\n","ECHO,123456789012345678901234567890123456789012\r\n",0U,41U)==0U);
 CHECK(command("ECHO,1234567890123456789012345678901234567890123\n",0,1U,41U)==0U);
 CHECK(command("INFO\r\n","INFO,UART1,115200,8N1,VM0\r\n",0U,41U)==0U);
 CHECK(command("GET\n","VALUE,41\r\n",0U,41U)==0U);
 CHECK(command("SET,0\n","VALUE,0\r\n",0U,0U)==0U);
 CHECK(command("SET,1000\n","VALUE,1000\r\n",0U,1000U)==0U);
 CHECK(command("SET,0001\n","VALUE,1\r\n",0U,1U)==0U);
 CHECK(command("SET,1001\n","ERR,RANGE\r\n",1U,41U)==0U);
 CHECK(command("SET,9999999999999999999999\n","ERR,RANGE\r\n",1U,41U)==0U);
 CHECK(command("SET,\n","ERR,RANGE\r\n",1U,41U)==0U);
 CHECK(command("SET,-1\n","ERR,RANGE\r\n",1U,41U)==0U);
 CHECK(command("SET,+1\n","ERR,RANGE\r\n",1U,41U)==0U);
 CHECK(command("SET,1x\n","ERR,RANGE\r\n",1U,41U)==0U);
 CHECK(command("SET, 1\n","ERR,RANGE\r\n",1U,41U)==0U);
 CHECK(command("GET \n","ERR,COMMAND\r\n",1U,41U)==0U);
 CHECK(command("get\n","ERR,COMMAND\r\n",1U,41U)==0U);
 CHECK(command("G\x1f" "ET\n",0,1U,41U)==0U);
 CHECK(command("G\x7f" "ET\n",0,1U,41U)==0U);
 CHECK(command("G\x80" "ET\n",0,1U,41U)==0U);
 CHECK(command("\r\n",0,0U,41U)==0U);
 /* 499ms retains the partial command; 500ms clears it. Include tick wrap. */
 for(n=0U;n<4U;n++){
  uart_reset();clock_ms=n>=2U?0xFFFFFF00U:1000U;input="PING,x\n";input_length=7U;input_split=1U;input_gap=(n%2U)?500U:499U;
  CHECK(pair(0U)==0U);CHECK(s_link_pending==1U);CHECK(g_uart_link_errors==n%2U);CHECK(pair(1U)==0U);CHECK(!strcmp(sent,n%2U?"ERR,COMMAND\r\n":"PONG,x\r\n"));uart_cases++;
 }
 /* Overflow recovers at newline or timeout, and CR is ignored. */
 uart_reset();input="BAD\tX\nGET\n";input_length=strlen(input);CHECK(pair(0U)==0U);CHECK(g_uart_link_errors==1U&&g_uart_link_rx_lines==1U);CHECK(pair(1U)==0U);CHECK(!strcmp(sent,"VALUE,41\r\n"));uart_cases++;
 uart_reset();input="BAD\tGET\n";input_length=strlen(input);input_split=4U;input_gap=500U;CHECK(pair(0U)==0U);CHECK(g_uart_link_errors==1U&&g_uart_link_rx_lines==1U);CHECK(pair(1U)==0U);CHECK(!strcmp(sent,"VALUE,41\r\n"));uart_cases++;
 /* Reply > pending YAW > due heartbeat. Failed sends retain original state. */
 uart_reset();input="PING,x\nSET,5\n";input_length=strlen(input);CHECK(pair(0U)==0U);CHECK(g_uart_link_rx_lines==2U&&g_uart_link_dropped==1U&&g_uart_link_test_value==41U);
 s_link_yaw_pending=1U;g_hwt_yaw_deg=-12.345f;g_hwt_valid_mask=3U;g_hwt_update_tick=clock_ms-12U;
 send_ok=0U;CHECK(pair(1U)==0U);CHECK(!strcmp(sent,"PONG,x\r\n"));CHECK(s_link_pending==1U&&s_link_yaw_pending==1U&&g_uart_link_tx_replies==0U);priority_cases++;
 send_ok=1U;CHECK(pair(1U)==0U);CHECK(!strcmp(sent,"PONG,x\r\n"));CHECK(s_link_pending==0U&&s_link_yaw_pending==1U&&g_uart_link_tx_replies==1U);priority_cases++;
 send_ok=0U;CHECK(pair(1U)==0U);CHECK(!strcmp(sent,"LINK,YAW,-12345,3,12\r\n"));CHECK(s_link_yaw_pending==1U&&g_uart_link_hb_count==0U);priority_cases++;
 send_ok=1U;CHECK(pair(1U)==0U);CHECK(!strcmp(sent,"LINK,YAW,-12345,3,12\r\n"));CHECK(s_link_yaw_pending==0U&&g_uart_link_hb_count==0U);priority_cases++;
 send_ok=0U;CHECK(pair(1U)==0U);CHECK(!strcmp(sent,"LINK,HB,0,1000,VM0\r\n"));CHECK(s_link_hb_tick==0U&&s_link_yaw_pending==0U&&g_uart_link_hb_count==0U);priority_cases++;
 send_ok=1U;CHECK(pair(1U)==0U);CHECK(!strcmp(sent,"LINK,HB,0,1000,VM0\r\n"));CHECK(s_link_hb_tick==1000U&&s_link_yaw_pending==1U&&g_uart_link_hb_count==1U);priority_cases++;
 CHECK(pair(1U)==0U);CHECK(!strncmp(sent,"LINK,YAW,",9U));CHECK(pair(1U)==0U);CHECK(sent_used==0U);priority_cases+=2U;
 clock_ms=1999U;CHECK(pair(1U)==0U);CHECK(sent_used==0U);clock_ms=2000U;CHECK(pair(1U)==0U);CHECK(!strcmp(sent,"LINK,HB,1,2000,VM0\r\n"));priority_cases+=2U;
 /* Out-of-range, NaN and infinity remain sentinel YAW values. */
 for(n=0U;n<5U;n++){
  union {uint32_t u;float f;} number;number.u=n==0U?0x7fc00000U:0x7f800000U;g_hwt_yaw_deg=n<2U?number.f:(n==2U?181.0f:(n==3U?-180.0f:180.0f));
  s_link_yaw_pending=1U;CHECK(pair(1U)==0U);CHECK(!strncmp(sent,n<3U?"LINK,YAW,-2147483648,":(n==3U?"LINK,YAW,-180000,":"LINK,YAW,180000,"),n<3U?21U:16U));priority_cases++;
 }
 /* Every health predicate independently rejects a new preview request. */
 for(n=0U;n<12U;n++){
  heading_reset();g_heading_live_request=1U;
  switch(n){case 0:value.valid_mask=1U;break;case 1:g_hwt_version_result=0U;break;case 2:g_hwt_angle_age_ms=501U;break;case 3:g_hwt_rate_age_ms=501U;break;case 4:g_hwt_update_tick=899U;break;case 5:g_hwt_cal_active=1U;break;case 6:g_hwt_cal_request=1U;break;case 7:g_hwt_cal_end_tx_result=2U;break;case 8:g_hwt_version_state=1U;break;case 9:g_hwt_version_request=1U;break;case 10:g_hwt_a7_request=1U;break;case 11:g_hwt_update_tick=clock_ms+1U;break;}
  CHECK(heading_step()==0U);CHECK(g_heading_live_reject==3U&&g_heading_live_request==0U&&s_heading_live_used==0U);
 }
 for(n=0U;n<3U;n++){
  heading_reset();g_heading_live_request=1U;if(n==0U)g_vm_test_request=1U;else if(n==1U)g_rtos_fault=1U;else g_safety_fault=1U;
  CHECK(pair(2U)==0U);CHECK(g_heading_live_reject==2U&&s_heading_live_used==0U);CHECK(g_vm_test_request==(n==0U?1U:0U));heading_cases++;
 }
 for(n=0U;n<4U;n++){
  union {uint32_t u;float f;} number;number.u=n==0U?0x7fc00000U:0x7f800000U;
  heading_reset();g_heading_live_request=1U;g_heading_live_relative_deg=n<2U?number.f:(n==2U?90.001f:-90.001f);
  CHECK(heading_step()==0U);CHECK(g_heading_live_reject==4U&&s_heading_live_used==0U);
 }
 /* Accepted preview, exact age boundaries, cancel/one-shot, observer bus copy. */
 heading_reset();g_hwt_angle_age_ms=g_hwt_rate_age_ms=500U;g_hwt_update_tick=900U;g_heading_live_request=1U;
 CHECK(heading_step()==0U);CHECK(g_heading_live_reject==0U&&g_heading_live_state==HEADING_CTRL_TURNING&&g_heading_live_command_rpm==5&&s_heading_live_used==1U);
 CHECK(pair(3U)==0U);CHECK(g_heading_bus_valid==1U&&g_heading_bus_rpm==5&&g_heading_bus_stop==0U&&g_heading_bus_sequence==bus.sequence);heading_cases++;
 g_heading_live_request=2U;CHECK(heading_step()==0U);CHECK(g_heading_live_state==HEADING_CTRL_CANCELLED&&g_heading_live_command_rpm==0);
 g_heading_live_request=1U;CHECK(heading_step()==0U);CHECK(g_heading_live_reject==1U&&g_heading_live_request==0U);
 /* Failed request is consumed, never deferred to later good health. */
 heading_reset();g_heading_live_request=1U;g_hwt_cal_active=1U;CHECK(heading_step()==0U);g_hwt_cal_active=0U;CHECK(heading_step()==0U);CHECK(g_heading_live_state==HEADING_CTRL_IDLE&&s_heading_live_used==0U);
 /* Active preview loses health and locks fault without hardware execution. */
 heading_reset();g_heading_live_request=1U;CHECK(heading_step()==0U);g_hwt_rate_age_ms=501U;CHECK(heading_step()==0U);CHECK(g_heading_live_state==HEADING_CTRL_FAULT&&g_heading_live_cause==HEADING_CAUSE_SENSOR_INVALID&&g_heading_live_stop_required==1U);
 /* Both signs, wrap near +/-180, natural settle and timeout. */
 for(n=0U;n<4U;n++){
  heading_reset();g_hwt_yaw_deg=n<2U?0.0f:179.0f;g_heading_live_relative_deg=n%2U?-45.0f:45.0f;g_heading_live_request=1U;CHECK(heading_step()==0U);
  CHECK(g_heading_live_command_rpm==(n%2U?-5:5));clock_ms+=100U;g_hwt_update_tick=clock_ms;g_hwt_yaw_deg=g_heading_live_target_deg;CHECK(heading_step()==0U);CHECK(g_heading_live_state==HEADING_CTRL_SETTLING);
  clock_ms+=700U;g_hwt_update_tick=clock_ms;CHECK(heading_step()==0U);CHECK(g_heading_live_state==HEADING_CTRL_DONE&&g_heading_live_stop_required==1U);
 }
 heading_reset();g_heading_live_request=1U;CHECK(heading_step()==0U);clock_ms+=20000U;g_hwt_update_tick=clock_ms;CHECK(heading_step()==0U);CHECK(g_heading_live_cause==HEADING_CAUSE_TIMEOUT);
 bus_valid=0U;bus.command_rpm=-3;bus.stop_required=1U;CHECK(pair(3U)==0U);CHECK(g_heading_bus_valid==0U&&g_heading_bus_rpm==-3&&g_heading_bus_stop==1U);heading_cases++;
 CHECK(send_attempts>0U&&send_failures>0U&&publish_calls>0U&&observe_calls>0U&&read_calls>0U);
#else
 CHECK(uart_cases==0U&&heading_cases==0U&&priority_cases==0U&&send_attempts==0U&&send_failures==0U&&publish_calls==0U&&observe_calls==0U&&read_calls==0U);
#endif
 return 0U;
}
'''
 out=c.T/('unit_'+label);out.mkdir(exist_ok=True);(out/'unit.c').write_bytes(harness.encode('gbk'))
 (out/'unit.sct').write_text('LR_ROM 0x08000000 0x100000 { ER_ROM 0x08000000 0x100000 { unit.o (i.test_entry, +First) *(+RO) } RW_RAM 0x20000000 0x20000 { *(+RW,+ZI) } }\n')
 simple=[x for x in opts if x not in ['--cpu','Cortex-M4.fp.sp']]
 commands=[]
 for src,obj in [(out/'unit.c','unit.o'),(c.U/'heading_controller.c','heading.o'),(c.U/'chassis_kinematics.c','kinematics.o')]:
  commands.append([str(c.B/'armcc.exe'),'--cpu','Cortex-M3','--gnu']+simple+['--split_sections','-O1','-c',c.short(src),'-o',obj])
 commands.append([str(c.B/'armlink.exe'),'--entry','test_entry','--scatter','unit.sct','unit.o','heading.o','kinematics.o','--library_type=microlib','-o','unit.axf'])
 for cmd in commands:
  p=subprocess.run(cmd,cwd=out,capture_output=True,timeout=60);assert p.returncode==0,(p.stdout+p.stderr).decode(errors='replace')
 (out/'commands.json').write_text(json.dumps(commands,indent=2),'utf8')
 sys.path.insert(0,str(c.R/'工创赛/tmp/position_alignment_deps'));import unicorn as uc
 from unicorn import arm_const as reg
 blob=(out/'unit.axf').read_bytes();elf=struct.unpack_from('<16sHHIIIIIHHHHHH',blob)
 m=uc.Uc(uc.UC_ARCH_ARM,uc.UC_MODE_THUMB|uc.UC_MODE_MCLASS);m.mem_map(0x08000000,0x100000);m.mem_map(0x20000000,0x20000);m.mem_write(0x20000000,b'\xa5'*0x20000)
 initialized_rw=zeroed=nonzero_rw=0;loaded=[]
 for i in range(elf[12]):
  _,kind,flags,addr,off,size,_,_,_,_=struct.unpack_from('<IIIIIIIIII',blob,elf[6]+i*elf[11])
  if flags&2 and size:
   if kind==1:
    data=blob[off:off+size];assert len(data)==size;m.mem_write(addr,data)
    if flags&1:initialized_rw+=size;nonzero_rw+=sum(bool(b) for b in data)
   elif kind==8:m.mem_write(addr,b'\x00'*size);zeroed+=size
   else:continue
   loaded.append(dict(type='PROGBITS' if kind==1 else 'NOBITS',execution_address=hex(addr),size=size,writable=bool(flags&1)))
 symbols=subprocess.check_output([str(c.B/'fromelf.exe'),'--text','-s','unit.axf'],cwd=out).decode(errors='replace');(out/'symbols.txt').write_text(symbols,'utf8')
 def address(n):return int(re.search(r'\b'+n+r'\s+(0x[0-9a-fA-F]+)',symbols)[1],16)
 def value(n):return struct.unpack('<I',m.mem_read(address(n),4))[0]
 initial=dict(send_ok=value('send_ok'),bus_valid=value('bus_valid'),cases=value('cases'),assertions=value('assertions'),dry=value('g_heading_dry_result'),old_dry=value('old_g_heading_dry_result'),config_pointer=value('fixture_config'),config_address=address('g_heading_controller_default_config'))
 assert initial['send_ok']==initial['bus_valid']==1 and initial['cases']==initial['assertions']==initial['dry']==initial['old_dry']==0
 assert initial['config_pointer']==initial['config_address']
 if fixture:
  initial.update(bus_stop=value('g_heading_bus_stop'),old_bus_stop=value('old_g_heading_bus_stop'),used=value('s_heading_live_used'),pending=value('s_link_pending'))
  assert initial['bus_stop']==initial['old_bus_stop']==1 and initial['used']==initial['pending']==0
 stop=0x080FF000;m.reg_write(reg.UC_ARM_REG_SP,0x2001FFF0);m.reg_write(reg.UC_ARM_REG_LR,stop|1);m.emu_start(elf[4]|1,stop,timeout=60000000,count=100000000)
 assert m.reg_read(reg.UC_ARM_REG_PC)==stop,'Emulator not complete'
 result=m.reg_read(reg.UC_ARM_REG_R0)
 if result:
  print('ARM failure',label,'line',result,harness.splitlines()[result-1],flush=True)
  print({n:value(n) for n in ['cases','assertions','uart_cases','heading_cases','priority_cases']},flush=True)
 assert result==0,f'C assertion line {result}'
 counts={n:value(n) for n in ['cases','assertions','uart_cases','heading_cases','priority_cases','send_attempts','send_failures','publish_calls','observe_calls','read_calls']}
 report=dict(**counts,initialized_rw_bytes=initialized_rw,nonzero_initialized_bytes=nonzero_rw,zeroed_nobits_bytes=zeroed,loader='SHF_ALLOC PROGBITS at execution address; explicitly zero NOBITS after poisoning RAM',initial_values=initial,loaded_sections=loaded,artifact_sha256={str(p.relative_to(c.R)).replace('\\','/'):c.sha(p) for p in sorted(out.iterdir()) if p.is_file() and p.name!='result.json'},controller_sha256=c.sha(c.U/'heading_controller.c'),kinematics_sha256=c.sha(c.U/'chassis_kinematics.c'))
 (out/'result.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),'utf8');return report
