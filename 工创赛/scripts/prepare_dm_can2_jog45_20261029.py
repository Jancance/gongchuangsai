"""准备/安装用户指定的45度有界MIT试转；不下载，不发送硬件命令。"""
from pathlib import Path
import hashlib,json,re,shutil,subprocess,sys

ROOT=Path(__file__).resolve().parents[1]
ACTUAL=Path('D:/新建文件夹4/工创赛/代码/stm32/3.stm32f407工程')
STAGE=ROOT/'tmp/dm_can2_jog45_20261029/project-v2'
EV=ROOT/'evidence/p2-arm/2026-10-29/dm-can2-jog45'
NAMES=['dm_can2.c','dm_can2.h','dm_can2_protocol.c','dm_can2_motion.c']
FILES=['HARDWARE/DM_CAN2/'+n for n in NAMES]+['USER/main.c','gongxun-f407.uvprojx']
PROTECTED=['SYSTEM/CAN/can.c','USER/app_config.h','HARDWARE/LIFT/lift.c','HARDWARE/LIFT/lift_home.c',
           'HARDWARE/LIFT/lift_config.h','HARDWARE/EXTENSION/extension.c','HARDWARE/EXTENSION/extension.h']
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
def build(folder,log):
 r=subprocess.run(['C:/Keil_v5/UV4/UV4.exe','-b',str(folder/'gongxun-f407.uvprojx'),'-j0','-o',str(log)],capture_output=True,timeout=120)
 t=log.read_text(encoding='utf-8',errors='replace')
 assert r.returncode==0 and re.search(r'0 Error\(s\), 0 Warning\(s\)',t),t[-10000:]
 print(t[-650:],flush=True)
if '--install' in sys.argv:
 m=json.loads((EV/'manifest.json').read_text(encoding='utf-8'))
 assert m['host_pass'] and m['stage_build_pass']
 for f in m['files']:
  assert sha(ACTUAL/f['file'])==f['before'],'Actual source drift: '+f['file']
  assert sha(STAGE/f['file'])==f['after']
 for n,h in m['protected'].items():assert sha(ACTUAL/n)==h
 for f in m['files']:
  p=ACTUAL/f['file'];p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(STAGE/f['file'],p)
  assert sha(p)==f['after']
 m['installed']=True;m['actual_build_pass']=False
 (EV/'installed.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
 build(ACTUAL,EV/'actual-build.log')
 for n,h in m['protected'].items():assert sha(ACTUAL/n)==h
 m['actual_build_pass']=True
 (EV/'installed.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
 print('Installed 45-degree candidate; no flash or hardware commands.',flush=True)
 raise SystemExit(0)
refreshing = '--refresh' in sys.argv
if not refreshing: assert not STAGE.exists()
EV.mkdir(parents=True,exist_ok=True)
if not refreshing: shutil.copytree(ACTUAL,STAGE,ignore=shutil.ignore_patterns('Objects','Listings','.vscode','*.uvguix.*'))
for name in NAMES:shutil.copy2(ROOT/'output/dm_can2_jog45_20261029'/name,STAGE/'HARDWARE/DM_CAN2'/name)
if not refreshing:
 p=STAGE/'USER/main.c';data=p.read_bytes()
 assert data.count(b'            DM_CAN2_Service(')==1
 data=data.replace(b'            DM_CAN2_Service(',b'            DM_CAN2_ModuleService(',1)
 needle=b'            Lift_Service(((g_can_test_result == 1U) &&'
 assert data.count(needle)==1
 guard='''            /* 旋转试验期间阻止另一轴启动，保留原停止/退出回零请求。 */
             if (DM_CAN2_JogIsBusy())
             {
                 if (g_lift_motion_request != 3U) g_lift_motion_request = 0U;
                 if (g_lift_home_request != 2U) g_lift_home_request = 0U;
                 g_extension_read_request = 0U;
             }
 '''.replace('\n','\r\n').encode('gb18030')
 data=data.replace(needle,guard+needle,1);p.write_bytes(data)
 p=STAGE/'gongxun-f407.uvprojx';text=p.read_text(encoding='utf-8-sig')
 pattern=r'(<Group>\s*<GroupName>DM_CAN2</GroupName>\s*<Files>)(.*?)(</Files>\s*</Group>)'
 entry='<File><FileName>dm_can2_motion.c</FileName><FileType>1</FileType><FilePath>.\\HARDWARE\\DM_CAN2\\dm_can2_motion.c</FilePath></File>'
 text,count=re.subn(pattern,lambda m:m[1]+m[2]+entry+m[3],text,flags=re.S)
 assert count==1;p.write_text(text,encoding='utf-8')
host=ROOT/'tmp/dm_can2_jog45_20261029/host-check';host.mkdir(exist_ok=True)
old=ROOT/'tmp/dm_can2_pose_20261029/host-check'
header=(old/'stm32f4xx_can.h').read_text()
extra='''
typedef struct { uint32_t ESR; } CAN_TypeDef;
extern CAN_TypeDef mock_can2;
#define CAN2 (&mock_can2)
#define CAN_FIFO0 0
#define CAN_FLAG_FOV0 16
#define CAN_ESR_BOFF 4
#define CAN_TxStatus_NoMailBox 4
#define CAN_TxStatus_Ok 1
#define CAN_TxStatus_Failed 0
#define RESET 0
uint8_t CAN_Transmit(CAN_TypeDef*,CanTxMsg*);
uint8_t CAN_TransmitStatus(CAN_TypeDef*,uint8_t);
void CAN_CancelTransmit(CAN_TypeDef*,uint8_t);
uint8_t CAN_MessagePending(CAN_TypeDef*,uint8_t);
void CAN_Receive(CAN_TypeDef*,uint8_t,CanRxMsg*);
void CAN_ClearFlag(CAN_TypeDef*,uint32_t);
uint8_t CAN_GetFlagStatus(CAN_TypeDef*,uint32_t);
'''
(host/'stm32f4xx_can.h').write_text(header.replace('#endif',extra+'\n#endif'),encoding='utf-8')
(host/'FreeRTOS.h').write_text('#include <stdint.h>\ntypedef uint32_t TickType_t;\n#define pdMS_TO_TICKS(x) (x)\n#define portTICK_PERIOD_MS 1U\n',encoding='utf-8')
(host/'task.h').write_text('#include "FreeRTOS.h"\nTickType_t xTaskGetTickCount(void);\n',encoding='utf-8')
codec=(old/'test.c').read_text()
addition='''
 {
  unsigned j;float previous=-1.0f;
  const unsigned char expected[]={0x7F,0xFF,0x7F,0xF0,0x28,0x19,0x97,0xFF};
  assert(DM_CAN2_BuildMitHold(0,12.5f,30.0f,10.0f,&tx));
  assert(tx.StdId==1 && tx.IDE==0 && tx.RTR==0 && tx.DLC==8);
  assert(memcmp(tx.Data,expected,8)==0);
  assert(DM_CAN2_BuildMitPreload(-2.0f,12.5f,30.0f,10.0f,&tx));
  assert((tx.Data[3]&15)==0 && tx.Data[4]==0);
  assert(!DM_CAN2_BuildMitHold(13,12.5f,30.0f,10.0f,&tx));
  assert(!DM_CAN2_BuildMitHold(0,0,30.0f,10.0f,&tx));
  assert(DM_CAN2_BuildSwitch(0xFC,&tx)&&tx.Data[7]==0xFC);
  assert(DM_CAN2_BuildSwitch(0xFD,&tx)&&tx.Data[7]==0xFD);
  assert(!DM_CAN2_BuildSwitch(0xFE,&tx));assert(!DM_CAN2_BuildSwitch(0xFB,&tx));
  for(j=0;j<=100;j++){
   float v=DM_CAN2_SmoothStep((float)j/100.0f);
   assert(v>=previous && v>=0 && v<=1);previous=v;
  }
  assert(DM_CAN2_SmoothStep(0)==0&&DM_CAN2_SmoothStep(1)==1);
  assert(DM_CAN2_SmoothStep(0.5f)>0.499f&&DM_CAN2_SmoothStep(0.5f)<0.501f);
 }
'''
codec=codec.replace(' puts(',addition+' puts(',1)
(host/'codec.c').write_text(codec,encoding='utf-8')
shutil.copy2(ROOT/'scripts/dm_can2_jog45_host_model.c',host/'motion.c')
module=STAGE/'HARDWARE/DM_CAN2'
compiler='C:/mingw/mingw64/bin/gcc.exe'
common=[compiler,'-std=c99','-Wall','-Wextra','-Werror','-I'+str(host),'-I'+str(module)]
results=[]
for name,files in [('codec',['dm_can2_protocol.c']),('motion',['dm_can2_protocol.c','dm_can2_motion.c'])]:
 command=common+[str(module/n) for n in files]+[str(host/(name+'.c')),'-o',str(host/(name+'.exe'))]
 r=subprocess.run(command,capture_output=True,text=True);assert r.returncode==0,r.stdout+r.stderr
 cases=[''] if name=='codec' else ['positive','negative','stale','txfail','noreply','fault','disabled','tracking','limit','busy','abort','repeat','gap','badprofile','stoplost']
 for case in cases:
  args=[str(host/(name+'.exe'))]+([case] if case else [])
  r=subprocess.run(args,capture_output=True,text=True)
  results.append(r.stdout+r.stderr)
  assert r.returncode==0,(name,case,r.stdout,r.stderr)
(EV/'host-check.log').write_text('\n'.join(results),encoding='utf-8')
print('\n'.join(results),flush=True)
m={'files':[],'protected':{n:sha(ACTUAL/n) for n in PROTECTED},'host_pass':True,'stage_build_pass':False,
   'user_requested_degrees':45,'driver_mode_unchanged':'MIT','flash_performed':False,'motion_performed':False}
for name in FILES:
 source=ACTUAL/name
 if source.exists():
  backup=EV/'original'/name
  if backup.exists() and sha(backup)!=sha(source):backup=EV/'revision-before'/name
  backup.parent.mkdir(parents=True,exist_ok=True)
  if not backup.exists():shutil.copy2(source,backup)
 m['files'].append({'file':name,'before':sha(source),'after':sha(STAGE/name)})
(EV/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
build(STAGE,EV/'stage-build.log');m['stage_build_pass']=True
(EV/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2),encoding='utf-8')
print('Candidate staged; all checks passed; actual sources untouched.',flush=True)
