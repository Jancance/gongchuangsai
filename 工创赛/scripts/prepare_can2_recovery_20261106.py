"""只修改CAN1过滤器保留及DM通信初始化/恢复；不改运动、VM或原点逻辑。"""
from pathlib import Path
import shutil,json,hashlib
R=Path(__file__).resolve().parents[1]
B=R/'tmp/integration10_complete_20261106/project'
S=R/'tmp/integration10_can2_recovery_20261106/project'
O=R/'evidence/integration/2026-11-06/can2-recovery'

def main():
    assert json.loads((O/'matlab-result.json').read_text())['passed']
    assert not S.exists(),'Preserve existing candidate'
    shutil.copytree(B,S,ignore=shutil.ignore_patterns('Objects','ObjectsOffline','Listings','*.uvguix.*'))
    changes={}
    def edit(name,fn):
        p=S/name;raw=p.read_bytes()
        try:t=raw.decode('utf-8');enc='utf-8'
        except UnicodeDecodeError:t=raw.decode('gbk');enc='gbk'
        nl='\r\n' if b'\r\n' in raw else '\n'
        p.write_bytes(fn(t.replace('\r\n','\n')).replace('\n',nl).encode(enc))
        changes[name]={'encoding':enc,'newline':repr(nl),'before':hashlib.sha256(raw).hexdigest(),'after':hashlib.sha256(p.read_bytes()).hexdigest()}
    def sub(t,a,b):assert t.count(a)==1,a;return t.replace(a,b)
    def can(t):
        mark='static uint32_t CAN1_InitMode(uint8_t mode)'
        helpers='''/* CAN1/CAN2共享过滤器。仅保存/恢复原14..27号CAN2资源，
 * 所有初始化仍由唯一CAN任务串行调用；不改CAN1刚重建的过滤器0。 */
typedef struct {
 uint32_t valid,fmr,fm1r,fs1r,ffa1r,fa1r,fr[14][2];
} CAN2FilterBackup;
static void CAN2_FiltersSave(CAN2FilterBackup *b)
{
 uint32_t i;memset(b,0,sizeof(*b));
 if(!(RCC->APB1ENR&RCC_APB1Periph_CAN2) || ((CAN1->FMR>>8)&63U)!=14U)return;
 b->valid=1U;b->fmr=CAN1->FMR;b->fm1r=CAN1->FM1R;b->fs1r=CAN1->FS1R;
 b->ffa1r=CAN1->FFA1R;b->fa1r=CAN1->FA1R;
 for(i=0;i<14U;i++){b->fr[i][0]=CAN1->sFilterRegister[i+14U].FR1;b->fr[i][1]=CAN1->sFilterRegister[i+14U].FR2;}
}
static void CAN2_FiltersRestore(const CAN2FilterBackup *b)
{
 const uint32_t mask=0x0fffc000U;uint32_t i;
 if(!b->valid)return;
 CAN1->FMR|=1U;
 CAN1->FMR=(CAN1->FMR&~0x3f00U)|(b->fmr&0x3f00U)|1U;
 CAN1->FA1R&=~mask;
 for(i=0;i<14U;i++){CAN1->sFilterRegister[i+14U].FR1=b->fr[i][0];CAN1->sFilterRegister[i+14U].FR2=b->fr[i][1];}
 CAN1->FM1R=(CAN1->FM1R&~mask)|(b->fm1r&mask);
 CAN1->FS1R=(CAN1->FS1R&~mask)|(b->fs1r&mask);
 CAN1->FFA1R=(CAN1->FFA1R&~mask)|(b->ffa1r&mask);
 CAN1->FA1R=(CAN1->FA1R&~mask)|(b->fa1r&mask);
 CAN1->FMR=(CAN1->FMR&~1U)|(b->fmr&1U);
}
'''
        t=sub(t,mark,helpers+mark)
        t=sub(t,'    TickType_t init_start_tick;','    TickType_t init_start_tick;\n    CAN2FilterBackup can2_filters;')
        t=sub(t,'    CAN_DeInit(CAN1);','    CAN2_FiltersSave(&can2_filters);\n    CAN_DeInit(CAN1);\n    CAN2_FiltersRestore(&can2_filters);')
        # CAN_FilterInit只修改自己一组，但再核一次恢复结果可保留所有CAN2资源。
        return t
    edit('CHASSIS/SYSTEM/CAN/can.c',can)
    def dm(t):
        marker='static uint32_t DM_CAN2_IsBusy(void)'
        helpers='''volatile uint32_t g_dm_can2_lec,g_dm_can2_tec,g_dm_can2_rec,g_dm_can2_bus_off;
volatile uint32_t g_dm_can2_first_error_valid,g_dm_can2_first_error_esr,g_dm_can2_first_error_tick;
volatile uint32_t g_dm_can2_filter_repairs,g_dm_can2_hardware_initializations;
static uint32_t s_dm_hardware_ready;
static void DM_CAN2_RecordErrors(void)
{
 uint32_t esr=CAN2->ESR,lec=(esr>>4)&7U;
 g_dm_can2_esr=esr;g_dm_can2_lec=lec;g_dm_can2_tec=(esr>>16)&255U;
 g_dm_can2_rec=(esr>>24)&255U;g_dm_can2_bus_off=(esr>>2)&1U;
 if(!g_dm_can2_first_error_valid && ((lec>=1U && lec<=6U) || (esr&7U))){
  g_dm_can2_first_error_esr=esr;g_dm_can2_first_error_tick=(uint32_t)xTaskGetTickCount();
  g_dm_can2_first_error_valid=1U;
 }
}
static uint32_t DM_CAN2_EnsureFilter(void)
{
 CAN_FilterInitTypeDef filter;const uint32_t bit=1U<<14;
 g_dm_can2_filter_start=(CAN1->FMR>>8)&63U;
 if(g_dm_can2_filter_start!=14U)return 0U; /* 不擅自夺取其他分配。 */
 if(!(CAN1->FMR&1U) && (CAN1->FA1R&bit) && (CAN1->FS1R&bit) &&
    !(CAN1->FM1R&bit) && !(CAN1->FFA1R&bit) &&
    CAN1->sFilterRegister[14].FR1==0U && CAN1->sFilterRegister[14].FR2==0xffe00006U)return 1U;
 memset(&filter,0,sizeof(filter));filter.CAN_FilterNumber=14U;
 filter.CAN_FilterMode=CAN_FilterMode_IdMask;filter.CAN_FilterScale=CAN_FilterScale_32bit;
 filter.CAN_FilterMaskIdHigh=0xffe0U;filter.CAN_FilterMaskIdLow=6U;
 filter.CAN_FilterFIFOAssignment=CAN_FIFO0;filter.CAN_FilterActivation=ENABLE;
 CAN_FilterInit(&filter);g_dm_can2_filter_repairs++;
 return !(CAN1->FMR&1U) && (CAN1->FA1R&bit) && (CAN1->FS1R&bit) &&
    !(CAN1->FM1R&bit) && !(CAN1->FFA1R&bit) &&
    CAN1->sFilterRegister[14].FR1==0U && CAN1->sFilterRegister[14].FR2==0xffe00006U;
}
'''
        t=sub(t,marker,helpers+marker)
        t=t.replace('    g_dm_can2_esr = CAN2->ESR;','    DM_CAN2_RecordErrors();')
        t=sub(t,'    CAN_FilterInitTypeDef filter;','')
        t=sub(t,'    CAN_DeInit(CAN2);','''    if(s_dm_hardware_ready && CAN2->BTR==0x001a0002U &&
       !(CAN2->MCR&CAN_MCR_INRQ) && !(CAN2->MSR&CAN_MSR_INAK) &&
       !(CAN2->MCR&CAN_MCR_NART) && (CAN2->MCR&CAN_MCR_ABOM)){
        if(!DM_CAN2_EnsureFilter())return 9U;
        return 1U;
    }
    /* 配置一次；普通读参数不得复位正在工作的CAN2。 */
    s_dm_hardware_ready=0U;
    CAN_DeInit(CAN2);''')
        t=sub(t,'    can.CAN_NART = ENABLE;','    can.CAN_NART = DISABLE;\n    can.CAN_ABOM = ENABLE; /* 原30ms截止/取消邮箱仍生效，不恢复旧运动。 */')
        start=t.index('    memset(&filter, 0, sizeof(filter));',t.index('static uint32_t DM_CAN2_Init'))
        end=t.index('    g_dm_can2_can1_btr_after = CAN1->BTR;',start)
        t=t[:start]+'''    if(!DM_CAN2_EnsureFilter())return 9U;
    s_dm_hardware_ready=1U;g_dm_can2_hardware_initializations++;
'''+t[end:]
        t=sub(t,'    if (((CAN1->FMR >> 8) & 0x3FU) != 14U || (CAN1->FA1R & (1UL << 14)) == 0U)',
                   '    if (!DM_CAN2_EnsureFilter())')
        return t
    edit('HARDWARE/DM_CAN2/dm_can2.c',dm)
    (O/'staged.json').write_text(json.dumps({'base':str(B),'candidate':str(S),'changes':changes,'firmware_downloaded':False,'motion_logic_changed':False},indent=2),encoding='utf-8')
    print('Two communication source files patched; not flashed.')

if __name__=='__main__':main()
