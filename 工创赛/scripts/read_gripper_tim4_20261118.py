"""Read actual a5e gripper TIM4_CH4/PB9 and ELF globals; no output writes."""
from datetime import datetime
import json,sys
import stage3_resume_status_20261118 as status

def main():
    path=status.OUT/('gripper-tim4-readonly-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.json')
    report={'read_only':True,'motion_requests':0,'ram_writes':0,'actual_pin':'PB9/TIM4_CH4',
            'note':'Earlier TIM2 diagnostic is not applicable to current a5e hardware mapping'}
    code=0
    try:
        axf,image,audit=status.local_audit();assert audit['source_manifest_matches']
        import gripper210_attach_read_20261105 as live
        from source_body_probe_20261111 import attach
        from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard
        from elftools.elf.elffile import ELFFile
        with axf.open('rb') as stream:
            elf=ELFFile(stream);objects={s.name:int(s['st_value']) for s in elf.get_section_by_name('.symtab').iter_symbols() if s['st_info']['type']=='STT_OBJECT'}
        live.IMAGE=axf;symbols,segments=live.load_image()
        with attach() as session:
            t=status.ReadOnlyTarget(session.target);board=CommittedPickBoard(t,symbols,segments)
            report['verified_flash_bytes']=board.verified_flash_bytes
            regs={'TIM4_CR1':0x40000800,'TIM4_CCMR2':0x4000081c,'TIM4_CCER':0x40000820,
                  'TIM4_CNT':0x40000824,'TIM4_PSC':0x40000828,'TIM4_ARR':0x4000082c,
                  'TIM4_CCR4':0x40000840,'GPIOB_MODER':0x40020400,'GPIOB_AFRH':0x40020424,
                  'GPIOB_ODR':0x40020414,'RCC_APB1ENR':0x40023840}
            report['registers']={n:t.read32(a) for n,a in regs.items()}
            report['second_counter']=t.read32(0x40000824)
            names=('g_gripper_signal_stop_count','g_jiazhua_debug_request','g_jiazhua_debug_result',
                   'g_gripper_task_create','g_it_action','g_it_state','g_it_rejected')
            report['globals']={n:t.read32(objects[n]) if n in objects else 'not present in ELF' for n in names}
    except Exception as e:report['error']=repr(e);code=1
    finally:
        with path.open('x',encoding='utf-8') as stream:json.dump(report,stream,indent=2)
        print(json.dumps({'evidence':str(path),'report':report}))
    return code

if __name__=='__main__':raise SystemExit(main())
