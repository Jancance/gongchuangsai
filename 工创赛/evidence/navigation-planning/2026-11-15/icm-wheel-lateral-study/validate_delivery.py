"""Bind final local evidence and diagnose the two assert-line metadata bytes."""
from pathlib import Path
import hashlib,json,struct,subprocess
R=Path.cwd();E=Path(__file__).resolve().parent;U=R/'3.stm32f407工程/CHASSIS/USER'
OLD=R/'工创赛/evidence/navigation-planning/2026-11-15/rollback-half-ff-20261115/stop-distance-observer'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def sections(path):
    b=path.read_bytes();h=struct.unpack_from('<16sHHIIIIIHHHHHH',b);out=[]
    for i in range(h[12]):
        _,kind,flags,addr,off,size,_,_,_,_=struct.unpack_from('<IIIIIIIIII',b,h[6]+i*h[11])
        if flags&2 and size:out.append((kind,flags,addr,size,b'\0'*size if kind==8 else b[off:off+size]))
    return out
old=json.loads((OLD/'build_summary.json').read_text('utf8'))
new=json.loads((E/'build_summary.json').read_text('utf8'))
arm=json.loads((E/'observer_arm_summary.json').read_text('utf8'))
pold=Path(old['builds']['baseline_off_integrated']['axf']);pnew=Path(new['builds']['baseline_off_integrated']['axf'])
aold=sections(pold);anew=sections(pnew);assert len(aold)==len(anew)
differences=[]
for index,(before,after) in enumerate(zip(aold,anew)):
    assert before[:4]==after[:4]
    differences.extend({'section':index,'offset':i,'address':before[2]+i,'before':a,'after':b}
        for i,(a,b) in enumerate(zip(before[4],after[4])) if a!=b)
assert [(d['section'],d['offset'],d['before'],d['after']) for d in differences]==[(0,6808,145,149),(0,6830,215,219)]
assert aold[1:]==anew[1:]
snippets={}
for label,path in [('before',pold),('after',pnew)]:
    text=subprocess.check_output(['D:/Keil_v5/ARM/arm5_compiler/Bin/fromelf.exe','--text','-c',str(path)]).decode('utf8',errors='replace')
    lines=text.splitlines();snippets[label]=[line for line in lines if any(x in line for x in ['0x08001a96:','0x08001a9a:','0x08001aac:','0x08001ab0:'])]
assert '#0x191' in snippets['before'][0] and '#0x195' in snippets['after'][0]
assert '#0x1d7' in snippets['before'][2] and '#0x1db' in snippets['after'][2]
assert all('App_RtosAssertFailed' in snippets[x][i] for x in snippets for i in [1,3])
before_lines=(E/'before/main.c').read_bytes().decode('gbk').splitlines()
after_lines=(U/'main.c').read_bytes().decode('gbk').splitlines()
assert all('configASSERT' in before_lines[n-1] for n in [401,471])
assert all('configASSERT' in after_lines[n-1] for n in [405,475])
for n,d in arm['source_sha256'].items():assert sha(U/n)==d
assert sha(R/'工创赛/scripts/vehicle_uart_capture.py')==arm['parser_sha256']
for n,d in new['source_sha256'].items():assert sha(U/n)==d
assert sha(U.parents[1]/'gongxun-f407.uvprojx')==new['project_sha256']
m=json.loads((E/'pre_implementation_manifest.json').read_text('utf8'))
changed=[p for p,d in m.items() if not (R/p).exists() or sha(R/p)!=d]
assert set(changed)=={'3.stm32f407工程\\gongxun-f407.uvprojx','3.stm32f407工程\\CHASSIS\\USER\\main.c','3.stm32f407工程\\CHASSIS\\USER\\app_serial_runtime.c'}
report={'status':'PASS_LOCAL_CANDIDATE_ONLY','pre_manifest_items':len(m),'changed_existing_manifest_paths':changed,
 'baseline_off_allocated_sizes_equal':True,'baseline_off_allocated_byte_identical':False,
 'baseline_off_only_assert_source_line_differences':differences,'disassembly':snippets,
 'assert_source_lines':{'before':[401,471],'after':[405,475]},
 'baseline_off_old_axf':str(pold),'baseline_off_new_axf':str(pnew),
 'baseline_off_old_axf_sha256':sha(pold),'baseline_off_new_axf_sha256':sha(pnew),
 'source_binding_to_arm_and_build':'PASS','actual_arm_cases':arm['cases'],'actual_arm_assertions':arm['assertions'],
 'icm_task_static_chain_bytes':592,'icm_task_allocated_stack_bytes':1024,'observer_step_static_frame_bytes':328,
 'io1_known_static_chain_bytes':376,'io1_stack_unknown_printf':True,
 'taskA_stack_watermark_not_measured':True,'hardware_accessed':False,'flashed':False,
 'candidate_axf':{k:v['axf_sha256'] for k,v in new['builds'].items()},
 'matlab_evidence_sha256':{p.name:sha(p) for pattern in ['observer_local_matlab*json','observer_schedule_matlab*json'] for p in E.glob(pattern)},
 'limits':'Static linker chain excludes interrupt/context overhead; printf has unknown library stack. RTOS execution timing, real spare stack, physical 200Hz continuity and ground benefit remain untested.'}
(E/'observer_validation_summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf8')
print(json.dumps({k:report[k] for k in ['status','pre_manifest_items','baseline_off_allocated_sizes_equal','baseline_off_allocated_byte_identical','source_binding_to_arm_and_build','actual_arm_cases','actual_arm_assertions']}))
