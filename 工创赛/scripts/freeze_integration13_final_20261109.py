"""Freeze the physically accepted local build and its exact source/evidence."""
from pathlib import Path
import hashlib,json,shutil

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-07/stage2-mainline'
P=R/'tmp/stage2_sub20_20261109/project'
OUT=R/'handoff/integration13_final_20261109'
manifest_path=E/'stage2-fast-candidate-20261109.json'
m=json.loads(manifest_path.read_text(encoding='utf-8'))
trace=E/'integration13-three-immediate-once-20261109-213859.json'
run=json.loads(trace.read_text(encoding='utf-8'))
assert run['complete'] and run['sha256']==m['images']['Vehicle_LiftRuler']['sha256']
assert run['three_to_release_command_s']==17.987 and run['three_to_zero_s']==19.185
assert not any(run['last'][k] for k in ('g_safety_fault','g_rtos_fault','g_it_cause','g_it_rejected',
 'g_source_parallel_cause','g_source_parallel_stop_mask','g_vehicle_arm_stop_pending','g_lift_ruler_stop_pending'))
for name,h in m['changed'].items():
 assert hashlib.sha256((P/'CHASSIS/USER'/name).read_bytes()).hexdigest()==h,name
assert not OUT.exists(),'Preserve existing frozen delivery'
shutil.copytree(P,OUT/'project',ignore=shutil.ignore_patterns('Objects','Listings','*.uvguix.*','*.bak'))
(OUT/'images').mkdir();(OUT/'verification').mkdir()
for target,im in m['images'].items():
 src=R/im['frozen_axf'];assert hashlib.sha256(src.read_bytes()).hexdigest()==im['sha256']
 shutil.copy2(src,OUT/'images'/(target+'.axf'))
 shutil.copy2(R/im['build_log'],OUT/'verification'/(target+'-build.log'))
for name in [trace.name,'integration13-zero-transient-flash-20261109.log',
 'integration13-empty-readonly-20261109-213759.json',
 'stage2-fast-port-check-20261109.log','integration13-immediate-host-20261109.log',
 'integration13-zero-transient-matlab-20261109.json',
 'integration13-right160-turn-overlap-matlab-20261109.json',
 'integration13-slot37-release200-matlab-20261109.json']:
 shutil.copy2(E/name,OUT/'verification'/name)
m['hardware_motion']=True;m['under20_demonstrated']=True
m['under20_scope']='one supervised mechanical three-piece cycle including return0; no route3, no cold-start preparation'
m['commissioning_start']='IT16 once in sole CAN owner after real references; formal mission/vision provider unbound'
m['current_field_run']={'trace':trace.relative_to(R).as_posix(),
 'third_release_command_s':17.987,'zero_complete_s':19.185,
 'item_to_release_command_s':run['item_to_release_command_s'],
 'physical_placement_observed_after_run':True,
 'operator_observation':'三槽各一件且放稳，无碰擦，已停稳',
 'exact_physical_stable_time_sensor_present':False,'route3_enabled':False,
 'start_reference_seconds':[5.093,1.391]}
m['lead5']['field_verified']=True
m['right160_yaw_retract_overlap']['field_verified']=True
m['three_direct']['right_early_retract_field_verified']=True
m['zero_return_transient']['whole_cycle_passed']=True
m['zero_return_transient']['actual_soft_excursion_in_successful_run_logged']=False
m['final_delivery']=OUT.relative_to(R).as_posix()
# Keep supplied observations readable; old failures and old image hashes remain.
for entry in [m.get('empty_field_run',{})]+[x.get('empty_field_run',{}) for x in m.get('previous_deployed_images',[])]:
 if entry:entry['operator_observation']='全程无碰擦卡顿，已停稳'
manifest_path.write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
shutil.copy2(manifest_path,OUT/'candidate-manifest.json')
readme='''# 整合13最终本地冻结交付（2026-11-09）

本轮三件现场确认通过：第三次松爪命令17.987秒，回0°并软件完成19.185秒；三槽各一件放稳、无碰擦、已停稳。计时不含建参考；第三段未启动。

- 完整源码：project/gongxun-f407.uvprojx，目标Vehicle_LiftRuler及Vehicle_LiftRuler_Offline。
- 固件：images/下冻结AXF，哈希见candidate-manifest.json；实际板为在线8da8171c…，Flash166752字节全匹配。
- 参数：下180°/0mm，左195°/156mm，右165°/160mm，共用放料37°，松爪后至少200ms再转；两处提前5mm、带料回升5mm开始回收，末段转臂与回收重叠。
- project内保留原文件编码/换行；不含Objects/Listings和用户调试布局。外部实车正本、根旧STM32工程未覆盖，未提交推送GitHub。
- 验证及现场原件副本：verification/；完整历史证据仍在仓库evidence/integration/2026-11-07/stage2-mainline/。
- 现场入口：仓库scripts/integration13_empty_overlap_live_20261109.py --three-immediate-once。需要匹配实际Flash、真实参考、摆料和一次授权；默认/--check不接探针。故障保留首故障并发IT9停止，不自动重试。
- 收尾：4310约0.0219°、伸缩−0.3°、升降809.7电机度，夹爪开、三槽有料、动力PE2=1。断电后不继承参考，不盲目再运行。
- 尚未闭环：视觉毫米纠偏、颜色/实例驱动、动态停盘、正式整车使命/真实库存证据提供者、第三段路线与行驶中回0°。本包是已验机械基线，不是完整比赛程序验收。

已核R SRC-01/02、RUN-01/02/04/07、FIELD-06、PWR-01、MECH-01/02；E SRC-04、CAL-02/05、VIS-15、RUN-06、MECH-06；官方Q仍待技术答疑确认。详见根交接和docs/INTEGRATION13_SUB20_CONTINUATION_2026-11-09.md。
'''
(OUT/'README.md').write_text(readme,encoding='utf-8')
files={}
for p in sorted(OUT.rglob('*')):
 if p.is_file():files[p.relative_to(OUT).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
(OUT/'manifest.json').write_text(json.dumps({'files':files,'count':len(files),
 'source_project':P.relative_to(R).as_posix(),'online_sha256':run['sha256'],
 'physically_accepted':True,'route3_accepted':False},indent=2)+'\n',encoding='utf-8')
for path,h in files.items():assert hashlib.sha256((OUT/path).read_bytes()).hexdigest()==h,path
print(json.dumps({'delivery':str(OUT),'files_verified':len(files),'online_sha256':run['sha256'],
 'third_release_command_s':17.987,'zero_complete_s':19.185},indent=2))
