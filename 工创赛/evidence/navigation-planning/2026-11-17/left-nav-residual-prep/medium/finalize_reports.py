from pathlib import Path
E=Path('D:/Desktop/工创赛/工创赛/evidence/navigation-planning/2026-11-17/left-nav-residual-prep/medium');p=E/'apply_navigation.py';s=p.read_text('utf8');calls=''
for indent in ['    ','        ']:
 old='\n'+indent+'s_baseline40.anchor_valid=0U;\n';new=old+'#if APP_LEFT_NAV_RESIDUAL_ENABLE\n'+indent+'s_navigation40.anchor_valid=0U;\n#endif\n';calls+='ch('+repr('app_odometry.c')+','+repr(old)+','+repr(new)+')\n'
assert s.count('report=[]\n')==1;s=s.replace('report=[]\n',calls+'report=[]\n');p.write_text(s,'utf8')
p=E/'REPORT.md';s=p.read_text('utf8');s=s.replace('此阶段仅独立验证脚本/证据；产品、控制、原始RAW/轮锚/采样/硬件均未改，右1%软件未安装的状态不变。正式生命周期合同已由root确认并经MATLAB验证；产品仍等待root批准，不据模型宣布产品已实现。','MATLAB正式前置通过后，root批准的4源导航候选已完成实现及medium实际C验证，见IMPLEMENTATION_REPORT.md。原始RAW/采样/控制/硬件未改，右1%软件未安装的状态不变。完整Keil与high独立审查由root协调，不据MATLAB或C桩结果宣布实车通过。');p.write_text(s,'utf8')