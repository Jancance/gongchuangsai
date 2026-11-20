from pathlib import Path
E=Path(__file__).resolve().parent;F=E.parents[1]/'float-timing-20261115'
s=(F/'check_actual_entry.py').read_text(encoding='utf-8')
s=s.replace("E=R/'工创赛/evidence/navigation-planning/2026-11-15/float-timing-20261115';T=R/'工创赛/tmp/float-timing-c-check-20261115'","E=Path(__file__).resolve().parent;T=R/('工创赛/tmp/tail-snapshot-before-20261115' if '--baseline' in sys.argv else '工创赛/tmp/tail-snapshot-after-20261115')")
s=s.replace("auto=(E/'chassis_auto_test.before.c').read_text('gbk') if before_mode else read('chassis_auto_test.c');kin=", "auto=(E/'chassis_auto_test.c.before').read_bytes().decode('gbk') if '--baseline' in sys.argv else read('chassis_auto_test.c');kin=")
s=s.replace("names_auto=['Auto_EnterLocked'", "names_auto=['Auto_ProfileFollowRpmLocked','Auto_DistanceFaultLocked','Auto_DistanceCheckLocked','Auto_EnterLocked'")
s=s.replace("# Prototype for real functions called before their textual definition.","""# Snapshot-only checks run with actual DistanceCheck/Begin/CanStep functions.
extensions=(E/'arm_extensions.c').read_text('utf8')
tests=tests.replace('__attribute__((section("i.test_entry")))',extensions+'\\n__attribute__((section("i.test_entry")))')
tests=tests.replace(' /* Real pure C integer invariants', ' {uint32_t r=observe_distance_control();if(r)return r;}\\n'+(' {uint32_t r=check_tail_snapshots();if(r)return r;}\\n' if '--baseline' not in sys.argv else '')+' /* Real pure C integer invariants',1)

# Prototype for real functions called before their textual definition.""")
s=s.replace("report={'all_passed':True", """def raw_symbol(n,size):
 a=int(re.search(r'\\b'+n+r'\\s+(0x[0-9a-fA-F]+)',symbols)[1],16);return bytes(m.mem_read(a,size))
kind='before' if '--baseline' in sys.argv else 'after'
(E/(kind+'_distance_control.bin')).write_bytes(raw_symbol('distance_control_trace',32*12*4))
if '--baseline' not in sys.argv:(E/'synthetic_snapshot_from_ARM.bin').write_bytes(raw_symbol('snapshot_observation_blob',176))
report={'baseline_control_mode':'--baseline' in sys.argv,'all_passed':True""")
s=s.replace("(E/('before_unit.c' if before_mode else 'after_unit.c'))", "(E/('before_unit.c' if '--baseline' in sys.argv else 'after_unit.c'))")
s=s.replace("(E/('actual_c_before_check.json' if before_mode else 'actual_c_check.json'))", "(E/('actual_c_before_check.json' if '--baseline' in sys.argv else 'actual_c_check.json'))")
s=s.replace("'tested_auto_sha256':hashlib.sha256((E/'chassis_auto_test.before.c' if before_mode else U/'chassis_auto_test.c').read_bytes()).hexdigest()", "'tested_auto_sha256':hashlib.sha256(auto.encode('gbk')).hexdigest()")
s=s.replace("'source_sha256':{n:hashlib.sha256((U/n).read_bytes()).hexdigest() for n in ['app_entry.c'", "'source_sha256':{n:hashlib.sha256(auto.encode('gbk') if n=='chassis_auto_test.c' else (U/n).read_bytes()).hexdigest() for n in ['chassis_auto_test.h','app_entry.c'")
(E/'check_actual_entry.py').write_text(s,encoding='utf-8')
b=(F/'build_isolated.py').read_text(encoding='utf-8').replace("E=R/'工创赛/evidence/navigation-planning/2026-11-15/float-timing-20261115'","E=Path(__file__).resolve().parent").replace('工创赛/tmp/float-timing-build-20261115','工创赛/tmp/tail-snapshot-build-20261115')
(E/'build_isolated.py').write_text(b,encoding='utf-8')
