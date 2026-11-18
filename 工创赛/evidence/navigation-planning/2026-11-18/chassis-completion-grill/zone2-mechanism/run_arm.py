"""Replay the established radar ARM fixtures against the 4200-step candidate."""
from pathlib import Path


ROOT = Path.cwd()
SCRIPTS = ROOT / "工创赛/scripts"
EVIDENCE = ROOT / "工创赛/evidence/navigation-planning/2026-11-18/chassis-completion-grill/zone2-mechanism"


def replace_once(source, old, new):
    assert source.count(old) == 1, old
    return source.replace(old, new, 1)


def load_fixture(filename):
    source = (SCRIPTS / filename).read_text(encoding="utf8")
    source = replace_once(
        source, "工创赛/evidence/navigation-planning/2026-11-12/radar-zone2/us733",
        EVIDENCE.relative_to(ROOT).as_posix(),
    )
    return source.replace("工创赛/tmp/radar_us733_20261112/", "工创赛/tmp/zone2_mechanism_20261118/")


def run_fixture(filename, source):
    exec(compile(source, str(SCRIPTS / filename), "exec"), {"__name__": "__main__"})


def run_observer():
    name = "check_radar_us733_observer_arm_20261112.py"
    source = load_fixture(name)
    for old, new in [
        ("servo_angle,init_calls;", "servo_angle,init_calls,release_denied,energized_at_stow;"),
        ("servo_angle=(uint32_t)a;outputs++", "servo_angle=(uint32_t)a;if(a==180.0f)energized_at_stow=g_radar_lift_enable;outputs++"),
        ("if(seq!=fixture_lift.sequence", "if(release_denied||seq!=fixture_lift.sequence"),
        ("releases=enables=claims=init_calls=servo_angle=0;", "releases=enables=claims=init_calls=servo_angle=release_denied=energized_at_stow=0;g_radar_lift_enable=0;"),
        ("budget!=750U && budget!=80000U", "budget!=3080U && budget!=80000U"),
        ("RadarZone2Config c={7,0,8,5,5,5,1000,1}", "RadarZone2Config c={7,0,4200,5,5,5,3330,1}"),
        ("status.stage==2 && outputs==2", "status.stage==2 && outputs==2 && last_n==4200 && last_dir==0 && last_interval==733"),
        ("status.stage==5 && up_calls==1&&enables==2&&g_radar_lift_enable", "status.stage==5 && up_calls==1&&enables==2&&g_radar_lift_enable&&last_n==4200&&last_dir==1&&last_interval==733"),
        ("status.stage==6&&releases==1&&g_radar_lift_enable", "status.stage==6&&releases==2&&!g_radar_lift_enable&&servo_angle==180&&!energized_at_stow&&claims==1&&!plan_calls"),
        ("clock_ms+=5;RadarObserver_Service();CHECK(status.stage==7", "clock_ms+=4;RadarObserver_Service();CHECK(status.stage==6&&releases==2&&!plan_calls);clock_ms++;RadarObserver_Service();CHECK(status.stage==7"),
        ("clock_ms+=1001;", "clock_ms+=3331;"),
        ("&&status.stage==5&&cal_calls==1);cases++;", """&&status.stage==5&&cal_calls==1);
 fixture_lift.state=2;fixture_lift.done=4200;RadarObserver_Service();
 CHECK(status.stage==6&&releases==1&&g_radar_lift_enable&&energized_at_stow);
 clock_ms+=5;RadarObserver_Service();CHECK(status.stage==8&&releases==2&&!g_radar_lift_enable&&!plan_calls);cases++;"""),
        ("unsigned test_entry(void){interval_bounds();", """static void release_guard(void){
 radar_client_grid_result_t grid={10,0,{0}};unsigned leg,kind;
 for(leg=0;leg<2;leg++)for(kind=0;kind<6;kind++){
  reset(0);configure();
  if(leg){until_scan();cb.grid(&grid);busy=0;RadarObserver_Service();CHECK(status.stage==5);}
  else{CHECK(RadarObserver_StartZone2(0,3,true));RadarObserver_Service();clock_ms+=5;RadarObserver_Service();CHECK(status.stage==2);}
  fixture_lift.state=2;fixture_lift.done=4200;
  if(kind==0)fixture_lift.sequence++;
  if(kind==1)fixture_lift.result=2;
  if(kind==2)fixture_lift.done--;
  if(kind==3)RadarObserver_Cancel();
  if(kind==4)safe=0;
  if(kind==5)release_denied=1U;
  RadarObserver_Service();
  CHECK(status.stage==(kind==2?(leg?5U:2U):99U));
  CHECK(releases==leg&&g_radar_lift_enable&&servo_angle==45&&!plan_calls);
  cases++;
 }
}
unsigned test_entry(void){release_guard();if(failed)return failed;interval_bounds();"""),
    ]:
        source = replace_once(source, old, new)
    assert source.count("fixture_lift.done=8;") == 3
    run_fixture(name, source.replace("fixture_lift.done=8;", "fixture_lift.done=4200;"))


def run_main():
    name = "check_radar_us733_main_arm_20261112.py"
    source = replace_once(load_fixture(name), "CHECK(configured_steps==8559);", "CHECK(configured_steps==4200);")
    run_fixture(name, source)


if __name__ == "__main__":
    import sys
    sys.path.insert(0, str(SCRIPTS))
    run_observer()
    run_main()
