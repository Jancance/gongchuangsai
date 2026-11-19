"""Record confirmed field state and offline candidate scope without device I/O."""
from pathlib import Path
import json

R = Path(__file__).resolve().parents[1]
E = R / 'evidence/integration/2026-11-10/second-diag'


def main():
    observation = {
        'trace': 'integration13-three-immediate-once-20261110-222451.json',
        'source': 'User confirmation in current chat after STOP',
        'user_text': '已停稳，位置如上，无碰擦',
        'mechanism_physically_stopped': True,
        'first_item_in_slot1': True,
        'left_item_on_source_tray': True,
        'contact_reported': False,
        'three_placed_and_stable': False,
        'right_item_not_attempted': True,
    }
    (E / 'integration13-three-immediate-once-20261110-222451.operator.json').write_text(
        json.dumps(observation, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    manifest_path = E / 'candidate.json'
    m = json.loads(manifest_path.read_text(encoding='utf-8'))
    m.update(hardware_access=True, deployed=True, actual_tests_run=1,
             flash_verified_bytes=171088, field_verified=False,
             latest_run_complete=False, mcu_three_complete=False,
             physical_placement_confirmation_pending=False,
             latest_operator_observation=observation,
             high_speed_overrun_resolved=False,
             current_board_state='Stopped fault; holding power retained at last recorded snapshot')
    manifest_path.write_text(json.dumps(m, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    # Preserve historical paragraphs, but mark the current authoritative state first.
    note = '''## 2026-11-10 · 最新停点：第二档诊断版；半程伸出修复进行中

- 当前板载为 `tmp/pickup_second_diag_20261110/project`，在线 SHA256 `3e0a6a2c2c48061e1d10f7b5273ee4495f33ef86a928891a57f71eb0769d4c39`，烧录后完整读回171088字节匹配。下方c750单速版状态为历史快照。
- 第二档2500/300 RPM这一轮在6.891秒左侧第二件停止，第三件未开始。目标ACK02、FD后目标读取与软件目标相符；采样实际速度最高324.9 RPM，末段42.3→47.1→107 RPM后越位约2.70mm。STOP已确认，用户确认停稳、第一件在1号槽、左件仍在盘、无碰擦。此轮不是三件通过。
- 用户最新要求：只做修订第二档，不做第三档；伸缩200RPM/加减速600RPM/s，升降2500RPM/45055RPM/s；左右伸缩延后到本次下降行程50%才启动，保持单速，不做两段速度。新的独立候选为 `tmp/pickup_half_descent_20261110/project`，状态以 `docs/PICKUP_HALF_DESCENT_2026-11-10.md` 为准；半程时序修改不能冒充末端回摆已修复。
- 保留pzq与历史原始日志。没有清除故障、复位、额外运动或Git上传。下一次换版前需要现场重新摆齐三件、清槽和可靠承托；保留只读驱动参数采集；换版及撤承托确认后，按用户修订的200/600/600做一轮三件，不自动重试。

'''
    p = R / 'P1电控组交接_2026-10-15.md'
    data = p.read_text(encoding='utf-8')
    marker = '## 2026-11-10 · 最新停点：第二档诊断版；半程伸出修复进行中'
    if marker not in data:
        first, rest = data.split('\n', 1)
        p.write_text(first + '\n\n' + note + rest.lstrip('\n'), encoding='utf-8')
    p = R / 'docs/PICKUP_SINGLE_SPEED_2026-11-10.md'
    data = p.read_text(encoding='utf-8')
    banner = '> 当前状态已由第二档诊断版覆盖：板载3e0a6a2c…，最新一轮第二档在左件越位约2.70mm后STOP，用户确认停稳无碰擦。下文c750及其测试为历史记录；最新接续见[PICKUP_HALF_DESCENT_2026-11-10.md](PICKUP_HALF_DESCENT_2026-11-10.md)。\n\n'
    if banner not in data:
        first, rest = data.split('\n', 1)
        p.write_text(first + '\n\n' + banner + rest.lstrip('\n'), encoding='utf-8')
    print('Recorded operator STOP confirmation; source trace preserved unchanged.')


if __name__ == '__main__':
    main()
