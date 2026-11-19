"""Read-only monitor: exact image match and alignment phase timestamps."""
from pathlib import Path
import sys
import integration23_live_20261115 as monitor
monitor.E=Path(__file__).resolve().parents[1]/'evidence/integration/2026-11-16/source-alignment-fast'
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard
original=CommittedPickBoard.snapshot
def snapshot(board):
    state=original(board)
    for name in ('g_chassis_auto_state','g_chassis_auto_session','g_chassis_auto_segment',
                 'g_chassis_auto_io_result','g_chassis_auto_last_address',
                 'g_source_body_step_elapsed_ms','g_low_wheel_hold_stage'):
        if name in board.symbols:state[name]=board.target.read32(board.symbols[name])
    return state
CommittedPickBoard.snapshot=snapshot
if __name__=='__main__':monitor.main()
