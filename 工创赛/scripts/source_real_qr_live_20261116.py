"""Manifest check by default; explicit --snapshot/--watch only reads matched firmware."""
from pathlib import Path
from datetime import datetime
import hashlib,json,sys
import source_mcu_cycle_watch_20261115 as monitor

R=Path(__file__).resolve().parents[1]
E=R/'evidence/integration/2026-11-16/source-real-qr'
SCALARS=('g_source_scan_owner_run','g_source_scan_locked_run','g_source_scan_pending',
 'g_source_scan_accept','g_source_scan_result','g_source_scan_frame_start_ms',
 'g_source_scan_frame_end_ms','g_source_scan_rejected','g_source_scan_displayed_run',
 'g_saoma_rx_count','g_saoma_rx_errors','g_saoma_accept_count','g_saoma_reject_count',
 'g_saoma_last_result','g_saoma_locked','g_saoma_frame_start_ms','g_saoma_frame_end_ms',
 'g_scan_display_result','g_scan_display_count','g_vis_task_valid','g_vis_task_batch',
 'g_vis_task_item','g_vis_task_run_id','g_vis_loaded_mask','g_vis_batch_complete',
 'g_vis_pending_seq','g_vis_pending_slot','g_vis_pending_grasped')

def scanner_symbols(image):
 sys.path.insert(0,str(R/'tmp/gripper_debug_deps'))
 from elftools.elf.elffile import ELFFile
 with (R/image['axf']).open('rb') as stream:
  elf=ELFFile(stream)
  found={s.name:(int(s['st_value']),int(s['st_size'])) for s in elf.get_section_by_name('.symtab').iter_symbols() if s['st_info']['type']=='STT_OBJECT'}
 for name in (*SCALARS,'g_source_scan_code','SAOMA_Data','g_source_cycle_colors','g_vis_task_colors'):
  assert name in found,name
 for name in ('g_source_scan_code','SAOMA_Data'):assert found[name][1]==16,(name,found[name])
 return {'SAOMA_Data':found['SAOMA_Data'][0]}

def manifest():
 c=json.loads((E/'candidate.json').read_text());checks=json.loads((E/'checks.json').read_text())
 assert c['build_passed'] and checks['passed']
 for name,digest in c['changes'].items():assert hashlib.sha256((R/c['project']/name).read_bytes()).hexdigest()==digest,name
 for name,digest in c['unchanged_sha256'].items():assert hashlib.sha256((R/c['project']/name).read_bytes()).hexdigest()==digest,name
 image=c['images']['Vehicle_LiftRuler'];assert hashlib.sha256((R/image['axf']).read_bytes()).hexdigest()==image['sha256']
 scanner_symbols(image)
 return image

def main():
 image=manifest()
 if '--watch' not in sys.argv and '--snapshot' not in sys.argv:
  print(json.dumps(dict(local_check=True,hardware_access=False,image_sha256=image['sha256'],start='one PC13 then real UART5 QR')));return
 import gripper210_attach_read_20261105 as live
 original_load=live.load_image
 def load_image_with_uart():
  symbols,segments=original_load();symbols.update(scanner_symbols(image));return symbols,segments
 live.load_image=load_image_with_uart
 from source_body_probe_20261111 import attach
 sys.path.insert(0,str(R/'src'))
 from smart_carrier.vision_manipulation.committed_pick_board import CommittedPickBoard
 original=CommittedPickBoard.snapshot
 def snapshot(board):
  state=original(board);target=board.target;symbols=board.symbols
  state.update({name:target.read32(symbols[name]) for name in SCALARS})
  for name in ('g_source_scan_code','SAOMA_Data'):
   state[name]=bytes(target.read_memory_block8(symbols[name],16)).split(b'\0',1)[0].decode('ascii',errors='replace')
  for name in ('g_source_cycle_colors','g_vis_task_colors'):
   state[name]=list(target.read_memory_block32(symbols[name],3))
  for name in ('g_source_cycle_run','g_source_cycle_state','g_source_cycle_pose',
               'g_source_cycle_prep','g_source_cycle_item','g_source_cycle_fault'):
   state[name]=target.read32(symbols[name])
  return state
 CommittedPickBoard.snapshot=snapshot
 if '--watch' in sys.argv:
  monitor.E=E;monitor.manifest=manifest;monitor.main();return
 live.IMAGE=R/image['axf'];symbols,segments=live.load_image()
 with attach() as session:
  board=CommittedPickBoard(session.target,symbols,segments)
  report=dict(when=datetime.now().isoformat(),image_sha256=image['sha256'],
   verified_flash_bytes=board.verified_flash_bytes,state=board.snapshot(),
   host_request_writes=0,reset=False,flashed=False)
 out=E/('read-only-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'.json')
 out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
 print(json.dumps(report,ensure_ascii=False,indent=2));print(out)
if __name__=='__main__':main()
