"""通过既有Keil调试命令框执行命令并读回输出；默认只读表达式。"""
import argparse,ctypes as C,json,time
from ctypes import wintypes as W
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--pid',type=int,required=True)
p.add_argument('--command');p.add_argument('--state',action='store_true');p.add_argument('--allow-write',action='store_true')
p.add_argument('--out',type=Path);p.add_argument('--wait',type=float,default=.4);a=p.parse_args()
if a.state:a.command=r'printf ("CODEX_STATE %u %u %u %u %u %u %u %u\n", g_vm_test_request, g_lift_state, g_arm_sequence_state, g_arm_sequence_enable, g_lift_home_state, g_lift_motion_state, g_test_count_a, g_test_count_b)'
if not a.command:raise SystemExit('Specify command or --state')
if not a.allow_write:
    if not a.command.lstrip().lower().startswith(('printf','dir ')) or '=' in a.command or ';' in a.command:
        raise SystemExit('Read-only bridge only accepts printf/dir without assignment')
u=C.WinDLL('user32',use_last_error=True)
u.GetWindowTextW.argtypes=[W.HWND,W.LPWSTR,C.c_int]
u.GetClassNameW.argtypes=[W.HWND,W.LPWSTR,C.c_int]
u.GetWindowThreadProcessId.argtypes=[W.HWND,C.POINTER(W.DWORD)]
u.GetDlgCtrlID.argtypes=[W.HWND];u.GetDlgCtrlID.restype=C.c_int
u.SendMessageW.argtypes=[W.HWND,W.UINT,W.WPARAM,W.LPARAM];u.SendMessageW.restype=C.c_ssize_t
u.PostMessageW.argtypes=[W.HWND,W.UINT,W.WPARAM,W.LPARAM]
cb=C.WINFUNCTYPE(W.BOOL,W.HWND,W.LPARAM)
u.EnumWindows.argtypes=[cb,W.LPARAM];u.EnumChildWindows.argtypes=[W.HWND,cb,W.LPARAM]
def text(h,cls=False):
    b=C.create_unicode_buffer(1024);(u.GetClassNameW if cls else u.GetWindowTextW)(h,b,len(b));return b.value
roots=[]
@cb
def root(h,arg):
    pid=W.DWORD();u.GetWindowThreadProcessId(h,C.byref(pid))
    if pid.value==a.pid and 'gongxun-f407.uvprojx' in text(h):roots.append(int(h))
    return True
u.EnumWindows(root,0)
if len(roots)!=1:raise SystemExit('Require exactly one verified gongxun Keil window')
panes=[]
@cb
def pane(h,arg):
    if text(h)=='Command':panes.append(int(h))
    return True
u.EnumChildWindows(roots[0],pane,0)
if len(panes)!=1:raise SystemExit('Cannot uniquely locate Command pane')
edits=[];lists=[]
@cb
def child(h,arg):
    kind=text(h,True)
    if kind=='Edit':edits.append(int(h))
    if kind=='ListBox':lists.append(int(h))
    return True
u.EnumChildWindows(panes[0],child,0)
if len(edits)!=1 or len(lists)!=1:raise SystemExit('Unexpected Command controls')
def output():
    n=u.SendMessageW(lists[0],0x18B,0,0)
    if n<0 or n>100000:raise RuntimeError('Invalid command list count')
    result=[]
    for i in range(max(0,n-24),n):
        size=u.SendMessageW(lists[0],0x18A,i,0)
        if size<0 or size>16384:continue
        b=C.create_unicode_buffer(size+1);u.SendMessageW(lists[0],0x189,i,C.cast(b,C.c_void_p).value)
        result.append(b.value)
    return result
before=output()
b=C.create_unicode_buffer(a.command)
u.SendMessageW(edits[0],0x000C,0,C.cast(b,C.c_void_p).value)
u.PostMessageW(edits[0],0x0100,13,0)
u.PostMessageW(edits[0],0x0101,13,0)
time.sleep(max(.1,min(a.wait,10.)))
result=dict(pid=a.pid,window=roots[0],command=a.command,allow_write=a.allow_write,
            edit_after=text(edits[0]),before=before,after=output())
if a.out:
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
