"""只读枚举指定Keil进程的窗口/菜单，不发按键或改变调试状态。"""
import ctypes as C
from ctypes import wintypes as W
import json,sys
u=C.WinDLL('user32',use_last_error=True)
u.GetMenu.argtypes=[W.HWND];u.GetMenu.restype=W.HMENU
u.GetSubMenu.argtypes=[W.HMENU,C.c_int];u.GetSubMenu.restype=W.HMENU
u.GetMenuItemCount.argtypes=[W.HMENU];u.GetMenuItemCount.restype=C.c_int
u.GetMenuItemID.argtypes=[W.HMENU,C.c_int];u.GetMenuItemID.restype=W.UINT
u.GetMenuStringW.argtypes=[W.HMENU,W.UINT,W.LPWSTR,C.c_int,W.UINT]
u.GetWindowTextW.argtypes=[W.HWND,W.LPWSTR,C.c_int]
u.GetClassNameW.argtypes=[W.HWND,W.LPWSTR,C.c_int]
callback=C.WINFUNCTYPE(W.BOOL,W.HWND,W.LPARAM)
u.EnumWindows.argtypes=[callback,W.LPARAM];u.EnumChildWindows.argtypes=[W.HWND,callback,W.LPARAM]
u.GetWindowThreadProcessId.argtypes=[W.HWND,C.POINTER(W.DWORD)]
u.GetParent.argtypes=[W.HWND];u.GetParent.restype=W.HWND
u.GetDlgCtrlID.argtypes=[W.HWND];u.GetDlgCtrlID.restype=C.c_int
def text(h,kind=False):
    b=C.create_unicode_buffer(1024)
    (u.GetClassNameW if kind else u.GetWindowTextW)(h,b,len(b));return b.value
def menu(h,depth=0):
    out=[]
    if not h or depth>3:return out
    for i in range(u.GetMenuItemCount(h)):
        b=C.create_unicode_buffer(250);u.GetMenuStringW(h,i,b,len(b),0x400)
        sub=u.GetSubMenu(h,i)
        out.append(dict(text=b.value,id=u.GetMenuItemID(h,i),submenu=menu(sub,depth+1)))
    return out
pid=int(sys.argv[1]);result=[]
@callback
def visit(h,param):
    p=W.DWORD();u.GetWindowThreadProcessId(h,C.byref(p))
    if p.value==pid:
        children=[]
        @callback
        def child(ch,arg):
            t=text(ch)
            children.append(dict(handle=int(ch),parent=int(u.GetParent(ch) or 0),id=u.GetDlgCtrlID(ch),cls=text(ch,True),text=t[:180]))
            return True
        u.EnumChildWindows(h,child,0)
        result.append(dict(handle=int(h),title=text(h),cls=text(h,True),menu=menu(u.GetMenu(h)),children=children))
    return True
u.EnumWindows(visit,0)
result=[x for x in result if 'gongxun' in x['title'].lower() or x['cls']=='#32770']
if '--command' in sys.argv:
    for root in result:
        selected={c['handle'] for c in root['children'] if c['text']=='Command'}
        for _ in range(4):selected.update(c['handle'] for c in root['children'] if c['parent'] in selected)
        root['children']=[c for c in root['children'] if c['handle'] in selected]
print(json.dumps(result,ensure_ascii=False,indent=2))
