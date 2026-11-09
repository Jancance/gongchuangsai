"""仅向已验证的当前Keil窗口发送一次Ctrl+F5切换调试会话。"""
import ctypes as C,time,json
from ctypes import wintypes as W
u=C.WinDLL('user32',use_last_error=True)
u.SetForegroundWindow.argtypes=[W.HWND];u.SetForegroundWindow.restype=W.BOOL
u.GetForegroundWindow.restype=W.HWND
u.GetWindowTextW.argtypes=[W.HWND,W.LPWSTR,C.c_int]
u.GetWindowThreadProcessId.argtypes=[W.HWND,C.POINTER(W.DWORD)]
u.ShowWindow.argtypes=[W.HWND,C.c_int]
u.keybd_event.argtypes=[W.BYTE,W.BYTE,W.DWORD,C.c_size_t]
h=1967766;b=C.create_unicode_buffer(600);u.GetWindowTextW(h,b,len(b))
pid=W.DWORD();u.GetWindowThreadProcessId(h,C.byref(pid))
assert pid.value==22260 and 'gongxun-f407.uvprojx' in b.value
u.ShowWindow(h,9);u.SetForegroundWindow(h);time.sleep(.2)
assert int(u.GetForegroundWindow() or 0)==h,'Keil not foreground: no key sent'
for key,flags in [(0x11,0),(0x74,0),(0x74,2),(0x11,2)]:u.keybd_event(key,0,flags,0)
time.sleep(.5)
print(json.dumps(dict(window=h,pid=pid.value,shortcut='Ctrl+F5',sent_once=True)))
