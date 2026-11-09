"""本项目120度相机候选适配。所有入口共用，CPU处理、原图为默认。"""
from pathlib import Path
import multiprocessing as mp
import queue
import subprocess
import time
import cv2
import numpy as np

import json
from camera120_startup_check import initialize_camera, verify_identity

CAMERA_PROFILE_PATH=Path(__file__).resolve().parent/'config/generic_120_camera.json'
CAMERA_PROFILE=json.loads(CAMERA_PROFILE_PATH.read_text(encoding='utf-8'))
DEFAULT_DEVICE=CAMERA_PROFILE['device']
DEFAULT_WIDTH=CAMERA_PROFILE['width']
DEFAULT_HEIGHT=CAMERA_PROFILE['height']
DEFAULT_FPS=float(CAMERA_PROFILE['fps_request'])
DEFAULT_CALIBRATION=Path(__file__).resolve().parent/'config/generic_120_unconfigured.yaml'

def prepare_camera(cap,device,width=DEFAULT_WIDTH,height=DEFAULT_HEIGHT,fps=DEFAULT_FPS,lock_controls=True):
    """仅接受已识别设备与候选模式；每次打开/重连重新调整并关闭AWB。"""
    cv2.ocl.setUseOpenCL(False)
    try:
        if (str(device),width,height,fps)!=(DEFAULT_DEVICE,DEFAULT_WIDTH,DEFAULT_HEIGHT,DEFAULT_FPS):
            raise RuntimeError('120度候选只接受独立profile中的设备和1920x1080 MJPG30请求')
        if not lock_controls:
            raise RuntimeError('120度候选不得绕过启动锁参流程')
        verify_identity(CAMERA_PROFILE)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,4)
        initialize_camera(cap,CAMERA_PROFILE)
        return cap
    except Exception:
        cap.release()
        raise

def open_project_camera(device=DEFAULT_DEVICE,width=DEFAULT_WIDTH,height=DEFAULT_HEIGHT,fps=DEFAULT_FPS):
    return prepare_camera(cv2.VideoCapture(str(device),cv2.CAP_V4L2),device,width,height,fps)

def calibration_maps(path,enabled=False,size=(DEFAULT_WIDTH,DEFAULT_HEIGHT)):
    cv2.ocl.setUseOpenCL(False)
    if not enabled:return None
    path=Path(path)
    if not path.is_file():raise RuntimeError('相机标定文件不存在')
    fs=cv2.FileStorage(path.read_text(encoding='utf-8'),cv2.FILE_STORAGE_READ|cv2.FILE_STORAGE_MEMORY)
    try:
        if not fs.isOpened():raise RuntimeError('相机标定文件不存在，请先取得自己的标定参数')
        actual=(int(fs.getNode('image_width').real()),int(fs.getNode('image_height').real()))
        if actual!=tuple(size):raise RuntimeError('标定分辨率与当前DECXIN配置不一致')
        name=fs.getNode('camera_name').string()
        if 'realtek' in name.lower():raise RuntimeError('禁止使用原作者Realtek相机标定')
        flag=fs.getNode('calibrated')
        if not flag.empty() and not bool(flag.real()):raise RuntimeError('相机内参尚未标定')
        matrix=fs.getNode('camera_matrix').mat();dist=fs.getNode('distortion_coefficients').mat()
        new=fs.getNode('new_camera_matrix').mat()
        if matrix is None or matrix.shape!=(3,3) or dist is None:
            raise RuntimeError('相机内参或畸变矩阵缺失')
        if not np.isfinite(matrix).all() or not np.isfinite(dist).all():raise RuntimeError('标定值非有限数')
        if new is None:new=matrix
        return cv2.initUndistortRectifyMap(matrix,dist,None,new,tuple(size),cv2.CV_16SC2)
    finally:fs.release()

class LatestProjectCamera:
    """主程序专用持续采集进程；只返回新序号、主机年龄合格的最新帧。"""
    def __init__(self,device=DEFAULT_DEVICE,width=DEFAULT_WIDTH,height=DEFAULT_HEIGHT,fps=DEFAULT_FPS):
        if (str(device),width,height,fps)!=(DEFAULT_DEVICE,DEFAULT_WIDTH,DEFAULT_HEIGHT,DEFAULT_FPS):
            raise RuntimeError('120度持续采集仅接受profile候选模式')
        from project_capture import LatestSlot,capture_worker
        ctx=mp.get_context('spawn')
        self._slot=LatestSlot(ctx,shape=(height,width,3));self._ready=ctx.Event();self._stop=ctx.Event();self._status=ctx.Queue()
        self._worker=ctx.Process(target=capture_worker,args=(device,self._slot,self._ready,self._stop,self._status,4),daemon=True)
        self._sequence=0;self.last_read_completed_time=None;self._error=None;self._released=False;self.events=[]
        self._worker.start();deadline=time.monotonic()+40
        try:
            while not self._ready.is_set():
                self._poll()
                if self._error or not self._worker.is_alive():raise RuntimeError(self._error or '采集进程启动失败')
                if time.monotonic()>deadline:raise RuntimeError('采集启动超时')
                self._ready.wait(.1)
        except Exception:self.release();raise
    def _poll(self):
        while True:
            try:event=self._status.get_nowait()
            except queue.Empty:break
            self.events.append(event)
            if event['event']=='error':self._error=event['message']
    def isOpened(self):
        return not self._released and self._worker.is_alive() and self._error is None
    def read(self):
        deadline=time.monotonic()+.5
        while self.isOpened() and time.monotonic()<deadline:
            self._poll()
            if self._error:return False,None
            item=self._slot.take_after(self._sequence)
            if item:
                seq,stamp,image=item;self._sequence=seq
                if time.monotonic()-stamp<=.15:
                    self.last_read_completed_time=stamp
                    return True,image
            time.sleep(.002)
        return False,None
    def release(self):
        if self._released:return
        from project_capture import stop_worker
        self._released=True
        stop_worker(self._worker,self._stop)
        self._worker.close();self._status.close()

class DisabledSerialLink:
    """桌面入口不打开任何串口；原协议模块仍保留作离线编解码参考。"""
    status='DISABLED_P2_INTEGRATION_PENDING'
    last_sent_ring_alignment=None
    last_sent_material_slots=None
    last_sent_center=None
    last_task_code=None
    sent_count=0
    sent_ring_batches=0
    sent_material_batches=0
    received_task_count=0
    invalid_task_count=0
    def __init__(self,*args,**kwargs):pass
    def poll_commands(self):return []
    def send_center(self,*args):return False
    def send_ring_alignment(self,*args):return False
    def send_material_slots(self,*args):return False
    def close(self):pass
