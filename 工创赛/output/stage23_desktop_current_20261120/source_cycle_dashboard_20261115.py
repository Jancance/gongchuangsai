"""One960px-wide window for the1024x768 Orange Pi desktop and laptop VNC."""
from functools import lru_cache
from pathlib import Path
import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont

LABELS={0:'等待按键启动',1:'机构准备中',2:'视觉纠偏中',3:'底盘微调中',4:'纠偏收尾中',
        5:'纠偏完成',6:'观察物料',7:'抓取与放料中',8:'本轮动作完成',9:'已停止'}

@lru_cache(maxsize=4)
def font(size):
    for path in ('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
                 '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc','C:/Windows/Fonts/msyh.ttc'):
        if Path(path).is_file():return ImageFont.truetype(path,size)
    return ImageFont.load_default()

def compose(alignment,pickup,status,*,active,body=None,observe_motion=False,learning=False,preview_only=False,initial_rotation=None):
    canvas=np.full((400,960,3),(32,28,25),dtype=np.uint8)
    for x,image in ((12,alignment),(486,pickup)):
        canvas[82:342,x:x+462]=cv2.resize(image,(462,260),interpolation=cv2.INTER_AREA)
    image=Image.fromarray(cv2.cvtColor(canvas,cv2.COLOR_BGR2RGB));draw=ImageDraw.Draw(image)
    white=(235,239,245);muted=(160,175,190);accent=(86,205,190)
    title=LABELS.get(status['state'],'等待机器人') if active else '等待机器人连接'
    if learning:title='学习转盘运动：先观察一圈'
    if active and status.get('scheduled'):title='等待物料进入抓取时刻'
    if body is not None and body.get('observation_valid') is False:title='等待有效盘缘图像'
    if active and status['state']==2 and body is None:title='正在重新寻找真实外圆'
    if active and status['state']==6 and initial_rotation and not initial_rotation['ready']:
        title=f"等待转盘换位 {abs(initial_rotation['observed_net_deg']):.0f}/120°"
    if active and status['state']==6 and initial_rotation and initial_rotation.get('initial_branch')=='classifying':
        title='正在判断双鸣后首次运动状态'
    if preview_only:title='盘缘诊断预览：未发送纠偏量'
    draw.text((15,10),title,font=font(23),fill=white)
    draw.text((16,49),'视觉纠偏',font=font(20),fill=accent)
    draw.text((490,49),'物料抓取',font=font(20),fill=accent)
    progress=f"动作进度 {status['item']}/3" if active else '动作尚未启动'
    draw.text((700,15),progress,font=font(18),fill=muted)
    error=f"纠偏误差 {body['error_norm_mm']:.1f} / 5.0 mm" if body else '底盘按原料盘边缘定位'
    if body is not None and body.get('observation_valid') is False:error=f"无效拟合：{body['fit_rms_px']:.1f} px，等待新图"
    draw.text((16,356),error,font=font(17),fill=muted)
    detail='转盘持续旋转 · 正在采集运动，不夹取' if observe_motion else '物料连续停稳5帧后抓取'
    draw.text((490,356),detail,font=font(17),fill=muted)
    return cv2.cvtColor(np.asarray(image),cv2.COLOR_RGB2BGR)
