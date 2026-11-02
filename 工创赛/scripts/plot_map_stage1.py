"""将实际ARM C导出的坐标画成核对图；不把名义地图当实物测量。"""
from pathlib import Path
import csv
import json
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'工创赛/tmp/turn_sim_plot_deps'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.font_manager import FontProperties

OUT=ROOT/'工创赛/evidence/integration/2026-11-02/map_stage1'
data=json.loads((OUT/'actual_c_map.json').read_text(encoding='utf8'))
font=FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
fig,ax=plt.subplots(figsize=(9,9),layout='constrained')
ax.add_patch(Rectangle((0,0),2400,2400,facecolor='#e9edf1',edgecolor='#244059',lw=1.4))
for x in (550,1400):
    for y in (550,1400):
        ax.add_patch(Rectangle((y,x),450,450,facecolor='#fff0a3',edgecolor='#d3b227',lw=1))
for x in (0,2100):
    ax.add_patch(Rectangle((x,0),300,300,facecolor='#3976e5',alpha=.85))
for value in range(480,2400,480):
    ax.plot([0,2400],[value,value],color='#8292a3',lw=.6)
    ax.plot([value,value],[0,2400],color='#8292a3',lw=.6)
centers=list(zip(data['centers_mm'][::2],data['centers_mm'][1::2]))
parks=list(zip(data['parking_mm'][::2],data['parking_mm'][1::2]))
for node,(x,y) in enumerate(centers):
    ax.plot(x,y,'.',color='#596675',ms=3)
    ax.text(x,y+43,str(node),ha='center',va='center',fontsize=10,color='#1a2a3c')
names={0:'启停区2',4:'启停区1',14:'原料区',10:'粗加工区',22:'暂存区'}
for node,label in names.items():
    x,y=parks[node]
    ax.plot(x,y,'o',color='#173b6e',ms=6)
    dx=-28 if node==4 else 28
    ax.annotate(f'{label}\nX{x:.0f}, Y{y:.0f}',(x,y),xytext=(dx,17),textcoords='offset points',
                fontproperties=font,fontsize=9,ha='right' if node==4 else 'left',color='#173b6e')
sx,sy,gx,gy,forward_cm,left_cm,gap=data['stage1']
ax.annotate('',(sx,gy),(sx,sy),arrowprops=dict(arrowstyle='->',lw=3,color='#d34438'))
ax.annotate('',(gx,gy),(sx,gy),arrowprops=dict(arrowstyle='->',lw=3,color='#d34438'))
ax.plot(gx,gy,'s',color='#d34438',ms=8)
ax.text(1500,465,f'先前进{forward_cm:.0f}cm，再左移{left_cm:.0f}cm；不旋转',fontproperties=font,color='#aa2c23',fontsize=10,ha='center')
ax.annotate('车头 +Y',(sx,sy+420),(sx,sy+85),fontproperties=font,fontsize=9,
            ha='center',arrowprops=dict(arrowstyle='->',lw=1.3,color='#173b6e'))
ax.text(gx-100,gy+50,f'扫码点({gx:.0f}, {gy:.0f})',ha='right',va='bottom',fontproperties=font,fontsize=10,color='#aa2c23')
ax.annotate('',(gx,gap),(gx,0),arrowprops=dict(arrowstyle='<->',lw=1,color='#475569'))
ax.text(gx-50,gap*.5,f'名义读距{gap:.1f}mm\n待实测',ha='right',fontproperties=font,fontsize=8,color='#475569')
ax.plot([1100,1300],[0,0],color='#20252b',lw=6)
ax.text(1200,-105,'二维码板（位置示意，实际工作距离待量）',ha='center',fontproperties=font,fontsize=9)
ax.set(xlim=(-100,2500),ylim=(-160,2500),xticks=range(0,2401,480),yticks=range(0,2401,480))
ax.set_aspect('equal')
ax.set_xlabel('X / mm →（启停区2到启停区1）',fontproperties=font)
ax.set_ylabel('Y / mm →（启停区2到粗加工区）',fontproperties=font)
ax.set_title('用户坐标系与第一阶段候选路线\n格中心480mm均分；扫码点为两区中点向+Y前移150mm',fontproperties=font,fontsize=13)
fig.savefig(OUT/'map_stage1.png',dpi=150)
with (OUT/'map_coordinates.csv').open('w',encoding='utf-8-sig',newline='') as stream:
    writer=csv.writer(stream)
    writer.writerow(['grid_id','center_x_mm','center_y_mm','park_x_mm','park_y_mm','station'])
    for node,((cx,cy),(px,py)) in enumerate(zip(centers,parks)):
        writer.writerow([node,cx,cy,px,py,names.get(node,'')])
    writer.writerow(['SCAN','','',gx,gy,'两区中点向+Y前移150mm；不是2号格中心'])
print(OUT/'map_stage1.png')
