"""Export simulation plots, clearly separate from vehicle measurements."""
import argparse
import csv
import json
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser();p.add_argument('--model-dir',type=Path,required=True);a=p.parse_args()
    out=a.model_dir.resolve()
    os.environ['MPLCONFIGDIR']=str(ROOT/'工创赛/tmp/perimeter_plot_cache')
    sys.path.insert(0,str(ROOT/'工创赛/tmp/turn_sim_plot_deps'))
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
    with (out/'traces.csv').open(encoding='utf8') as f:rows=[{k:float(v) for k,v in r.items()} for r in csv.DictReader(f)]
    backward=[r for r in rows if r['scenario']==0]
    loop=[r for r in rows if r['scenario']==24]
    cfg=json.loads((ROOT/'工创赛/config/p1_perimeter_validation.json').read_text(encoding='utf8'))
    from p1_perimeter_preview import evaluate
    report=evaluate(cfg);poses=[report['start_pose']]+[r['end_pose'] for r in report['rows']]
    fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    ax=axes[0]
    ax.plot([(r['tick_ms']-1000)/1000 for r in backward],[r['ground_y_mm']-2250 for r in backward],label='model ground')
    ax.plot([(r['tick_ms']-1000)/1000 for r in backward],[r['odom_y_mm']-2250 for r in backward],'--',label='encoder + yaw estimate')
    ax.axhline(150,color='gray',linestyle=':',label='150 mm target')
    ax.set_title('后退15cm / 参考5cm/s（模型）',fontproperties=font)
    ax.set_xlabel('time (s)');ax.set_ylabel('backward travel (mm)');ax.legend(fontsize=8);ax.grid(alpha=.3)
    ax=axes[1]
    ax.plot([v[0] for v in poses],[v[1] for v in poses],':',color='gray',label='planned centre path')
    ax.plot([r['ground_x_mm'] for r in loop],[r['ground_y_mm'] for r in loop],label='model ground')
    ax.plot([r['odom_x_mm'] for r in loop],[r['odom_y_mm'] for r in loop],'--',label='encoder + yaw estimate')
    ax.plot(2250,2250,'ko',markersize=5,label='start / target return')
    ax.set_xlim(0,2400);ax.set_ylim(0,2400);ax.set_aspect('equal');ax.set_xlabel('map X (mm)');ax.set_ylabel('map Y (mm)')
    ax.set_title('12步外围路线（模型，无XY补偿）',fontproperties=font);ax.grid(alpha=.3);ax.legend(fontsize=8,loc='lower center')
    fig.suptitle('离线仿真示例，非实车轨迹；全部压力/故障结果见JSON报告',fontproperties=font,fontsize=12)
    fig.savefig(out/'model_paths.png',dpi=160);fig.savefig(out/'model_paths.svg')
    plt.close(fig)


if __name__=='__main__':main()
