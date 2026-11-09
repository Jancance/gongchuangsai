"""本项目视觉程序与原有四个工具的统一启动入口。"""
import argparse
from pathlib import Path
import runpy
import sys

FEATURES={
    'main':'main.py',
    'color':'颜色标定.py',
    'turntable-settings':'转盘参数设置.py',
    'pickup-regions':'物料抓取区域标定.py',
    'stack-lines':'叠放位直线标定.py',
}

def main():
    # 所选程序负责--help，避免工具帮助被外层启动器截获。
    p=argparse.ArgumentParser(description=__doc__,add_help=False)
    p.add_argument('--feature',choices=tuple(FEATURES),default='main',
                   help='主程序或四个调参工具；其余参数传给所选程序')
    args,rest=p.parse_known_args()
    code=Path(__file__).resolve().parent/'code'
    sys.path.insert(0,str(code))
    script=code/FEATURES[args.feature]
    sys.argv=[str(script),*rest]
    runpy.run_path(str(script),run_name='__main__')

if __name__=='__main__':main()
