"""Generate a metric vector checkerboard. Print at actual size, then measure."""
from pathlib import Path
import json
from reportlab.pdfgen import canvas
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from pypdf import PdfReader

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output/pdf'
OUT.mkdir(parents=True,exist_ok=True)
pdf=OUT/'A4_checkerboard_10x7_15mm.pdf'
w,h=A4
c=canvas.Canvas(str(pdf),pagesize=A4,pageCompression=1)
c.setTitle('A4 camera checkerboard - 10 x 7 squares - 15 mm')
c.setAuthor('')
c.setSubject('9 x 6 internal corners; 15 mm pitch; print actual size 100 percent')
c.setFillColorRGB(1,1,1);c.rect(0,0,w,h,stroke=0,fill=1)

# Vector black squares on a white page; no outlines or strokes at grid edges.
left=30*mm;top=h-96*mm;pitch=15*mm
c.setFillColorRGB(0,0,0)
black_count=0
for row in range(7):
    for col in range(10):
        if (row+col)%2==0:
            c.rect(left+col*pitch,top-(row+1)*pitch,pitch,pitch,stroke=0,fill=1)
            black_count+=1

c.showPage();c.save()

r=PdfReader(str(pdf));assert len(r.pages)==1
box=r.pages[0].mediabox
assert abs(float(box.width)/mm-210)<.001 and abs(float(box.height)/mm-297)<.001
assert black_count==35
assert not r.pages[0].extract_text().strip()
description={
    'board_id':'a4-chessboard-10x7-square15-plain-v2',
    'page_mm':[210,297],'squares_xy':[10,7],'inner_corners_xy':[9,6],
    'square_size_mm_nominal':15.0,'board_size_mm':[150,105],
    'board_top_left_on_page_mm':[30,96],'black_squares':black_count,
    'layout':'Centered checkerboard only; no text or measurement lines',
    'print_scale':'100 percent / actual size; no fit/shrink',
    'status':'Printable design; physical dimensions and camera readability unverified',
}
(OUT/'A4_checkerboard_10x7_15mm.json').write_text(json.dumps(description,ensure_ascii=False,indent=2),encoding='utf-8')
print(pdf)
