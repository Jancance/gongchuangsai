"""Generate exact-size vector QR PDFs and validate rendered payloads."""
from pathlib import Path
import itertools
import json
import random
import cv2
import fitz
import numpy as np
from PIL import Image, ImageOps, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'output/pdf/task_qr_20260920'
QA = ROOT / 'tmp/pdfs/task_qr_20260920'
OUT.mkdir(parents=True, exist_ok=True)
QA.mkdir(parents=True, exist_ok=True)
MM = 72 / 25.4
W, H = 297 * MM, 210 * MM
FONT = 'C:/Windows/Fonts/simhei.ttf'
rng = random.Random(20260920)
colors = dict(zip('123456', ['红', '黄', '蓝', '绿', '黑', '浅蓝']))
perms = list(itertools.permutations('123'))
triples = list(itertools.combinations('123456', 3))
rng.shuffle(triples)
rows = []
for group in ('R', 'N'):
    for i in range(10):
        first = list(triples[i]) if group == 'R' else rng.sample(list('123456'), 3)
        rng.shuffle(first)
        second = first[:] if group == 'R' else list(set('123456') - set(first))
        second.sort()
        rng.shuffle(second)
        if group == 'R' and first == second:
            second = second[1:] + second[:1]
        p1 = ''.join(rng.choice(perms))
        p2 = ''.join(rng.choice(perms))
        code = '+'.join((''.join(first), p1, ''.join(second), p2))
        rows.append(dict(id=f'{group}{i+1:02}', group=group, code=code))
assert len({r['code'] for r in rows}) == 20
for row in rows:
    a, b, c, d = row['code'].split('+')
    assert len(set(a)) == len(set(c)) == 3
    assert sorted(b) == sorted(d) == list('123')
    if row['group'] == 'R':
        assert set(a) == set(c)
    else:
        assert sorted(a+c) == list('123456')

def centered(page, text, baseline, size=12, font='cn', color=(0, 0, 0)):
    f = fitz.Font(fontfile=FONT) if font == 'cn' else fitz.Font(fontname='cour')
    width = f.text_length(text, fontsize=size)
    assert width < W - 24 * MM
    page.insert_text(((W-width)/2, baseline), text, fontname=font, fontsize=size, color=color)

def matrix_for(code):
    params = cv2.QRCodeEncoder_Params()
    params.correction_level = cv2.QRCodeEncoder_CORRECT_LEVEL_M
    a = cv2.QRCodeEncoder_create(params).encode(code)
    yy, xx = np.where(a == 0)
    core = a[yy.min():yy.max()+1, xx.min():xx.max()+1]
    assert core.shape[0] == core.shape[1]
    return np.pad(core, 4, constant_values=255)

check = []
previews = []
for size in (50, 70, 80):
    doc = fitz.open()
    bounds = []
    for idx, row in enumerate(rows):
        page = doc.new_page(width=W, height=H)
        page.insert_font(fontname='cn', fontfile=FONT)
        title = '两批同三色 · 顺序随机' if row['group'] == 'R' else '两批六色各一次 · 识别测试'
        centered(page, f"{row['id']}  |  {title}", 20*MM, 15)
        centered(page, f'{size} × {size} mm（含四周白边）', 29*MM, 11)
        side = size * MM
        x0, y0 = (W-side)/2, (H-side)/2
        matrix = matrix_for(row['code'])
        n = matrix.shape[0]
        unit = side/n
        shape = page.new_shape()
        for y in range(n):
            x = 0
            while x < n:
                if matrix[y, x] != 0:
                    x += 1
                    continue
                start = x
                while x < n and matrix[y, x] == 0:
                    x += 1
                shape.draw_rect(fitz.Rect(x0+start*unit, y0+y*unit, x0+x*unit, y0+(y+1)*unit))
        shape.finish(color=None, fill=(0,0,0))
        shape.commit()
        bounds.append((x0, y0, x0+side, y0+side))
        # Only external light corner marks: no line through the QR quiet zone.
        for sx, sy in ((-1,-1), (1,-1), (-1,1), (1,1)):
            cx = x0 if sx == -1 else x0+side
            cy = y0 if sy == -1 else y0+side
            page.draw_line((cx+sx*2*MM, cy), (cx+sx*4*MM, cy), color=(.65,.65,.65), width=.3)
            page.draw_line((cx, cy+sy*2*MM), (cx, cy+sy*4*MM), color=(.65,.65,.65), width=.3)
        centered(page, row['code'], y0+side+12*MM, 20, font='cour')
        if row['group'] == 'N':
            centered(page, '仅用于识别/解析测试；不作为同色码垛任务。', 177*MM, 10)
        else:
            centered(page, '两批颜色集合相同，每批内部不重复。', 177*MM, 10)
        centered(page, 'A4横向 · 单面 · 实际大小100% · 关闭适合页面/缩放', 186*MM, 10)
        centered(page, f'颜色：1红  2黄  3蓝  4绿  5黑  6浅蓝    |    第{idx+1}/20页', 195*MM, 9)
        page.draw_line((15*MM,195*MM), (65*MM,195*MM), width=.6)
        for x in (15,65):
            page.draw_line((x*MM,193.5*MM), (x*MM,196.5*MM), width=.6)
        page.insert_text((23*MM,191*MM), '校准线 50 mm', fontname='cn', fontsize=9)
    doc.set_metadata({'title': f'A4 Task QR - {size}mm - 20 codes', 'subject': 'Vector QR, four-module quiet zone included; print actual size 100%'})
    doc.xref_set_key(doc.pdf_catalog(), 'ViewerPreferences', '<</PrintScaling /None /Duplex /Simplex>>')
    path = OUT / f'task_codes_A4_{size}mm.pdf'
    doc.save(path, garbage=4, deflate=True)
    doc.close()
    doc = fitz.open(path)
    assert len(doc) == 20
    assert '/None' in doc.xref_get_key(doc.pdf_catalog(), 'ViewerPreferences')[1]
    for idx, page in enumerate(doc):
        assert abs(page.rect.width-W) < .01 and abs(page.rect.height-H) < .01
        assert rows[idx]['code'] in page.get_text()
        rect = fitz.Rect(bounds[idx])
        assert abs(rect.width/MM-size) < 1e-8
        # Verify actual vector geometry, not only the planned rectangle.
        fill = [d for d in page.get_drawings() if d['fill'] == (0.0,0.0,0.0)]
        assert len(fill) == 1
        core_mm = fill[0]['rect'].width/MM
        matrix_n = matrix_for(rows[idx]['code']).shape[0]
        assert abs(core_mm-size*(matrix_n-8)/matrix_n) < .01
        pix = page.get_pixmap(matrix=fitz.Matrix(4,4), clip=rect, alpha=False)
        im = np.frombuffer(pix.samples, np.uint8).reshape(pix.height,pix.width,3)
        decoded, _, _ = cv2.QRCodeDetector().detectAndDecode(im)
        if decoded != rows[idx]['code']:
            for pixels in (580, 435, 290):
                sample = cv2.resize(im, (pixels,pixels), interpolation=cv2.INTER_AREA)
                decoded, _, _ = cv2.QRCodeDetector().detectAndDecode(sample)
                if decoded == rows[idx]['code']:
                    break
        assert decoded == rows[idx]['code'], (size,idx,decoded)
        page.get_pixmap(matrix=fitz.Matrix(1,1), alpha=False).save(QA / f'{size}_{idx+1:02}.png')
        check.append({'size_mm':size,'page':idx+1,'id':rows[idx]['id'],'decoded':decoded,'ok':True})
        if idx in (0,10):
            previews.append(QA / f'{size}_{idx+1:02}.png')
    doc.close()

sheet = Image.new('RGB',(1200,900),'#d9dfe5')
for i,p in enumerate(previews):
    tile=Image.open(p).convert('RGB')
    tile.thumbnail((590,285))
    # columns = groups, rows = physical sizes
    sheet.paste(tile,((i%2)*600+(600-tile.width)//2,(i//2)*300+8))
sheet.save(QA / 'preview_contact.png')
(OUT/'verification.json').write_text(json.dumps({'seed':20260920,'rows':rows,'checks':check,'pages':60,'qr_size_includes_quiet_zone':True,'quiet_zone_modules':4},ensure_ascii=False,indent=2),encoding='utf-8')
listing = ['# 二维码打印包说明','',
'生成日期：2026-09-20。20条任务码，50/70/80mm三份PDF，各20页；每页一个码，A4横向。',
'PDF为本地生成的标准静态文本QR码，不是草料网站导出。二维码只含15个ASCII字符的任务码，不含标题、网址、空格或换行。',
'尺寸包含四边各4模块白色静区；黑色图案本身小于标注尺寸。角标在尺寸范围外，不要裁掉白边。',
'打印：A4横向、单面、实际大小/100%，关闭适合页面及多页合一。先打一页，尺量左下50mm校准线，再批量打印。建议哑光白纸。',
'每份第1-10页R组：两批同三种颜色，各批内部不同且两批顺序不同。第11-20页N组：两批六色各一次，仅识别/解析测试，不作为同色码垛任务。',
'50/70mm是缩小识别测试尺寸；80mm对应用户提供的命题尺寸。本文件不替代正式赛场二维码板。',
'', '## 草料网站操作',
'1. 打开 https://cli.im/text/other ，选择文本二维码。',
'2. 每次只粘贴下表一整条任务码，例如156+123+516+231，不附编号或说明。',
'3. 点击生成二维码，选择普通黑白QR Code，不添加Logo，不使用网址/活码。',
'4. 下载SVG或PNG；网站图片像素不是纸上毫米尺寸。已有本包PDF可以直接打印，不必重复在网站生成。',
'5. 若用草料批量生码，按官方页面说明一行一个任务码导入；保持静态纯文本，下载后仍要核对内容和物理尺寸。',
'', '## 规则与证据',
'R：采用用户本轮提供的四段格式及颜色映射1红2黄3蓝4绿5黑6浅蓝；第2/4段均为123排列。',
'E：4模块白边、无Logo、三尺寸测试、每页单码以及随机样本为工程测试设计。',
'Q：基线QR-04仍含历史待确认项；本次按用户原文生成，未静默修改基线或宣称六色不重复是正式赛题。',
'已检查QR-01～04、VIS-01；验证60页A4尺寸、矢量二维码几何、文字与解码一致及PrintScaling=None，见verification.json。打印机实际缩放和相机识别表现仍待实物验收。',
'', '## 20组任务码','', '| 编号 | 任务码 | 第一批颜色 | 第二批颜色 |','|---|---|---|---|']
for row in rows:
    a,_,c,_=row['code'].split('+')
    listing.append(f"| {row['id']} | `{row['code']}` | {'、'.join(colors[x] for x in a)} | {'、'.join(colors[x] for x in c)} |")
(OUT/'打印说明与任务码.md').write_text('\n'.join(listing)+'\n',encoding='utf-8')
(OUT/'草料批量输入_20组.txt').write_text('\n'.join(r['code'] for r in rows)+'\n',encoding='utf-8')
print(json.dumps({'output':str(OUT),'pages_checked':len(check),'preview':str(QA/'preview_contact.png')},ensure_ascii=False))
