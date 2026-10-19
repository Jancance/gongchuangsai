"""Keep only the original QR and its payload; preserve exact physical size."""
from pathlib import Path
import json
import fitz
import cv2
import numpy as np
from PIL import Image

root = Path(__file__).resolve().parents[1]
source = root / 'output/pdf/task_qr_20260920'
out = root / 'output/pdf/task_qr_clean_20260921'
qa = root / 'tmp/pdfs/task_qr_clean_20260921'
out.mkdir(parents=True, exist_ok=True)
qa.mkdir(parents=True, exist_ok=True)
rows = json.loads((source / 'verification.json').read_text(encoding='utf-8'))['rows']
mm = 72/25.4
w, h = 297*mm, 210*mm
checks = []
previews = []
for size in (50,70,80):
    src = fitz.open(source / f'task_codes_A4_{size}mm.pdf')
    doc = fitz.open()
    side = size*mm
    box = fitz.Rect((w-side)/2,(h-side)/2,(w+side)/2,(h+side)/2)
    for i, row in enumerate(rows):
        page = doc.new_page(width=w,height=h)
        # Copy the original vector QR only, excluding external corner marks.
        page.show_pdf_page(box,src,i,clip=box)
        code = row['code']
        text_width = fitz.get_text_length(code,fontname='cour',fontsize=20)
        page.insert_text(((w-text_width)/2,box.y1+12*mm),code,fontname='cour',fontsize=20)
    doc.set_metadata({'title':f'Task QR clean {size}mm','subject':'Only QR and task code; print A4 landscape at 100 percent'})
    doc.xref_set_key(doc.pdf_catalog(),'ViewerPreferences','<</PrintScaling /None /Duplex /Simplex>>')
    target = out / f'task_codes_A4_{size}mm_clean.pdf'
    # Strip clipped-away source content, rather than leaving invisible text.
    for page in doc:
        page.add_redact_annot(fitz.Rect(0,0,w,box.y0),fill=False)
        page.add_redact_annot(fitz.Rect(0,box.y1,w,box.y1+1),fill=False)
        page.apply_redactions(images=0,graphics=0,text=0)
    doc.save(target,garbage=4,deflate=True)
    doc.close()
    src.close()
    doc = fitz.open(target)
    assert len(doc)==20
    for i,page in enumerate(doc):
        assert abs(page.rect.width-w)<.01 and abs(page.rect.height-h)<.01
        assert page.get_text().strip()==rows[i]['code'],repr(page.get_text())
        pix = page.get_pixmap(matrix=fitz.Matrix(4,4),clip=box,alpha=False)
        a=np.frombuffer(pix.samples,np.uint8).reshape(pix.height,pix.width,3)
        decoded=cv2.QRCodeDetector().detectAndDecode(a)[0]
        if decoded != rows[i]['code']:
            for resolution in (580,435,290):
                decoded=cv2.QRCodeDetector().detectAndDecode(cv2.resize(a,(resolution,resolution),interpolation=cv2.INTER_AREA))[0]
                if decoded==rows[i]['code']: break
        assert decoded==rows[i]['code']
        p=qa/f'{size}_{i+1:02}.png'
        page.get_pixmap(matrix=fitz.Matrix(1,1),alpha=False).save(p)
        if i in (0,10): previews.append(p)
        checks.append({'size_mm':size,'page':i+1,'code':decoded,'only_payload_text':True})
    doc.close()
sheet=Image.new('RGB',(1200,900),'#d9dfe5')
for i,p in enumerate(previews):
    tile=Image.open(p).convert('RGB')
    tile.thumbnail((590,285))
    sheet.paste(tile,((i%2)*600+(600-tile.width)//2,(i//2)*300+8))
sheet.save(qa/'preview.png')
(out/'verification.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2),encoding='utf-8')
print('Verified 60 pages: A4, original QR size and payload, only code text.')
