from pathlib import Path
import re
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, PageBreak
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfbase import pdfmetrics

root = Path(r'D:\code\tc264\Seekfree_TC264_Opensource_Library\docs')
out = root / 'lvgl_beginner_guide.pdf'
md = (root / 'lvgl_beginner_guide.md').read_text(encoding='utf-8')
img = Path(r'C:\Users\10503\AppData\Local\Temp\codex-clipboard-eae053dc-09af-4bd3-9a6a-ac2cbe12c9fb.jpg')

font_candidates = [
    r'C:\Windows\Fonts\msyh.ttc',
    r'C:\Windows\Fonts\msyh.ttf',
    r'C:\Windows\Fonts\simhei.ttf',
    r'C:\Windows\Fonts\simsun.ttc',
]
font_name = 'Helvetica'
for fp in font_candidates:
    p = Path(fp)
    if p.exists():
        try:
            pdfmetrics.registerFont(TTFont('CJKFont', str(p)))
            font_name = 'CJKFont'
            break
        except Exception:
            pass

styles = getSampleStyleSheet()
base = ParagraphStyle('base', parent=styles['BodyText'], fontName=font_name, fontSize=10.5, leading=15, alignment=TA_LEFT, spaceAfter=6)
head = ParagraphStyle('head', parent=styles['Heading1'], fontName=font_name, fontSize=18, leading=22, spaceAfter=10)
sub = ParagraphStyle('sub', parent=styles['Heading2'], fontName=font_name, fontSize=13, leading=16, spaceBefore=10, spaceAfter=6)
code = ParagraphStyle('code', parent=base, fontName='Courier', fontSize=9, leading=12, backColor=None)

story = []
story.append(Paragraph('LVGL 小白入门笔记', head))
if img.exists():
    story.append(Image(str(img), width=160*mm, height=120*mm))
    story.append(Spacer(1, 6*mm))

lines = md.splitlines()
para = []
in_code = False
for line in lines:
    if line.startswith('```'):
        if in_code:
            story.append(Paragraph('<br/>'.join(para).replace('&','&amp;').replace('<','&lt;').replace('>','&gt;').replace(' ','&nbsp;'), code))
            story.append(Spacer(1, 3*mm))
            para = []
        in_code = not in_code
        continue
    if in_code:
        para.append(line)
        continue
    if not line.strip():
        if para:
            story.append(Paragraph(' '.join(para), base))
            para = []
        continue
    if line.startswith('# '):
        if para:
            story.append(Paragraph(' '.join(para), base)); para=[]
        story.append(Paragraph(line[2:].strip(), head))
    elif line.startswith('## '):
        if para:
            story.append(Paragraph(' '.join(para), base)); para=[]
        story.append(Paragraph(line[3:].strip(), sub))
    elif line.startswith('### '):
        if para:
            story.append(Paragraph(' '.join(para), base)); para=[]
        story.append(Paragraph(line[4:].strip(), sub))
    elif line.startswith('- '):
        if para:
            story.append(Paragraph(' '.join(para), base)); para=[]
        story.append(Paragraph('• ' + line[2:].strip(), base))
    else:
        para.append(line.strip())
if para:
    story.append(Paragraph(' '.join(para), base))

doc = SimpleDocTemplate(str(out), pagesize=A4, rightMargin=16*mm, leftMargin=16*mm, topMargin=14*mm, bottomMargin=14*mm)
doc.build(story)
print(out)
