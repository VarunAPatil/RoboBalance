"""Render the editable implementation prompt as an A4 PDF."""

from pathlib import Path
from xml.sax.saxutils import escape

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Paragraph, SimpleDocTemplate


def build():
    here = Path(__file__).resolve().parent.parent
    fonts = Path('/usr/share/fonts/truetype/dejavu')
    for name, file in [('DejaVu', 'DejaVuSans.ttf'), ('DejaVu-Bold', 'DejaVuSans-Bold.ttf')]:
        pdfmetrics.registerFont(TTFont(name, str(fonts / file)))
    blue = colors.HexColor('#175C78')
    body = ParagraphStyle('body', fontName='DejaVu', fontSize=9.3, leading=13.2, spaceAfter=8)
    title = ParagraphStyle('title', parent=body, fontName='DejaVu-Bold', fontSize=19, leading=25, textColor=blue, spaceAfter=14)
    heading = ParagraphStyle('heading', parent=body, fontName='DejaVu-Bold', fontSize=12, leading=17, textColor=blue, spaceBefore=12, spaceAfter=7, keepWithNext=True)
    story = []
    for block in (here / 'MUJOCO_PREVIEW_PROMPT.md').read_text().strip().split('\n\n'):
        if block.startswith('# '): text, style = block[2:], title
        elif block.startswith('## '): text, style = block[3:], heading
        else: text, style = block, body
        story.append(Paragraph(escape(text).replace('\n', '<br/>'), style))

    def footer(canvas, doc):
        canvas.setFont('DejaVu', 8)
        canvas.setFillColor(blue)
        canvas.drawString(40, 24, 'WALL-E RL • MuJoCo preview implementation prompt • 10 October 2026')
        canvas.drawRightString(A4[0]-40, 24, str(doc.page))

    output = here / 'MUJOCO_PREVIEW_PROMPT.pdf'
    SimpleDocTemplate(str(output), pagesize=A4, leftMargin=40, rightMargin=40, topMargin=38, bottomMargin=44,
                      title='Wall-E MuJoCo preview: detailed implementation prompt', author='Wall-E RL project').build(story, onFirstPage=footer, onLaterPages=footer)
    print(output)


if __name__ == '__main__':
    build()
