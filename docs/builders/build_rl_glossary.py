"""Build RL_GLOSSARY.pdf from the editable RL_GLOSSARY.md source."""

from __future__ import annotations

import re
from pathlib import Path
from xml.sax.saxutils import escape

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    KeepTogether,
    PageTemplate,
    Paragraph,
    Spacer,
)


HERE = Path(__file__).resolve().parent.parent
SOURCE = HERE / "RL_GLOSSARY.md"
OUTPUT = HERE / "RL_GLOSSARY.pdf"
FONT_DIR = Path("/usr/share/fonts/truetype/dejavu")
BLUE = colors.HexColor("#175C78")
GRAY = colors.HexColor("#526370")
INK = colors.HexColor("#1F2830")


def register_fonts() -> None:
    pdfmetrics.registerFont(TTFont("DejaVu", str(FONT_DIR / "DejaVuSans.ttf")))
    pdfmetrics.registerFont(TTFont("DejaVu-Bold", str(FONT_DIR / "DejaVuSans-Bold.ttf")))
    pdfmetrics.registerFontFamily("DejaVu", normal="DejaVu", bold="DejaVu-Bold")


def markup(text: str) -> str:
    """Turn our limited Markdown inline syntax into ReportLab paragraph markup."""
    segments = re.split(r"(\[[^\]]+\]\([^)]+\)|`[^`]+`)", text)
    converted = []
    for segment in segments:
        link = re.fullmatch(r"\[([^\]]+)\]\(([^)]+)\)", segment)
        if link:
            label, target = link.groups()
            converted.append(f'<link href="{escape(target)}" color="#175C78">{escape(label)}</link>')
        elif segment.startswith("`") and segment.endswith("`"):
            converted.append(f'<font color="#175C78">{escape(segment[1:-1])}</font>')
        else:
            converted.append(escape(segment))
    return "".join(converted)


def page_decoration(canvas, doc) -> None:
    canvas.saveState()
    width, height = A4
    canvas.setStrokeColor(colors.HexColor("#D8E4EA"))
    canvas.line(26, height - 38, width - 26, height - 38)
    canvas.setFont("DejaVu-Bold", 8)
    canvas.setFillColor(BLUE)
    canvas.drawString(26, height - 30, "WALL-E RL  /  QUICK REFERENCE")
    canvas.setFont("DejaVu", 8)
    canvas.setFillColor(GRAY)
    canvas.drawRightString(width - 26, height - 30, "RL BASICS")
    canvas.line(26, 31, width - 26, 31)
    canvas.drawString(26, 19, "Definitions with CartPole examples")
    canvas.drawRightString(width - 26, 19, str(doc.page))
    canvas.restoreState()


def build() -> None:
    register_fonts()
    width, height = A4
    margin = 27
    gap = 16
    top = height - 49
    bottom = 42
    column_width = (width - 2 * margin - gap) / 2
    frames = [
        Frame(margin, bottom, column_width, top - bottom, leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0),
        Frame(margin + column_width + gap, bottom, column_width, top - bottom, leftPadding=0, rightPadding=0, topPadding=0, bottomPadding=0),
    ]
    doc = BaseDocTemplate(str(OUTPUT), pagesize=A4, pageTemplates=[PageTemplate(id="two-column", frames=frames, onPage=page_decoration)],
                          title="RL basics: quick-reference glossary", author="Wall-E RL project")
    title = ParagraphStyle("title", fontName="DejaVu-Bold", fontSize=19, leading=23, textColor=INK, spaceAfter=8)
    subtitle = ParagraphStyle("subtitle", fontName="DejaVu", fontSize=8.7, leading=13, textColor=GRAY, spaceAfter=12)
    section = ParagraphStyle("section", fontName="DejaVu-Bold", fontSize=11, leading=15, textColor=BLUE, spaceBefore=13, spaceAfter=6, keepWithNext=True)
    entry = ParagraphStyle("entry", fontName="DejaVu", fontSize=8.6, leading=12.4, textColor=INK, spaceAfter=6.8, alignment=TA_LEFT)
    source = ParagraphStyle("source", parent=entry, fontSize=8.2, leading=11.8, leftIndent=9, firstLineIndent=-9)
    story = []
    pending_heading = None
    in_sources = False
    definitions = 0
    for raw_line in SOURCE.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("# "):
            story.append(Paragraph(escape(line[2:]), title))
        elif line.startswith("## "):
            in_sources = line == "## Sources and next reading"
            pending_heading = Paragraph(escape(line[3:]), section)
        elif line.startswith("- **"):
            match = re.fullmatch(r"- \*\*(.+?)\*\* — (.+)", line)
            if not match:
                raise ValueError(f"Unexpected glossary entry: {line}")
            term, explanation = match.groups()
            definitions += 1
            item = Paragraph(f'<b>{escape(term)}</b> — {markup(explanation)}', entry)
            if pending_heading is not None:
                story.append(KeepTogether([pending_heading, item]))
                pending_heading = None
            else:
                story.append(KeepTogether(item))
        elif line.startswith("- ") and in_sources:
            item = Paragraph("• " + markup(line[2:]), source)
            if pending_heading is not None:
                story.append(KeepTogether([pending_heading, item]))
                pending_heading = None
            else:
                story.append(item)
        else:
            story.append(Paragraph(markup(line), subtitle))
    if definitions < 40:
        raise ValueError(f"Expected a substantial glossary, found {definitions} definitions")
    doc.build(story)
    print(f"Built {OUTPUT} with {definitions} definitions")


if __name__ == "__main__":
    build()
