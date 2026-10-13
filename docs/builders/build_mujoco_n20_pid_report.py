"""Build a concise PDF of the assumed N20 motor-model PID experiment."""

from pathlib import Path
from xml.sax.saxutils import escape

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle


HERE = Path(__file__).resolve().parent.parent
OUTPUT = HERE / "MUJOCO_N20_ASSUMED_PID_REPORT.pdf"
BLUE = colors.HexColor("#175C78")
INK = colors.HexColor("#1F2830")
GRAY = colors.HexColor("#526370")


def build() -> None:
    font_dir = Path("/usr/share/fonts/truetype/dejavu")
    pdfmetrics.registerFont(TTFont("DejaVu", str(font_dir / "DejaVuSans.ttf")))
    pdfmetrics.registerFont(TTFont("DejaVu-Bold", str(font_dir / "DejaVuSans-Bold.ttf")))
    pdfmetrics.registerFont(TTFont("DejaVuMono", str(font_dir / "DejaVuSansMono.ttf")))
    pdfmetrics.registerFontFamily("DejaVu", normal="DejaVu", bold="DejaVu-Bold")
    styles = {
        "title": ParagraphStyle("title", fontName="DejaVu-Bold", fontSize=19,
                                 leading=23, textColor=BLUE, spaceAfter=4),
        "sub": ParagraphStyle("sub", fontName="DejaVu", fontSize=8.5,
                               leading=12, textColor=GRAY, spaceAfter=10),
        "head": ParagraphStyle("head", fontName="DejaVu-Bold", fontSize=10.5,
                                leading=14, textColor=BLUE, spaceBefore=8, spaceAfter=4),
        "body": ParagraphStyle("body", fontName="DejaVu", fontSize=8,
                                leading=11.1, textColor=INK, spaceAfter=5),
        "cell": ParagraphStyle("cell", fontName="DejaVu", fontSize=7.3,
                                leading=9, textColor=INK),
        "code": ParagraphStyle("code", fontName="DejaVuMono", fontSize=6.5,
                                leading=9, backColor=colors.HexColor("#EFF4F7"),
                                borderPadding=6, spaceBefore=2, spaceAfter=6),
    }
    story = []

    def p(text: str, style: str = "body") -> None:
        story.append(Paragraph(text, styles[style]))

    def footer(canvas, doc) -> None:
        canvas.saveState()
        canvas.setStrokeColor(colors.HexColor("#D8E4EA"))
        canvas.line(36, 32, A4[0] - 36, 32)
        canvas.setFont("DejaVu", 7)
        canvas.setFillColor(GRAY)
        canvas.drawString(36, 20, "WALL-E RL  /  Simulation-only motor assumption  /  13 Oct 2026")
        canvas.drawRightString(A4[0] - 36, 20, str(doc.page))
        canvas.restoreState()

    p("Assumed N20 motor & PID sweep", "title")
    p("N20-12V-200 supplier-table values • 11.8 V assumed supply • MuJoCo 3.7.0", "sub")
    p("Motor curve approximation", "head")
    p("Used an effective geared-output-shaft DC model derived from the supplied stall/no-load endpoints and currents. This is an explicit experiment assumption, not measured characterization.")
    rows = [
        ["Parameter", "Assumed / inferred value"],
        ["Supply", "11.8 V"],
        ["Resistance", "16.0 Ω = 12 V / 0.75 A stall current"],
        ["Torque constant", "0.25583 N·m/A = 1.8 kg-cm / (0.75 − 0.06 A)"],
        ["Back-EMF constant", "0.52712 V/(rad/s), fitted to 200 RPM and 0.06 A no-load current"],
        ["Stall torque limit", "0.17652 N·m at rated 12 V"],
        ["Loss/dead zone", "0.06 A modeled as Coulomb loss; dead zone unknown, assumed zero"],
    ]
    table = Table([[Paragraph(escape(str(c)), styles["cell"]) for c in r] for r in rows],
                  colWidths=[110, 330], repeatRows=1)
    table.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), BLUE),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTNAME", (0, 0), (-1, 0), "DejaVu-Bold"),
        ("BACKGROUND", (0, 1), (-1, -1), colors.HexColor("#F5F8FA")),
        ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#D8E4EA")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 6),
        ("RIGHTPADDING", (0, 0), (-1, -1), 6),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
    ]))
    story.append(table)
    story.append(Spacer(1, 4))
    p("The image lists rated speed and no-load speed both as 200 RPM. They cannot both represent loaded/no-load endpoints. The curve uses 200 RPM as no-load speed; its linear fit predicts about 142 RPM at the listed 0.52 kg-cm rated torque. At 80% PWM, predicted standstill torque is about 0.136 N·m per wheel and no-load speed about 153 RPM.")

    p("PID grid result", "head")
    p("Setpoint fixed at 0°; firmware-compatible 60–80% PWM; 45° fall threshold. Exhaustively tested the finite grid: Kp 0–30 by 0.5, Ki 0–1 by 0.05, Kd 0–20 by 0.5 (52,521 candidates). Trials were at most 2 s; top 30 were validated for up to 10 s.")
    p("No tested combination balanced. Every candidate fell before 2 s. Best-ranked: <b>Kp=26.5, Ki=0.35, Kd=18.5</b>; it fell at 0.35 s in validation. Peak absolute pitch 47.38°, pitch span 50.42°, mean absolute PWM 61.22%, minimum contacts 0, final wheel speed 14.15 rad/s (about 135 RPM). This is a best-ranked failed run, not a recommended tune.")
    p("Interpretation and limits", "head")
    p("The sweep diagnoses only this provisional simulation. Wheel signs (+1, −1) are assumed, and the motor curve is inferred rather than measured. The CAD-derived inertia, contact, reset transient and firmware-style minimum PWM remain important limitations. Do not transfer the gains or inferred constants to hardware. The image's specs do not prove the motors installed on the robot are this exact model.")
    p("Data: experiments/mujoco_pid_grid/20261013_n20_assumption_full_firmware_grid/. Editable report: docs/MUJOCO_N20_ASSUMED_PID_REPORT.md. Checks: 15 tests + 6 subtests and verify_balance_io passed; software plumbing only.")

    cmd = (".venv/bin/python -m simulation.run_balance --tune "
           "--config configs/mujoco_balance_n20_assumption.yaml "
           "--pwm-output-mode firmware_clamped")
    p(escape(cmd), "code")
    p("This command opens the interactive simulation with slider tuning; it is not evidence of physical balancing.")

    doc = SimpleDocTemplate(str(OUTPUT), pagesize=A4, leftMargin=40, rightMargin=40,
                            topMargin=38, bottomMargin=44,
                            title="Wall-E assumed N20 motor model and PID sweep",
                            author="Wall-E RL project")
    doc.build(story, onFirstPage=footer, onLaterPages=footer)
    print(OUTPUT)


if __name__ == "__main__":
    build()
