"""Build the concise PDF report for the 2026-10-13 PID grid experiment."""

from pathlib import Path
from xml.sax.saxutils import escape

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import Paragraph, SimpleDocTemplate, Spacer, Table, TableStyle


HERE = Path(__file__).resolve().parent.parent
OUTPUT = HERE / "MUJOCO_PID_GRID_REPORT.pdf"
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
        "title": ParagraphStyle("title", fontName="DejaVu-Bold", fontSize=20,
                                 leading=25, textColor=BLUE, spaceAfter=4),
        "subtitle": ParagraphStyle("subtitle", fontName="DejaVu", fontSize=9,
                                    leading=13, textColor=GRAY, spaceAfter=13),
        "heading": ParagraphStyle("heading", fontName="DejaVu-Bold", fontSize=11,
                                   leading=15, textColor=BLUE, spaceBefore=10, spaceAfter=5),
        "body": ParagraphStyle("body", fontName="DejaVu", fontSize=8.7,
                                leading=12.5, textColor=INK, spaceAfter=6,
                                alignment=TA_LEFT),
        "code": ParagraphStyle("code", fontName="DejaVuMono", fontSize=7,
                                leading=10, backColor=colors.HexColor("#EFF4F7"),
                                borderPadding=7, spaceBefore=3, spaceAfter=8),
        "cell": ParagraphStyle("cell", fontName="DejaVu", fontSize=7.8,
                                leading=10, textColor=INK),
    }
    story = []

    def p(text: str, style: str = "body") -> None:
        story.append(Paragraph(text, styles[style]))

    def footer(canvas, doc) -> None:
        canvas.saveState()
        canvas.setStrokeColor(colors.HexColor("#D8E4EA"))
        canvas.line(38, 34, A4[0] - 38, 34)
        canvas.setFont("DejaVu", 7.5)
        canvas.setFillColor(GRAY)
        canvas.drawString(38, 22, "WALL-E RL  /  Simulation experiment  /  13 Oct 2026")
        canvas.drawRightString(A4[0] - 38, 22, str(doc.page))
        canvas.restoreState()

    p("PID grid sweep", "title")
    p("Wall-E balance controller • MuJoCo 3.7.0 • 13 October 2026", "subtitle")
    p("Bottom line", "heading")
    p("No tested gain combination balanced the current simulated model. All 52,521 candidates fell in the initial trials. The top-ranked candidate failed again in longer validation, so there is no winning PID setting to recommend.")

    p("Search definition", "heading")
    data = [
        ["Parameter", "Values tested", "Count"],
        ["Kp", "0 to 30, step 0.5", "61"],
        ["Ki", "0 to 1, step 0.05", "21"],
        ["Kd", "0 to 20, step 0.5", "41"],
        ["Total", "61 × 21 × 41", "52,521"],
    ]
    table = Table([[Paragraph(str(cell), styles["cell"]) for cell in row]
                   for row in data], colWidths=[78, 300, 65], repeatRows=1)
    table.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), BLUE),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTNAME", (0, 0), (-1, 0), "DejaVu-Bold"),
        ("BACKGROUND", (0, 1), (-1, -1), colors.HexColor("#F5F8FA")),
        ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#D8E4EA")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 7),
        ("RIGHTPADDING", (0, 0), (-1, -1), 7),
        ("TOPPADDING", (0, 0), (-1, -1), 6),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
    ]))
    story.append(table)
    story.append(Spacer(1, 5))
    p("Setpoint fixed at 0° (the configured reset-pose zero); firmware-clamped PWM; 2 s maximum initial trial; 30 top-ranked candidates rerun for up to 10 s. A fall was total chassis tilt reaching 45°. This exhausts the stated grid only—not continuous gain values or all possible setpoints.")

    p("Best-ranked point (not a recommendation)", "heading")
    p("Kp = 19.5, Ki = 0.75, Kd = 20.0. Survival was 0.20 s in both the initial run and 10 s validation. Validation stopped after falling: peak absolute pitch 48.91°, pitch span 48.65°, true-pitch RMS error 23.59°, and mean absolute PWM 71%.")
    p("What this means", "heading")
    p("This diagnoses the current simulated plant/controller setup, not the real robot: gains cannot compensate for incorrect actuator polarity, insufficient or unrealistic torque, bad contact, or incorrect dynamics. The sweep used an artificial constant 0.02 N·m full-scale torque and assumed wheel signs left +1/right −1. Neither is a measured motor model or verified hardware direction. The 60% minimum PWM clamp can also make small nonzero commands jump to substantial effort.")
    p("Do not flash these top-ranked values as a hardware solution. First inspect the saved telemetry and verify the restoring command direction, wheel/floor contact, and measured motor response; then rerun tuning against a validated plant.")

    p("Artifacts and reproducibility", "heading")
    p("Full data: experiments/mujoco_pid_grid/20261013T_full_firmware_grid/ (grid_results.csv, validation_results.csv, best_candidate_telemetry.csv, run_metadata.json). Editable report source: docs/MUJOCO_PID_GRID_REPORT.md.")
    cmd = (".venv/bin/python -m simulation.diagnostics.sweep_balance_pid_grid --kp-range 0 30 0.5 "
           "--ki-range 0 1 0.05 --kd-range 0 20 0.5 --trial-seconds 2 "
           "--validation-seconds 10 --validation-top 30 --setpoint-deg 0 "
           "--pwm-output-mode firmware_clamped --ideal-test-torque-nm 0.02 "
           "--left-sign 1 --right-sign -1 --output-dir "
           "experiments/mujoco_pid_grid/20261013T_full_firmware_grid")
    p(escape(cmd), "code")
    p("Verification after the sweep: 14 tests and 6 subtests passed; simulation.verify_balance_io passed. These checks validate software plumbing only, not physical balancing.", "body")

    doc = SimpleDocTemplate(str(OUTPUT), pagesize=A4, leftMargin=42, rightMargin=42,
                            topMargin=42, bottomMargin=47,
                            title="Wall-E MuJoCo PID grid sweep report",
                            author="Wall-E RL project")
    doc.build(story, onFirstPage=footer, onLaterPages=footer)
    print(OUTPUT)


if __name__ == "__main__":
    build()
