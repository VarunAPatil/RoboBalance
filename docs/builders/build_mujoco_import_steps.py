"""Build the two-page guide for importing the existing Wall-E URDF."""

from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, PageBreak, Spacer

HERE = Path(__file__).resolve().parent.parent
OUTPUT = HERE / "MUJOCO_IMPORT_STEPS.pdf"
BLUE = colors.HexColor("#175C78")


def build():
    fonts = Path("/usr/share/fonts/truetype/dejavu")
    for name, filename in [("DejaVu", "DejaVuSans.ttf"), ("DejaVu-Bold", "DejaVuSans-Bold.ttf"),
                           ("DejaVuMono", "DejaVuSansMono.ttf")]:
        pdfmetrics.registerFont(TTFont(name, str(fonts / filename)))
    pdfmetrics.registerFontFamily("DejaVu", normal="DejaVu", bold="DejaVu-Bold")
    styles = {
        "title": ParagraphStyle("title", fontName="DejaVu-Bold", fontSize=22, leading=28, textColor=BLUE, spaceAfter=13),
        "heading": ParagraphStyle("heading", fontName="DejaVu-Bold", fontSize=12, leading=17, textColor=BLUE, spaceBefore=12, spaceAfter=7),
        "body": ParagraphStyle("body", fontName="DejaVu", fontSize=10, leading=15, spaceAfter=8),
        "code": ParagraphStyle("code", fontName="DejaVuMono", fontSize=8.3, leading=13, backColor=colors.HexColor("#EFF4F7"), borderPadding=8, spaceBefore=5, spaceAfter=12),
        "small": ParagraphStyle("small", fontName="DejaVu", fontSize=8.5, leading=12, spaceAfter=6),
    }
    story = []

    def p(text, style="body"):
        story.append(Paragraph(text, styles[style]))

    def footer(canvas, doc):
        canvas.saveState()
        canvas.setFont("DejaVu", 8)
        canvas.setFillColor(BLUE)
        canvas.drawString(42, 25, "WALL-E RL • Existing URDF → MuJoCo • 10 October 2026")
        canvas.drawRightString(A4[0] - 42, 25, str(doc.page))
        canvas.restoreState()

    p("Bring Wall-E into MuJoCo", "title")
    p("Import steps for the existing Onshape export", "heading")
    p("<b>No new CAD export is required for the identified mass issues.</b> Use the existing URDF and mesh assets as the geometry source, then prepare corrected inertial properties and a MuJoCo-native model.")
    p("Verified installation and import status", "heading")
    p("On 10 October 2026, system Python (python3) successfully imported <b>MuJoCo 3.7.0</b> and mujoco.viewer. The project’s .venv could not import MuJoCo. Viewer-module availability was checked; a graphical viewer window was not tested.")
    p("Current source: <b>Assembly-Wall-E/assembly_1/urdf/assembly_1.urdf</b>. Its exported mass totals approximately <b>238.833 g</b>; this is not a verified physical robot mass.")
    p("Direct loading failed. A second in-memory import with visual meshes retained and fixed-body fusion enabled failed at the same component:")
    p("inertia must have positive eigenvalues<br/>Element: 0603_resistor_004_2", "code")
    p("The individual component inertias must be repaired or aggregated before compilation. Enabling fixed-body fusion alone does not bypass this validation error.")
    p("1. Prepare the corrected import", "heading")
    p("• Preserve the original export. Work on a derived copy and retain every mesh’s scale, orientation, and placement.")
    p("• Collapse fixed components into three moving bodies: chassis, left wheel, and right wheel. Preserve the two wheel hinge axes and joint relationships.")
    p("• Use the confirmed 153 g battery-and-holder group under the shared uniform-density assumption, 4.5 g per installed screw, 40 g per wheel, and 29 g per complete motor. Reconcile the populated-board total and remaining components.")
    p("• Recompute center of mass and inertia as well as mass. For combined bodies, transform component inertias into a common frame and apply the parallel-axis theorem. Do not retain the one-cell inertia for a three-cell mesh.")
    p("• Resolve package:// mesh references to available assets. Keep detailed meshes as visual geometry; add deliberate collision shapes later.")
    p("<b>Checkpoint:</b> valid aggregate inertias, correct geometry transforms, consistent units (kg, m, kg·m²), and a documented mass total. Check against the complete robot’s measured mass when supplied.")

    story.append(PageBreak())
    p("Load, view, then verify", "title")
    p("2. Save the prepared model as MJCF", "heading")
    p("MJCF is MuJoCo’s native XML format. A corrected URDF can be compiled with MjModel.from_xml_path and saved using mj_saveLastXML, or its geometry and corrected rigid-body properties can be written directly into MJCF.")
    p("Proposed output: <b>simulation/models/wall_e.xml</b>. This file has not been created yet. A first geometry preview can be kept static while physical parameters are completed; it is not a validated dynamic model.")
    p("3. Open the prepared model", "heading")
    p("From the project root, once wall_e.xml exists:")
    p("python3 -m mujoco.viewer --mjcf=simulation/models/wall_e.xml", "code")
    p("This uses the existing system-Python installation. Inspect scale, mesh alignment, body orientation, wheel spacing, and joint axes before relying on simulated motion.")
    p("4. Integrate MuJoCo into the project environment", "heading")
    p("For later use alongside the existing Gymnasium/PPO workflow, install the same verified version into .venv:")
    p('.venv/bin/python -m pip install "mujoco==3.7.0"', "code")
    p("Verify the installed version:")
    p('.venv/bin/python -c "import mujoco; print(mujoco.__version__)"', "code")
    p("These are instructions, not completed installation steps. Record the selected dependency version in the project configuration and experiment artifacts when integration is implemented.")
    p("5. Add and check physical behavior incrementally", "heading")
    p("After the required measurements are available, add a floating chassis, floor, simple wheel/chassis collision geometry, and measured motor behavior. Check passive falling, command polarity, contact, saturation, and timestep convergence before adding PPO.")
    p("The existing export has no collision definitions. A successful mesh preview does not establish correct ground contact, motor response, or physical balancing. Keep wheel position/speed out of deployable policy inputs unless actually measured or reliably estimated; the TT motors have no encoders.")
    p("Official references", "heading")
    p('<link href="https://mujoco.readthedocs.io/en/stable/python.html#standalone-app" color="#175C78">MuJoCo Python: installation and standalone viewer</link>', "small")
    p('<link href="https://mujoco.readthedocs.io/en/latest/modeling.html#urdf-extensions" color="#175C78">MuJoCo modeling: URDF import and MJCF conversion</link>', "small")
    p("Related project guide: docs/MUJOCO_STEPS.pdf. Rebuild this PDF with python3 docs/builders/build_mujoco_import_steps.py.", "small")
    doc = SimpleDocTemplate(str(OUTPUT), pagesize=A4, leftMargin=42, rightMargin=42,
                            topMargin=40, bottomMargin=45, title="Wall-E: URDF to MuJoCo import steps",
                            author="Wall-E RL project")
    doc.build(story, onFirstPage=footer, onLaterPages=footer)
    print(OUTPUT)


if __name__ == "__main__":
    build()
