"""Generate the four-page Onshape-to-MuJoCo workflow PDF."""

from pathlib import Path
from xml.sax.saxutils import escape

from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, PageBreak, Table, TableStyle

HERE = Path(__file__).resolve().parent.parent
OUTPUT = HERE / "MUJOCO_STEPS.pdf"
BLUE = colors.HexColor("#175C78")
INK = colors.HexColor("#24313B")


def build():
    fonts = Path("/usr/share/fonts/truetype/dejavu")
    for name, file in [("DejaVu", "DejaVuSans.ttf"), ("DejaVu-Bold", "DejaVuSans-Bold.ttf")]:
        pdfmetrics.registerFont(TTFont(name, str(fonts / file)))
    pdfmetrics.registerFontFamily("DejaVu", normal="DejaVu", bold="DejaVu-Bold")
    styles = {
        "title": ParagraphStyle("title", fontName="DejaVu-Bold", fontSize=23, leading=29, textColor=BLUE, spaceAfter=15),
        "heading": ParagraphStyle("heading", fontName="DejaVu-Bold", fontSize=13, leading=18, textColor=BLUE, spaceBefore=12, spaceAfter=8),
        "body": ParagraphStyle("body", fontName="DejaVu", fontSize=10, leading=15, textColor=INK, spaceAfter=8),
        "small": ParagraphStyle("small", fontName="DejaVu", fontSize=8.5, leading=12, textColor=INK, spaceAfter=6),
    }
    story = []

    def p(text, style="body"):
        story.append(Paragraph(text, styles[style]))

    def step(text):
        p("• " + text)

    def checkpoint(text):
        p("<b>Checkpoint:</b> " + text)

    def page(title):
        story.append(PageBreak())
        p(title, "title")

    def footer(canvas, doc):
        canvas.saveState()
        canvas.setFont("DejaVu", 8)
        canvas.setFillColor(BLUE)
        canvas.drawString(42, 26, "WALL-E RL • Onshape → MuJoCo • 10 October 2026")
        canvas.drawRightString(A4[0] - 42, 26, f"{doc.page} / 4")
        canvas.restoreState()

    p("MuJoCo steps", "title")
    p("A practical workflow for your actual Wall-E assembly", "heading")
    p("<b>First milestone:</b> the assembled robot appears in MuJoCo at the correct scale, with correctly rotating wheels, credible mass distribution, and plausible ground contact. Motor characterization and reinforcement learning follow after the mechanical checks pass.")
    p("STEP files supply geometry. A moving simulation also needs body relationships, joint axes, masses, inertia, contacts, and actuators. A correct-looking animation is not evidence of accurate physical behavior.")
    p("1. Prepare the Onshape assembly", "heading")
    step("Place components exactly as installed, including batteries, PCB, motor housings, brackets, and fasteners. Confirm the battery location against the physical robot.")
    step("Treat everything rigidly attached to the chassis as one moving body. Keep the left and right rotating wheel assemblies as two separate bodies. Include hubs or rotating shaft attachments where relevant.")
    step("Use fastened mates for rigid attachments and one revolute mate per wheel, aligned with the actual motor shaft. The initial model can represent gearbox effects without simulating internal gears.")
    p("<b>Body structure:</b> floating chassis → left wheel hinge and right wheel hinge. The chassis must ultimately be free to translate and rotate relative to the ground.")
    step("Choose X forward, Y left, Z up. Use the midpoint between wheel centers as the robot reference origin. Record the MPU6050 location and mounting orientation in this frame.")
    checkpoint("Only the two intended wheel rotations remain within the assembly; the installed geometry and component positions match the physical robot.")
    p("Current hardware context", "heading")
    p("ESP32-S3-WROOM-1, MPU6050, and two TT motors without encoders are recorded. The schematic includes a TB6612FNG module; confirm its physical marking. The supplied component subtotal is 186 g, excluding batteries, screws, and unlisted parts. It is not the assembled mass.")

    page("Measure and export")
    p("2. Establish geometry and mass properties", "heading")
    rows = [["Parameter", "Obtain / check"]]
    entries = [
        ("Wheel radius and width", "CAD dimensions; verify real wheels and loaded rolling radius."),
        ("Wheel-center spacing", "CAD distance between wheel centers; physical cross-check."),
        ("Complete assembled mass", "Weigh the complete robot with installed batteries."),
        ("Chassis and wheel masses", "Separate chassis from rotating wheel assemblies; avoid double counting."),
        ("Center of mass", "CAD estimate using credible component masses; physical check with battery installed."),
        ("Inertia tensors", "CAD estimates for chassis and each rotating wheel, with their reference frames."),
        ("IMU mounting", "Position, orientation, and axis mapping relative to the chassis."),
    ]
    rows += [[Paragraph(escape(a), styles["small"]), Paragraph(escape(b), styles["small"])] for a, b in entries]
    table = Table(rows, colWidths=[155, 350], hAlign="LEFT")
    table.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), BLUE), ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTNAME", (0, 0), (-1, 0), "DejaVu-Bold"), ("FONTSIZE", (0, 0), (-1, 0), 9),
        ("VALIGN", (0, 0), (-1, -1), "TOP"), ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
        ("TOPPADDING", (0, 0), (-1, -1), 7), ("GRID", (0, 0), (-1, -1), .4, colors.HexColor("#D9E3E8")),
    ]))
    story.append(table)
    story.append(Spacer(1, 10))
    p("Inertia measures resistance to rotation. The same mass placed higher changes balancing dynamics. Assign appropriate materials and measured mass overrides in Onshape, then inspect Mass Properties. Simplified motors and batteries may still have approximate internal mass distributions.")
    p("Record kilograms, meters, and kg·m² in the model; record measurement uncertainty and distinguish CAD estimates from physical measurements. Inertias must be expressed about the appropriate center of mass and in a known frame.")
    p("3. Export the assembly", "heading")
    step("Preferred route: right-click the Onshape Assembly tab → Export → URDF, if available. Keep the complete package, including meshes. Save a version of the assembly for traceability.")
    step("Load and inspect the URDF in MuJoCo, then save it as MJCF, MuJoCo’s native XML format. Check joint axes, transforms, masses, and inertias; add simulation-specific settings in MJCF.")
    step("Fallback: export individual binary STL meshes in meters plus a full assembly STEP reference. Preserve or record part placements so the assembly can be reconstructed in MJCF.")
    checkpoint("Geometry has a known scale and reference frame; rotating wheels remain distinct; mass data and unknowns are recorded before physical-model implementation.")

    page("Build the first simulation")
    p("4. Create and check the MuJoCo scene", "heading")
    step("Use a free joint for the chassis and a hinge joint for each wheel. Define explicit mass, center of mass, and inertia for each moving body using the measured and documented parameters.")
    step("Add a flat floor. Use CAD meshes for appearance and simple collision shapes for the chassis and wheels. Fine details such as PCB traces generally do not need collision geometry.")
    step("Check that visual geometry does not accidentally add mass or unwanted contacts. Ensure wheel contacts and chassis ground clearance match the assembly.")
    step("Inspect the stationary model first: scale, wheel centers, shaft axes, mesh orientation, and initial placement. Avoid starting with wheels penetrating the floor.")
    step("With motor commands at zero, start from a small tilt and observe passive falling. Compare with safe physical observations where available; gearbox drag can affect the real zero-command behavior.")
    step("Compare trajectories using smaller physics timesteps. Large changes indicate unresolved numerical or contact issues. Record the timestep, solver settings, and model revision.")
    checkpoint("The model compiles; wheels rotate about the correct axes; contact and passive motion are plausible; timestep checks do not reveal major instability.")
    p("5. Add characterized motor behavior", "heading")
    p("A wheel torque command in MuJoCo is not automatically an ESP32 PWM command. Measure the motor/driver response before selecting realistic limits or a PWM-to-torque model.")
    step("Confirm driver marking, supply voltage range, motor polarity, command dead zone, left/right asymmetry, response delay, and speed/load behavior through supported tests.")
    step("Use credible torque limits and account for battery voltage, motor saturation, lag, gearbox losses, and tire friction. Unloaded speed alone does not establish available torque.")
    step("Test equal forward commands, equal reverse commands, and differential commands. Verify the physical command signs and expected robot motion.")
    checkpoint("Motor parameters have measurement evidence or clearly documented uncertainty; simple commanded responses behave plausibly against that evidence.")
    p("Keep development incremental", "heading")
    p("CAD preparation and physical measurements can proceed together. Do not substitute generic TT motor specifications or incomplete component totals for this robot’s measured parameters. Simulation results remain simulation evidence until checked against hardware.")

    page("Connect sensing and learning")
    p("6. Add sensors, a baseline, and the RL interface", "heading")
    step("Place virtual IMU sensing at the physical MPU6050 location. Model the pitch estimation pipeline, gyro bias/noise, sample timing, and delay based on measurements.")
    step("MuJoCo exposes wheel position and speed for diagnostics, but the physical robot has no encoders. Deployable policy inputs must use available measurements or a validated estimator.")
    step("Test a simple balancing controller as a diagnostic baseline after the physics checks pass. Compare pitch response, drift, control effort, and failure behavior.")
    step("Wrap the simulation in Gymnasium: reset the robot, apply a motor command, advance physics, return sensor-like observations and reward, and report fall/time-limit endings. Verify the interface with check_env.")
    step("Then train PPO with reproducible seeds, configurations, dependency versions, checkpoints, and independent evaluations. Add measured noise, latency, and parameter uncertainty; evaluate unseen conditions.")
    p("Connection to CartPole: observe → choose action → advance physics → receive reward. Here the observation and command mapping must reproduce what the robot can actually sense and execute. Keep training, simulation, deployment, and independent safety cutoffs separate.")
    checkpoint("The environment passes interface checks and physical sanity checks before robot PPO training begins.")
    p("Immediate handoff checklist", "heading")
    step("Complete the assembly structure and check the URDF export option.")
    step("Supply the exported package’s local path and retain the assembly STEP reference.")
    step("Supply assembled mass with batteries, wheel dimensions, and wheel-center spacing. Include screw/battery counts and any missing component masses.")
    step("Supply CAD center-of-mass and inertia estimates with units/reference frames, plus IMU mounting information. Mark values not yet measured as unknown.")
    p("Project files to maintain", "heading")
    p("configs/robot.yaml: measured parameters. docs/HARDWARE.md: confirmed facts and unknowns. simulation/models/: future MJCF and meshes. simulation/environments/: future Gymnasium wrapper. docs/PLAN.md and docs/PROJECT_MAP.md: milestone and layout changes.", "small")
    p("Official references", "heading")
    refs = [
        ("Onshape: exporting files (including Assembly URDF and STL options)", "https://cad.onshape.com/help/Content/File/exporting_files.htm"),
        ("Onshape: Mass Properties tool", "https://cad.onshape.com/help/Content/View/mass_properties_tool.htm"),
        ("MuJoCo: modeling and URDF-to-MJCF workflow", "https://mujoco.readthedocs.io/en/latest/modeling.html#urdf-extensions"),
        ("MuJoCo: XML model reference", "https://mujoco.readthedocs.io/en/latest/XMLreference.html"),
    ]
    for label, url in refs:
        p(f'<link href="{url}" color="#175C78">{escape(label)}</link>', "small")
    doc = SimpleDocTemplate(str(OUTPUT), pagesize=A4, rightMargin=42, leftMargin=42,
                            topMargin=40, bottomMargin=45, title="MuJoCo steps — Wall-E assembly workflow",
                            author="Wall-E RL project")
    doc.build(story, onFirstPage=footer, onLaterPages=footer)
    print(OUTPUT)


if __name__ == "__main__":
    build()
