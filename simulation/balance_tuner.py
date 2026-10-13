"""Small Tk control panel for live MuJoCo pitch-PID tuning."""

from __future__ import annotations


class BalanceTuner:
    """A companion window; MuJoCo remains in its own native viewer window."""

    RANGES = {
        "setpoint_deg": (-90.0, 90.0, 0.1, "Pitch setpoint (deg)"),
        "kp": (0.0, 50.0, 0.1, "Kp"),
        "ki": (0.0, 10.0, 0.01, "Ki"),
        "kd": (0.0, 20.0, 0.1, "Kd"),
        "fall_angle_deg": (5.0, 85.0, 0.5, "Fall reset tilt (deg)"),
    }

    def __init__(self, setpoint_deg: float, kp: float, ki: float, kd: float,
                 fall_angle_deg: float, pwm_output_mode: str = "continuous"):
        import tkinter as tk

        self.tk = tk
        self.root = tk.Tk()
        self.root.title("Wall-E PID Tuner")
        self.root.resizable(False, False)
        self.closed = False
        self.paused = False
        self.reset_requested = False
        self.variables = {}
        initial = {"setpoint_deg": setpoint_deg, "kp": kp, "ki": ki, "kd": kd,
                   "fall_angle_deg": fall_angle_deg}

        tk.Label(self.root, text="Live MuJoCo balance tuning", font=("TkDefaultFont", 12, "bold"))\
            .grid(row=0, column=0, columnspan=2, padx=12, pady=(12, 4), sticky="w")
        mode_text = ("Continuous motor effort (simulation-only tuning mode)"
                     if pwm_output_mode == "continuous"
                     else "Firmware-style 60–80% PWM clamp")
        tk.Label(self.root, text=f"{mode_text}; changes apply on the next controller tick.")\
            .grid(row=1, column=0, columnspan=2, padx=12, pady=(0, 8), sticky="w")

        for row, (key, (minimum, maximum, resolution, label)) in enumerate(self.RANGES.items(), start=2):
            tk.Label(self.root, text=label, width=24, anchor="w")\
                .grid(row=row, column=0, padx=(12, 4), pady=3, sticky="w")
            variable = tk.DoubleVar(value=initial[key])
            self.variables[key] = variable
            tk.Scale(
                self.root, variable=variable, from_=minimum, to=maximum,
                resolution=resolution, orient=tk.HORIZONTAL, length=310,
                showvalue=True, digits=6,
            ).grid(row=row, column=1, padx=(4, 12), pady=3, sticky="ew")

        self.status = tk.StringVar(value="Running — tuning is live; falls reset this episode")
        tk.Label(self.root, textvariable=self.status, anchor="w")\
            .grid(row=6, column=0, columnspan=2, padx=12, pady=(8, 4), sticky="ew")

        buttons = tk.Frame(self.root)
        buttons.grid(row=7, column=0, columnspan=2, padx=12, pady=(4, 12), sticky="ew")
        self.pause_button = tk.Button(buttons, text="Pause", command=self.toggle_pause)
        self.pause_button.pack(side=tk.LEFT)
        tk.Button(buttons, text="Reset pose", command=self.request_reset).pack(side=tk.LEFT, padx=8)
        tk.Label(buttons, text="Close this panel to stop the run").pack(side=tk.RIGHT)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        self.pause_button.configure(text="Resume" if self.paused else "Pause")
        self.status.set("Paused" if self.paused else "Running — tuning is live; falls reset this episode")

    def request_reset(self) -> None:
        self.reset_requested = True

    def consume_reset_request(self) -> bool:
        requested = self.reset_requested
        self.reset_requested = False
        return requested

    def parameters(self) -> dict[str, float]:
        return {name: float(variable.get()) for name, variable in self.variables.items()}

    def pump(self) -> None:
        if self.closed:
            return
        try:
            self.root.update_idletasks()
            self.root.update()
        except self.tk.TclError:
            self.closed = True

    def close(self) -> None:
        self.closed = True
        try:
            self.root.destroy()
        except self.tk.TclError:
            pass
