# Tkinter gui with tilt pan yaw, and additional sliders for overall tightness and additional tightnesses at x=-15 and x=25
# These 6 sliders get converted to motor commands, which are displayed at the bottom and sent to an arduino if connected

# This program is used to create the additional tightness keyframes that get added on top of the closed-form control solution

import threading
from math import cos, sqrt, atan2, pi, acos

import serial
from serial.serialutil import to_bytes

from scipy.spatial.transform import Rotation

from tkinter import *

# ── Neck geometry (millimetres) ──────────────────────────────────────────────
VERTEBRA_RADIUS       = 52
VERTEBRA_HEIGHT       = 7
CENTER_RADIUS         = 5
ROPE_OFFSET           = 20
N_VERTEBRAE           = 6
SPINDLE_RADIUS        = 20

NEUTRAL_ROPE_LENGTH   = VERTEBRA_HEIGHT * 2 * N_VERTEBRAE

TIGHTNESS_ALL = 0

# ── Tightness keyframes ───────────────────────────────────────────────────────
# Each entry is (tilt_degrees, tightness_at_x_neg15, tightness_at_x_pos25)
# Must be sorted by tilt ascending.
# These keyframes can later be inserted into the arduino_parallel_v2 node to have the same function as here
TIGHTNESS_KEYFRAMES = [
    ( 0,  0,  0),
]

def interpolate_keyframes(tilt_deg):
    """Return (tightness_neg15, tightness_pos25) interpolated from keyframes."""
    if tilt_deg <= TIGHTNESS_KEYFRAMES[0][0]:
        return TIGHTNESS_KEYFRAMES[0][1], TIGHTNESS_KEYFRAMES[0][2]
    if tilt_deg >= TIGHTNESS_KEYFRAMES[-1][0]:
        return TIGHTNESS_KEYFRAMES[-1][1], TIGHTNESS_KEYFRAMES[-1][2]

    for i in range(len(TIGHTNESS_KEYFRAMES) - 1):
        t0, n0, p0 = TIGHTNESS_KEYFRAMES[i]
        t1, n1, p1 = TIGHTNESS_KEYFRAMES[i + 1]
        if t0 <= tilt_deg <= t1:
            t = (tilt_deg - t0) / (t1 - t0)
            return n0 + t * (n1 - n0), p0 + t * (p1 - p0)

def tightness_at_x(x, tightness_neg15, tightness_pos25):
    """Linearly interpolate tightness between x=-15 and x=25."""
    t = (x - (-15)) / (25 - (-15))
    t = max(0.0, min(1.0, t))
    return tightness_neg15 + t * (tightness_pos25 - tightness_neg15)

master = Tk()

def on_slider_change(_event=None):
    tilt_deg  = tilt_slider.get()
    tilt_rad  = tilt_deg * pi / 180
    pan_rad   = pan_slider.get() * pi / 180
    tightness = tightness_slider.get()

    kf_neg15, kf_pos25 = interpolate_keyframes(tilt_deg)
    tightness_neg15 = kf_neg15 + tightness_neg15_slider.get()
    tightness_pos25 = kf_pos25 + tightness_pos25_slider.get()

    tightness_neg15_label.config(text=f"Tightness at x=-15:  {tightness_neg15:.1f}  (keyframe: {kf_neg15:.1f})")
    tightness_pos25_label.config(text=f"Tightness at x= 25:  {tightness_pos25:.1f}  (keyframe: {kf_pos25:.1f})")

    for rope, base in zip(ropes, ROPE_NEUTRAL_BASES):
        rope.motor_neutral_angle = base + tightness + tightness_at_x(rope.x, tightness_neg15, tightness_pos25)
        rope.update(tilt_angle=tilt_rad, pan_angle=pan_rad)

    send_motor_angles(ropes)

Label(master, text="Tilt").pack()
tilt_slider = Scale(master, from_=0, to=90, orient=HORIZONTAL, resolution=1, length=300, command=on_slider_change)
tilt_slider.pack()

Label(master, text="Pan").pack()
pan_slider = Scale(master, from_=-180, to=180, orient=HORIZONTAL, resolution=1, length=300, command=on_slider_change)
pan_slider.pack()

Label(master, text="Yaw").pack()
yaw_slider = Scale(master, from_=-90, to=90, orient=HORIZONTAL, resolution=1, length=300, command=on_slider_change)
yaw_slider.pack()

Label(master, text="Tightness (all)").pack()
tightness_slider = Scale(master, from_=0, to=60, orient=HORIZONTAL, resolution=1, length=300, command=on_slider_change)
tightness_slider.set(TIGHTNESS_ALL)
tightness_slider.pack()

Label(master, text="Tightness offset at x=-15").pack()
tightness_neg15_slider = Scale(master, from_=-50, to=50, orient=HORIZONTAL, resolution=1, length=300, command=on_slider_change)
tightness_neg15_slider.set(0)
tightness_neg15_slider.pack()

Label(master, text="Tightness offset at x=25").pack()
tightness_pos25_slider = Scale(master, from_=-10, to=50, orient=HORIZONTAL, resolution=1, length=300, command=on_slider_change)
tightness_pos25_slider.set(0)
tightness_pos25_slider.pack()

tightness_neg15_label = Label(master, text="Tightness at x=-15:  0.0  (keyframe: 0.0)")
tightness_neg15_label.pack()
tightness_pos25_label = Label(master, text="Tightness at x= 25:  0.0  (keyframe: 0.0)")
tightness_pos25_label.pack()

# ── Motor table ───────────────────────────────────────────────────────────────
table_frame = Frame(master)
table_frame.pack(pady=8)

COLUMNS = ["Rope", "x", "Angle offset", "Motor angle"]
for col, header in enumerate(COLUMNS):
    Label(table_frame, text=header, font=("TkDefaultFont", 10, "bold"), width=14, relief=RIDGE, padx=4).grid(row=0, column=col)

ROPE_NAMES = ["Left", "Right", "Front", "Back"]
motor_cells = []

for i, name in enumerate(ROPE_NAMES):
    row = i + 1
    Label(table_frame, text=name, width=14, relief=RIDGE, padx=4).grid(row=row, column=0)
    x_lbl      = Label(table_frame, text="---", width=14, relief=RIDGE, padx=4)
    offset_lbl = Label(table_frame, text="---", width=14, relief=RIDGE, padx=4)
    angle_lbl  = Label(table_frame, text="---", width=14, relief=RIDGE, padx=4)
    x_lbl.grid(row=row, column=1)
    offset_lbl.grid(row=row, column=2)
    angle_lbl.grid(row=row, column=3)
    motor_cells.append((x_lbl, offset_lbl, angle_lbl))

class Rope:
    """Models a single drive rope and the motor that tensions it."""

    def __init__(self, angular_pos: float, motor_neutral_angle: int):
        self.angular_pos          = angular_pos
        self.motor_neutral_angle  = motor_neutral_angle
        self.angle_from_curve_centre = 0.0
        self.dist_from_curve_centre  = 0.0
        self.rope_length            = 0.0
        self.angle_offset           = 0.0
        self.motor_angle            = motor_neutral_angle
        self.x = 0.0
        self.update(tilt_angle=0.0, pan_angle=0.0)

    def update(self, tilt_angle: float, pan_angle: float) -> None:
        self._update_rope_geometry(pan_angle)
        self._update_rope_length(tilt_angle)
        self._update_motor_angle()

    def get_motor_angle(self) -> float:
        return self.motor_angle

    def _update_rope_geometry(self, pan_angle: float) -> None:
        self.x = ROPE_OFFSET * cos(pan_angle + self.angular_pos) + CENTER_RADIUS
        y = VERTEBRA_RADIUS - VERTEBRA_HEIGHT
        self.dist_from_curve_centre  = sqrt(self.x ** 2 + y ** 2)
        self.angle_from_curve_centre = -atan2(self.x, y)

    def _update_rope_length(self, tilt_angle: float) -> None:
        tilt_per_vertebra       = tilt_angle / N_VERTEBRAE / 2
        projected_offset        = self.dist_from_curve_centre * cos(self.angle_from_curve_centre - tilt_per_vertebra)
        length_per_vertebra     = VERTEBRA_RADIUS - projected_offset
        self.rope_length        = length_per_vertebra * 2 * N_VERTEBRAE

    def _update_motor_angle(self) -> None:
        length_delta        = NEUTRAL_ROPE_LENGTH - self.rope_length
        angle_offset_rad    = length_delta / SPINDLE_RADIUS
        angle_offset_deg    = angle_offset_rad * 180 / pi
        self.angle_offset   = angle_offset_deg
        self.motor_angle    = int(self.motor_neutral_angle + angle_offset_deg)

def send_motor_angles(ropes):
    motor_angles = []
    for i, rope in enumerate(ropes):
        angle = rope.get_motor_angle()
        clamped = min(max(int(angle), 0), 180)
        motor_angles.append(clamped)
        x_lbl, offset_lbl, angle_lbl = motor_cells[i]
        x_lbl.config(text=int(rope.x))
        offset_lbl.config(text=int(rope.angle_offset))
        angle_lbl.config(text=angle)

    yaw_angle = int(yaw_slider.get()) + 90 - 30
    motor_angles.append(min(max(yaw_angle, 0), 180))
    motor_angles.append(181)

    if arduino_connected:
        arduino.write(to_bytes(motor_angles))

try:
    arduino = serial.Serial(port='COM7', baudrate=115200, timeout=.1)
    arduino_connected = True
except serial.serialutil.SerialException:
    arduino_connected = False
    print("Could not connect to Arduino on /dev/ttyACM0. Logging only")

ROPE_NEUTRAL_BASES = [125, 125, 125, 125]

ropes = [
    Rope(0,     ROPE_NEUTRAL_BASES[0] + TIGHTNESS_ALL),
    Rope(pi,    ROPE_NEUTRAL_BASES[1] + TIGHTNESS_ALL),
    Rope(pi/2,  ROPE_NEUTRAL_BASES[2] + TIGHTNESS_ALL),
    Rope(-pi/2, ROPE_NEUTRAL_BASES[3] + TIGHTNESS_ALL),
]

master.mainloop()