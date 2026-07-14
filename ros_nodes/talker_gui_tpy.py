# Ros node for debugging and controlled environment accuracy data collection
# Uses three sliders for tilt pan and yaw, converts that to quaternion, and sends it to the robot through the orientation topic

# Additionally it can save the current setpoint (robot input) and imu orientation (robot output) to csv for accuracy data collection

#!/usr/bin/env python3
import tkinter as tk
import math
import csv
import os
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped
from scipy.spatial.transform import Rotation
import numpy as np


CSV_FILE = "src/vr_bridge/orientation_log.csv"
CSV_HEADER = [
    "timestamp",
    "setpoint_tilt_deg", "setpoint_pan_deg", "setpoint_yaw_deg",
    "setpoint_qx", "setpoint_qy", "setpoint_qz", "setpoint_qw",
    "imu_qx", "imu_qy", "imu_qz", "imu_qw",
]


class SliderPublisher(Node):
    def __init__(self):
        super().__init__('talker_quaternion')

        self.pub = self.create_publisher(QuaternionStamped, 'orientation', 10)

        # Latest IMU quaternion received from imu/orientation
        self._imu_lock = threading.Lock()
        self._imu_q = None  # (x, y, z, w) or None if not yet received

        self.create_subscription(
            QuaternionStamped,
            'imu/orientation',
            self._imu_callback,
            10,
        )

    # ------------------------------------------------------------------
    # IMU subscriber
    # ------------------------------------------------------------------
    def _imu_callback(self, msg: QuaternionStamped):
        with self._imu_lock:
            self._imu_q = (
                msg.quaternion.x,
                msg.quaternion.y,
                msg.quaternion.z,
                msg.quaternion.w,
            )

    def get_imu_quaternion(self):
        """Return the latest IMU quaternion (thread-safe), or None."""
        with self._imu_lock:
            return self._imu_q

    # ------------------------------------------------------------------
    # Publisher
    # ------------------------------------------------------------------
    def publish_rpy(self, tilt_deg, pan_deg, yaw_deg):
        t = math.radians(tilt_deg)
        p = math.radians(pan_deg)
        y = math.radians(yaw_deg)
        q = self.tilt_pan_yaw_to_quaternion(t, p, y)

        msg = QuaternionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.quaternion.x = q[0]
        msg.quaternion.y = q[1]
        msg.quaternion.z = q[2]
        msg.quaternion.w = q[3]
        self.pub.publish(msg)

        self.get_logger().info(
            f'RPY: [{tilt_deg:.1f}, {pan_deg:.1f}, {yaw_deg:.1f}] → '
            f'q: [w={msg.quaternion.w:.4f}, x={msg.quaternion.x:.4f}, '
            f'y={msg.quaternion.y:.4f}, z={msg.quaternion.z:.4f}]'
        )
        return q  # (x, y, z, w)

    def tilt_pan_yaw_to_quaternion(self, tilt: float, pan: float, yaw: float) -> np.ndarray:
        r_pan  = Rotation.from_euler('z', pan)
        r_tilt = Rotation.from_euler('x', tilt)
        r_yaw  = Rotation.from_euler('z', yaw - pan)
        r = r_pan * r_tilt * r_yaw
        return r.as_quat()  # [x, y, z, w]


# ----------------------------------------------------------------------
# CSV helper
# ----------------------------------------------------------------------
def save_to_csv(tilt_deg, pan_deg, yaw_deg, setpoint_q, imu_q, status_var):
    """Append one row to the CSV file and update the status label."""
    file_exists = os.path.isfile(CSV_FILE)

    imu_row = list(imu_q) if imu_q is not None else ["N/A"] * 4

    row = [
        datetime.now().isoformat(timespec='milliseconds'),
        f"{tilt_deg:.1f}", f"{pan_deg:.1f}", f"{yaw_deg:.1f}",
        f"{setpoint_q[0]:.6f}", f"{setpoint_q[1]:.6f}",
        f"{setpoint_q[2]:.6f}", f"{setpoint_q[3]:.6f}",
        *(f"{v:.6f}" if v != "N/A" else "N/A" for v in imu_row),
    ]

    with open(CSV_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(CSV_HEADER)
        writer.writerow(row)

    imu_status = "saved" if imu_q is not None else "saved (no IMU data yet)"
    status_var.set(f"✓ Row {imu_status}  →  {CSV_FILE}")


# ----------------------------------------------------------------------
# Main / GUI
# ----------------------------------------------------------------------
def main():
    rclpy.init()
    node = SliderPublisher()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    root.title("Quaternion Publisher")
    root.resizable(False, False)

    # Status label var declared early so slider/entry callbacks can reference it
    status_var = tk.StringVar(value="")

    sliders = {}
    _entry_updating = [False]  # guard to prevent recursive sync loops

    for i, axis in enumerate(['tilt', 'pan', 'Yaw']):
        tk.Label(root, text=axis, width=6, anchor='w').grid(
            row=i, column=0, padx=10, pady=8)

        var = tk.DoubleVar(value=0.0)
        entry_var = tk.StringVar(value="0")

        scale = tk.Scale(root, from_=-180, to=180, resolution=1,
                         orient=tk.HORIZONTAL, length=280, variable=var)
        scale.grid(row=i, column=1, padx=10)

        entry = tk.Entry(root, textvariable=entry_var, width=7,
                         justify='center')
        entry.grid(row=i, column=2, padx=(0, 10))

        sliders[axis] = (var, entry_var)

        # --- sync slider → entry ---
        def _on_var_write(name, index, op, _var=var, _evar=entry_var):
            if _entry_updating[0]:
                return
            _entry_updating[0] = True
            _evar.set(f"{int(_var.get())}")
            _entry_updating[0] = False

        var.trace_add('write', _on_var_write)

        # --- sync entry → slider (on Enter or focus-out) ---
        def _apply_entry(event=None, _var=var, _evar=entry_var, _axis=axis):
            if _entry_updating[0]:
                return
            raw = _evar.get().strip()
            try:
                val = float(raw)
                val = max(-180.0, min(180.0, val))
                _entry_updating[0] = True
                _var.set(val)
                _evar.set(f"{int(val)}")
                _entry_updating[0] = False
                status_var.set("")
            except ValueError:
                status_var.set(f"⚠ Invalid value for {_axis}: '{raw}'")

        entry.bind('<Return>', _apply_entry)
        entry.bind('<FocusOut>', _apply_entry)

    # Keep the last published setpoint quaternion so Save can use it
    # without re-publishing.
    last_setpoint_q = [None]

    def on_slider_change():
        r = sliders['tilt'][0].get()
        p = sliders['pan'][0].get()
        y = sliders['Yaw'][0].get()
        q = node.publish_rpy(r, p, y)
        last_setpoint_q[0] = q
        status_var.set("")

    for axis, (var, _) in sliders.items():
        var.trace_add('write', lambda *_: on_slider_change())

    def on_save():
        tilt = sliders['tilt'][0].get()
        pan  = sliders['pan'][0].get()
        yaw  = sliders['Yaw'][0].get()

        # Use last published setpoint q; compute it fresh if Publish was never clicked
        sq = last_setpoint_q[0]
        if sq is None:
            sq = node.tilt_pan_yaw_to_quaternion(
                math.radians(tilt), math.radians(pan), math.radians(yaw))

        imu_q = node.get_imu_quaternion()
        save_to_csv(tilt, pan, yaw, sq, imu_q, status_var)

    # --- Buttons row ---
    btn_frame = tk.Frame(root)
    btn_frame.grid(row=3, column=0, columnspan=3, pady=12)

    tk.Button(btn_frame, text="Publish", command=on_slider_change,
              bg='#4CAF50', fg='white', padx=20).pack(side=tk.LEFT, padx=8)

    tk.Button(btn_frame, text="Save to CSV", command=on_save,
              bg='#2196F3', fg='white', padx=20).pack(side=tk.LEFT, padx=8)

    # --- Status label ---
    tk.Label(root, textvariable=status_var, fg='#555', font=('TkDefaultFont', 9),
             anchor='w').grid(row=4, column=0, columnspan=3, padx=12, pady=(0, 8), sticky='w')

    root.protocol("WM_DELETE_WINDOW", lambda: (rclpy.shutdown(), root.destroy()))
    root.mainloop()


if __name__ == '__main__':
    main()