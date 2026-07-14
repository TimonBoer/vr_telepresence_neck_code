# Tkinter gui with four sliders that control the four base motors.
# This is to see at which values the ropes are taut, for calibration
# These values are the neutral motor angles used in the arduino_parallel node


#from controllers import BaseController, DotController, EmuController
import serial
from serial.serialutil import to_bytes
from tkinter import *
import threading

master = Tk()

ori_frame = Frame()
LR_frame = Frame()
FB_frame = Frame()

pitch = IntVar(value=90)
roll  = IntVar(value=90)
yaw   = IntVar(value=90)

# Wire them up at widget creation:
yaw_label   = Label(master=ori_frame, textvariable=yaw)
pitch_label = Label(master=LR_frame,  textvariable=pitch)
roll_label  = Label(master=FB_frame,  textvariable=roll)

yaw_label.grid(row=0, column=0)
ori_frame.pack()

default = 125
# LR_frame
left_var = DoubleVar(value=default)
right_var = DoubleVar(value=default)
left = Scale(master=LR_frame, from_=0, to=180, orient=HORIZONTAL, resolution=1, length=300, variable=left_var)
right = Scale(master=LR_frame, from_=0, to=180, orient=HORIZONTAL, resolution=1, length=300, variable=right_var)

left.grid(row=0, column=0)
pitch_label.grid(row=0, column=1)
right.grid(row=0, column=2)
LR_frame.pack()

# FB_frame
front_var = DoubleVar(value=default)
back_var = DoubleVar(value=default)
front = Scale(master=FB_frame, from_=0, to=180, orient=HORIZONTAL, resolution=1, length=300, variable=front_var)
back = Scale(master=FB_frame, from_=0, to=180, orient=HORIZONTAL, resolution=1, length=300, variable=back_var)

front.grid(row=0, column=0)
roll_label.grid(row=0, column=1)
back.grid(row=0, column=2)
FB_frame.pack()

yaw_slider = Scale(from_=-90, to=90, orient=HORIZONTAL, resolution=1, length=300)
yaw_slider.pack()

# --- Thread-safe shared state ---
latest_data = {
    "pitch": 90,
    "roll": 90,
    "yaw": 90,
}
data_lock = threading.Lock()

# class Project(DotController):
#     def __init__(self, sensor_ids):
#         super().__init__(sensor_ids, 'ori', False)
#         self.processed_data = {sensor_id: [] for sensor_id in self.sensor_ids}

#     def process_data(self, sensor_id, timestamp, ori, acc, gyr, queue_size):
#         # DON'T touch Tkinter here — just update the shared dict
#         with data_lock:
#             latest_data["pitch"] = int(ori[0])
#             latest_data["roll"] = int(ori[1])
#             latest_data["yaw"]  = int(ori[2])

#     @BaseController.getter
#     def get_processed_data(self):
#         return self.processed_data

def poll_sensor_data():
    with data_lock:
        pitch.set(latest_data["pitch"])
        roll.set(latest_data["roll"])
        yaw.set(latest_data["yaw"])

    # arduino expects: left, right, front, back
    motor_angles = [int(left_var.get()), int(right_var.get()), int(front_var.get()), int(back_var.get())]
    motor_angles.append(int(yaw_slider.get()) + 90)
    motor_angles.append(181)
    arduino.write(to_bytes(motor_angles))

    master.after(20, poll_sensor_data)

arduino = serial.Serial(port='COM7', baudrate=115200, timeout=.1)

#controller = Project(["DOT-01"])
#controller.start()

poll_sensor_data()   # kick off the polling loop
master.mainloop()

#controller.stop()

# left: 125  0
# right: 125     pi
# front: 125    pi/2
# back: 125     -pi/2

# yaw: -30