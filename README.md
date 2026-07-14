# VR Telepresence Neck Code

The main ros node for the control (`ros_nodes/arduino_parallel_v2.cpp`) was already sent by Direk, I have added some comments before each class and have added a class overview (see `ros_nodes/arduino_parallel_classes_overview.png`). 

Additionally some ros nodes and python scripts have been added that were used for debugging, data collection and calibration, along with the code for the arduino. 

## Arduino code
- **arduino_parallel_prototype** — Simple Arduino sketch, receives 5 bytes from the `arduino_parallel_v2.cpp` node, sends that to the 5 servos.

## ros_nodes

### Control
- **arduino_parallel_v2.cpp** — Main control node. Converts the QuaternionStamped messages from the topic `orientation` to motor commands and sends it to the arduino.
See `arduino_parallel_classes_overview.png` for the class overview.

### Controlled environment accuracy data collection
- **imu_talker.py** — Publishes head IMU data to `imu/orientation` topic, for accuracy data collection.

The imu can be calibrated using the command:
``` bash
ros2 topic pub --once imu/calibrate std_msgs/msg/Empty "{}"
```
- **imu_to_tilt_pan_yaw.cpp** — Logs the tilt, pan and yaw angles of the head IMU from the `imu/orientation` topic to the console, for debugging during data collection.
- **talker_gui_tpy.py** — GUI with 3 sliders (one per DOF), converts that to quaternion and publishes to the `orientation` topic, used for control debugging. Additionally it can save the current head IMU orientation from the topic `imu/orientation` together with the input orientation published on the topic `orientation` to a csv, for accuracy data collection.

## Calibration / parameterising python scripts
- **parallel_parameterising.py** — calibration script, using this you can find the neutral motor angles used by `arduino_parallel_v2.cpp`.
- **math_and_parameterising.py** — For debugging the closed-form solution and creating additional tightness keyframes at x=-15 and x=25 used to fine tune the tightness at specific tilt angles.
