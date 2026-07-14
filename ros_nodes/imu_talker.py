# Sends imu orientation to imu/orientation topic

#!/usr/bin/env python3
"""
Movella DOT → ROS 2 QuaternionStamped publisher (using bleak)
Install: pip install bleak --break-system-packages
"""

import asyncio
import struct
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped
from scipy.spatial.transform import Rotation

import std_msgs

# ── Replace this with your DOT's Bluetooth MAC address ────────────────────────
DOT_ADDRESS = "D4:22:CD:00:05:5E"

# ── BLE UUIDs from Movella DOT BLE Service Specification ─────────────────────
CONTROL_CHARACTERISTIC   = "15172001-4947-11e9-8646-d663bd873d93"
SHORT_PAYLOAD_NOTIFY     = "15172004-4947-11e9-8646-d663bd873d93"

# Payload mode 5 = Orientation (Quaternion) — writes to control characteristic
# Bytes: [0x01, 0x01, 0x05]  →  start measurement, mode 5
START_ORIENTATION_QUAT = bytes([0x01, 0x01, 0x05])
STOP_MEASUREMENT       = bytes([0x01, 0x00, 0x00])

class Calibrator:

    def __init__(self):
        self._ref_r = Rotation.identity()
        self._pending = False

    def request(self):
        self._pending = True

    # Pass in the raw quaternion, get back the calibrated one.
    # If a calibration was requested, this sample becomes the new reference.
    def apply(self, x, y, z, w) -> tuple:
        r_raw = Rotation.from_quat([x, y, z, w])  # scipy uses xyzw order

        if self._pending:
            self._ref_r = r_raw.inv()
            self._pending = False

        r_calibrated = self._ref_r * r_raw
        return r_calibrated.as_quat()  # returns [x, y, z, w]

class MovellaDotTalker(Node):

    def __init__(self):
        super().__init__('movella_dot_talker')

        self.declare_parameter('frame_id', 'imu_link')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pub = self.create_publisher(QuaternionStamped, 'imu/orientation', 10)

        self._calibrator = Calibrator()
        self.create_subscription(std_msgs.msg.Empty, 'imu/calibrate', self._on_calibrate, 10)

        # Run the bleak BLE loop in a background thread
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_ble, daemon=True)
        self._thread.start()

        self.get_logger().info(f'Connecting to DOT at {DOT_ADDRESS} ...')

        self._stop_event = asyncio.Event()  # To signal the BLE loop to stop on shutdown
        self.stopping = False

    # ── BLE thread ────────────────────────────────────────────────────────────
    def _run_ble(self):
        self._loop.run_until_complete(self._ble_loop())

    async def _ble_loop(self):
        from bleak import BleakClient

        async with BleakClient(DOT_ADDRESS) as client:
            self.get_logger().info('Connected to Movella DOT')
            await asyncio.sleep(1.0)

            await client.write_gatt_char(CONTROL_CHARACTERISTIC, STOP_MEASUREMENT)
            await asyncio.sleep(0.5)

            await client.start_notify(SHORT_PAYLOAD_NOTIFY, self._on_packet)
            await client.write_gatt_char(CONTROL_CHARACTERISTIC, START_ORIENTATION_QUAT)

            try:
                # Wait until stop is requested
                await self._stop_event.wait()
            finally:
                self.get_logger().info('Stopping measurement...')
                await client.write_gatt_char(CONTROL_CHARACTERISTIC, STOP_MEASUREMENT)
                await asyncio.sleep(0.3)

    def stop_ble(self):
        self.stopping = True
        # Called from the main thread to trigger clean shutdown
        self._loop.call_soon_threadsafe(self._stop_event.set)
    
    def _on_calibrate(self, msg):
        self._calibrator.request()


    # ── Packet parser ─────────────────────────────────────────────────────────
    def _on_packet(self, sender, data: bytearray):
        if self.stopping:
            return  # Ignore packets received during shutdown
        # Short payload (mode 5 — Orientation Quaternion) is 20 bytes:
        # [0..3]  timestamp (uint32, ms)
        # [4..7]  w (float32)
        # [8..11] x (float32)
        # [12..15] y (float32)
        # [16..19] z (float32)
        if len(data) < 20:
            return

        _, w, x, y, z = struct.unpack_from('<Iffff', data, 0)

        q_calibrated = self._calibrator.apply(x, y, z, w)  # apply calibration

        x, y, z, w = q_calibrated

        msg = QuaternionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # x and z axes are switched because the imu was attached vertically to the head rotated 90 degrees along the x axis.
        msg.quaternion.w = w
        msg.quaternion.x = -z
        msg.quaternion.y = y
        msg.quaternion.z = x

        print(f'Got DOT quaternion: w={msg.quaternion.w:.4f}, x={msg.quaternion.x:.4f}, y={msg.quaternion.y:.4f}, z={msg.quaternion.z:.4f}')

        self.pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = MovellaDotTalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down, stopping DOT...')
        node.stop_ble()
        node._thread.join(timeout=3.0)  # wait for BLE thread to finish
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()