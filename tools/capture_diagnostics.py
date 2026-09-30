#!/usr/bin/env python3
"""Read-only ROS image capture; never opens the game serial port."""
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class Capture(Node):
    def __init__(self):
        super().__init__('assessment_read_only_capture')
        self.directory = Path(__file__).resolve().parents[1] / 'diagnostics' / time.strftime('%Y%m%d-%H%M%S')
        self.directory.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.started = time.monotonic()
        self.last_saved = -1.0
        self.count = 0
        self.done = False
        from sensor_msgs.msg import Image
        self.subscription = self.create_subscription(Image, '/image_raw', self.capture, qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self.check_timeout)
        print('Ready. Click Start in the game now. No serial commands will be sent.', flush=True)

    def check_timeout(self):
        if time.monotonic() - self.started >= 90:
            print(f'Finished: {self.count} frames. Directory: {self.directory}', flush=True)
            self.done = True

    def capture(self, message):
        now = time.monotonic()
        if self.count >= 60 or now - self.last_saved < 0.5:
            return
        if message.width == 0 or message.height == 0:
            return
        try:
            raw = np.frombuffer(message.data, dtype=np.uint8)
            if raw.size < message.step * message.height:
                raise ValueError('Truncated image buffer')
            rows = raw[:message.step * message.height].reshape(message.height, message.step)
            if message.step == message.width * 4:
                pixels = rows.reshape(message.height, message.width, 4)
                rgba = cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
                bgra = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
                images = [('RGBA', rgba), ('BGRA', bgra)]
            else:
                images = [('declared_encoding', self.bridge.imgmsg_to_cv2(message, 'bgr8'))]
            prefix = self.directory / f'frame_{self.count:02d}'
            np.save(str(prefix) + '_raw.npy', raw)
            for label, image in images:
                if not cv2.imwrite(str(prefix) + '_' + label + '.png', image):
                    raise RuntimeError('PNG write failed')
            metadata = dict(width=message.width, height=message.height, step=message.step,
                            encoding=message.encoding, bytes=int(raw.size),
                            stamp_sec=message.header.stamp.sec, stamp_nsec=message.header.stamp.nanosec,
                            receive_monotonic=now)
            Path(str(prefix) + '.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
            self.count += 1
            self.last_saved = now
            print(f'Saved frame {self.count}/60', flush=True)
            if self.count == 60:
                print(f'Capture complete. Directory: {self.directory}', flush=True)
                self.done = True
        except (ValueError, RuntimeError, cv2.error) as error:
            print(f'Capture error: {error}', flush=True)
            self.done = True


def main():
    rclpy.init()
    node = Capture()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
