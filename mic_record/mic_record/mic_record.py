#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus
import subprocess
import datetime
import os


class MicRecord(Node):
    def __init__(self):
        super().__init__('mic_record')

        # Subscribe to PX4 VehicleStatus
        self.sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',  # 필요하면 네임스페이스 수정
            self.status_callback,
            10
        )

        # Internal state
        self.last_arming_state = None
        self.record_proc = None

        # Output directory (계정명 맞게 수정)
        self.output_dir = '/home/ubuntu/recordings'
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info('MicRecord node started.')

    def status_callback(self, msg: VehicleStatus):
        current_state = msg.arming_state

        # First message: initialize tracking
        if self.last_arming_state is None:
            self.last_arming_state = current_state
            return

        # DISARM -> ARM : start recording
        if (self.last_arming_state != VehicleStatus.ARMING_STATE_ARMED and
                current_state == VehicleStatus.ARMING_STATE_ARMED):
            self.get_logger().info('Detected ARM transition -> start recording')
            self.start_recording()

        # ARM -> DISARM : stop recording
        if (self.last_arming_state == VehicleStatus.ARMING_STATE_ARMED and
                current_state != VehicleStatus.ARMING_STATE_ARMED):
            self.get_logger().info('Detected DISARM transition -> stop recording')
            self.stop_recording()

        self.last_arming_state = current_state

    def start_recording(self):
        if self.record_proc is not None:
            self.get_logger().warn('Recording already in progress.')
            return

        # Timestamp for filename
        now = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'{now}.wav'
        wav_path = os.path.join(self.output_dir, filename)

        # arecord command for ReSpeaker (카드/채널은 arecord -l 로 확인해서 수정)
        cmd = [
            'arecord',
            '-D', 'hw:1,0',
            '-f', 'S16_LE',
            '-c', '6',
            '-r', '16000',
            '-t', 'wav',
            wav_path
        ]

        self.get_logger().info(f'Start recording: {wav_path}')
        try:
            self.record_proc = subprocess.Popen(cmd)
        except Exception as e:
            self.get_logger().error(f'Failed to start arecord: {e}')
            self.record_proc = None

    def stop_recording(self):
        if self.record_proc is None:
            self.get_logger().warn('No recording process to stop.')
            return

        self.get_logger().info('Stopping recording.')
        self.record_proc.terminate()
        try:
            self.record_proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('arecord did not terminate, killing.')
            self.record_proc.kill()

        self.record_proc = None


def main(args=None):
    rclpy.init(args=args)
    node = MicRecord()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_recording()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
