#!/usr/bin/env python3
"""Record an OAK-D Lite ROS 2 image topic to an MP4 file."""

import argparse
import math
import os
import signal
import sys
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class OakVideoRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("oak_d_lite_video_recorder")
        self.args = args
        self.bridge = CvBridge()
        self.writer = None
        self.output_path = self._make_output_path()
        self.frame_size = None
        self.frame_count = 0
        self.start_time = None
        self.last_log_time = self.get_clock().now()
        self.buffer = deque()
        self.buffer_timestamps = deque()

        self.subscription = self.create_subscription(
            Image,
            args.topic,
            self._image_callback,
            self._make_qos_profile(args.qos, args.queue_size),
        )

        self.get_logger().info(f"Recording topic: {args.topic}")
        self.get_logger().info(f"Output file: {self.output_path}")
        self.get_logger().info("Press Ctrl+C to stop and finalize the MP4.")

    def _make_qos_profile(self, reliability: str, queue_size: int) -> QoSProfile:
        reliability_policy = (
            ReliabilityPolicy.RELIABLE
            if reliability == "reliable"
            else ReliabilityPolicy.BEST_EFFORT
        )
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=queue_size,
            reliability=reliability_policy,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _make_output_path(self) -> Path:
        output_dir = Path(os.path.expanduser(self.args.output_dir)).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = self.args.filename or f"oak_d_lite_{timestamp}.mp4"
        if not filename.lower().endswith(".mp4"):
            filename += ".mp4"
        return output_dir / filename

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Could not convert image frame: {exc}")
            return

        if frame is None or frame.size == 0:
            self.get_logger().warn("Received an empty image frame; skipping.")
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        if self.writer is None and self.args.fps <= 0:
            self._buffer_until_fps_known(frame, msg)
            return

        if self.writer is None:
            self._open_writer(frame, self.args.fps)

        self._write_frame(frame)

    def _buffer_until_fps_known(self, frame, msg: Image) -> None:
        self.buffer.append(frame.copy())
        self.buffer_timestamps.append(self._message_time_seconds(msg))

        if len(self.buffer) < self.args.auto_fps_frames:
            return

        fps = self._estimate_fps()
        if fps is None:
            fps = self.args.fallback_fps
            self.get_logger().warn(
                f"Could not estimate FPS from timestamps; using {fps:.2f} FPS."
            )
        else:
            self.get_logger().info(f"Estimated stream rate: {fps:.2f} FPS")

        self._open_writer(self.buffer[0], fps)
        while self.buffer:
            self._write_frame(self.buffer.popleft())
        self.buffer_timestamps.clear()

    def _message_time_seconds(self, msg: Image) -> float:
        if msg.header.stamp.sec or msg.header.stamp.nanosec:
            return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds
        return now * 1e-9

    def _estimate_fps(self) -> float | None:
        timestamps = list(self.buffer_timestamps)
        intervals = [
            b - a
            for a, b in zip(timestamps, timestamps[1:])
            if b > a and math.isfinite(b - a)
        ]
        if not intervals:
            return None

        intervals.sort()
        median_interval = intervals[len(intervals) // 2]
        if median_interval <= 0:
            return None

        fps = 1.0 / median_interval
        return min(max(fps, self.args.min_auto_fps), self.args.max_auto_fps)

    def _open_writer(self, frame, fps: float) -> None:
        height, width = frame.shape[:2]
        self.frame_size = (width, height)
        fourcc = cv2.VideoWriter_fourcc(*self.args.codec)
        self.writer = cv2.VideoWriter(str(self.output_path), fourcc, fps, self.frame_size)

        if not self.writer.isOpened():
            raise RuntimeError(
                f"Could not open video writer for {self.output_path} "
                f"with codec {self.args.codec!r} at {fps:.2f} FPS"
            )

        self.get_logger().info(
            f"Started MP4 writer: {width}x{height}, {fps:.2f} FPS, codec {self.args.codec}"
        )

    def _write_frame(self, frame) -> None:
        if self.frame_size is None:
            return

        width, height = self.frame_size
        if frame.shape[1] != width or frame.shape[0] != height:
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

        self.writer.write(frame)
        self.frame_count += 1
        self._log_progress()

        if self.args.duration > 0 and self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            if elapsed >= self.args.duration:
                self.get_logger().info(f"Reached duration limit: {self.args.duration:.1f}s")
                raise KeyboardInterrupt

    def _log_progress(self) -> None:
        now = self.get_clock().now()
        elapsed_since_log = (now - self.last_log_time).nanoseconds * 1e-9
        if elapsed_since_log >= self.args.log_interval:
            self.get_logger().info(f"Recorded {self.frame_count} frames...")
            self.last_log_time = now

    def close(self) -> None:
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        self.get_logger().info(
            f"Finished recording {self.frame_count} frames to {self.output_path}"
        )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record an OAK-D Lite ROS 2 image topic to ~/Desktop as MP4."
    )
    parser.add_argument(
        "--topic",
        default="/oak/rgb/image_raw",
        help="ROS 2 image topic to record. Default: /oak/rgb/image_raw",
    )
    parser.add_argument(
        "--output-dir",
        default="~/Desktop",
        help="Directory where the MP4 will be saved. Default: ~/Desktop",
    )
    parser.add_argument(
        "--filename",
        default=None,
        help="Optional MP4 filename. Default: oak_d_lite_<timestamp>.mp4",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=0.0,
        help="Output FPS. Use 0 to estimate from incoming image timestamps. Default: 0",
    )
    parser.add_argument(
        "--fallback-fps",
        type=float,
        default=10.0,
        help="FPS used if automatic estimation fails. Default: 10",
    )
    parser.add_argument(
        "--auto-fps-frames",
        type=int,
        default=8,
        help="Number of initial frames used for automatic FPS estimation. Default: 8",
    )
    parser.add_argument(
        "--min-auto-fps",
        type=float,
        default=1.0,
        help="Minimum accepted automatically estimated FPS. Default: 1",
    )
    parser.add_argument(
        "--max-auto-fps",
        type=float,
        default=60.0,
        help="Maximum accepted automatically estimated FPS. Default: 60",
    )
    parser.add_argument(
        "--codec",
        default="mp4v",
        help="OpenCV fourcc codec for MP4 output. Default: mp4v",
    )
    parser.add_argument(
        "--qos",
        choices=("best_effort", "reliable"),
        default="best_effort",
        help="Subscription reliability. best_effort is least intrusive. Default: best_effort",
    )
    parser.add_argument(
        "--queue-size",
        type=int,
        default=5,
        help="ROS subscription queue depth. Default: 5",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Optional recording duration in seconds. 0 records until Ctrl+C. Default: 0",
    )
    parser.add_argument(
        "--log-interval",
        type=float,
        default=5.0,
        help="Seconds between progress logs. Default: 5",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if len(args.codec) != 4:
        print("--codec must be exactly four characters, for example mp4v", file=sys.stderr)
        return 2
    if args.auto_fps_frames < 2:
        print("--auto-fps-frames must be at least 2", file=sys.stderr)
        return 2

    rclpy.init()
    node = OakVideoRecorder(args)

    def handle_signal(_signum, _frame) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
