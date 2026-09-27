#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import time

import cv2


def parse_args():
    parser = argparse.ArgumentParser(description="录制 USB 摄像头视频到 mp4 文件")
    parser.add_argument("--device", default="/dev/video6", help="摄像头设备路径，默认 /dev/video6")
    parser.add_argument("--output", default="camera_record.mp4", help="输出 mp4 文件路径")
    parser.add_argument("--width", type=int, default=1280, help="输出视频宽度")
    parser.add_argument("--height", type=int, default=720, help="输出视频高度")
    parser.add_argument("--fps", type=float, default=30.0, help="录制帧率")
    parser.add_argument("--flip", action="store_true", help="是否对画面做上下翻转，适合部分倒装相机")
    parser.add_argument("--backend", choices=["v4l2", "default"], default="v4l2", help="视频采集后端，Jetson 上默认用 v4l2 更稳")
    parser.add_argument("--no-preview", action="store_true", help="不显示图像窗口，适合无图形界面环境")
    return parser.parse_args()


def main():
    args = parse_args()

    # 中文注释：先检查输出目录，避免录像启动后因为路径不存在而直接失败
    output_path = os.path.abspath(args.output)
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        print(f"[record_usb_camera] 输出目录不存在: {output_dir}")
        return 1

    if args.backend == "v4l2":
        # 中文注释：Jetson 上直接指定 V4L2 后端通常比默认 GStreamer 更稳定，避免 /dev/videoX 打开失败
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print(f"[record_usb_camera] 无法打开摄像头: {args.device}")
        return 1

    # 中文注释：这里尽量向驱动请求目标分辨率和帧率，但最终实际值仍以后面读到的帧为准
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    if actual_width <= 0 or actual_height <= 0:
        print("[record_usb_camera] 读取到的图像尺寸无效")
        cap.release()
        return 1
    if actual_fps <= 1e-3:
        actual_fps = args.fps

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(output_path, fourcc, actual_fps, (actual_width, actual_height))
    if not writer.isOpened():
        print(f"[record_usb_camera] 无法创建输出视频: {output_path}")
        cap.release()
        return 1

    print(f"[record_usb_camera] 开始录制: {args.device} -> {output_path}")
    print(f"[record_usb_camera] 实际分辨率: {actual_width}x{actual_height}, fps: {actual_fps:.2f}")
    if args.no_preview:
        print("[record_usb_camera] 当前为无预览模式，按 Ctrl+C 结束录制")
    else:
        print("[record_usb_camera] 按 q 结束录制")

    frame_count = 0
    start_time = time.time()

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[record_usb_camera] 读取图像失败，结束录制")
                break

            if args.flip:
                # 中文注释：部分机体安装时相机画面会上下颠倒，这里提供可选翻转
                frame = cv2.flip(frame, -1)

            writer.write(frame)
            frame_count += 1

            if not args.no_preview:
                cv2.imshow("record_usb_camera", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
    except KeyboardInterrupt:
        print("\n[record_usb_camera] 收到 Ctrl+C，结束录制")

    cap.release()
    writer.release()
    cv2.destroyAllWindows()

    duration = max(time.time() - start_time, 1e-6)
    print(f"[record_usb_camera] 结束录制，共写入 {frame_count} 帧，时长 {duration:.2f} 秒")
    return 0


if __name__ == "__main__":
    sys.exit(main())
