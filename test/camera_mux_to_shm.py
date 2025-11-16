#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
버튼 전 = RealSense → televuer_img 공유메모리로 컬러 프레임 쓰기
A 버튼(토글) = Kinova ROS 토픽 컬러 → televuer_img로 전환
좌/우 동일 프레임 복제 (480x1280x3), 이름 "televuer_img"
cmd 공유메모리(name="cmd"): test2.py에서 이미 생성/업데이트
  [0:3]=lin, [3:6]=ang, [6]=trigger, [7]=A, [8]=B, [9]=thumb_x, [10]=thumb_y
"""
import time, threading
import numpy as np
import cv2
from multiprocessing import shared_memory

# RealSense
import pyrealsense2 as rs

# ROS (Kinova color topic)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# -------------------- 설정 --------------------
SHM_IMG_NAME = "televuer_img"
SHM_IMG_SHAPE = (480, 640*2, 3)   # (H, 2*W, 3) -> 좌/우 복제
CMD_NAME     = "cmd"              # test2.py와 동일
TARGET_FPS   = 30.0
KINOVA_COLOR_TOPIC = "/camera/color/image_raw"  # 필요시 인자로 받아도 됨

# -------------------- 공유메모리 유틸 --------------------
class ShmCanvas:
    def __init__(self, name=SHM_IMG_NAME, shape=SHM_IMG_SHAPE, create=False):
        self.name = name
        self.shape = shape
        self.dtype = np.uint8
        nbytes = int(np.prod(shape)) * np.dtype(self.dtype).itemsize
        # attach 전용 (test2.py가 create=True로 이미 만들었음)
        self.shm = shared_memory.SharedMemory(name=self.name, create=create, size=nbytes if create else 0)
        self.canvas = np.ndarray(self.shape, dtype=self.dtype, buffer=self.shm.buf)
        self.H, self.W2, _ = self.shape
        self.W = self.W2 // 2

    def write_lr(self, img_bgr):
        # img를 (H, W, 3)로 변환 후 좌/우 동일 기록
        if img_bgr is None:
            left = np.zeros((self.H, self.W, 3), dtype=np.uint8)
        else:
            im = img_bgr
            if im.ndim == 2:
                im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
            elif im.ndim == 3 and im.shape[2] == 4:
                im = im[:, :, :3]
            if im.shape[:2] != (self.H, self.W):
                left = cv2.resize(im, (self.W, self.H))
            else:
                left = im
        self.canvas[:, :self.W, :] = left
        self.canvas[:, self.W:, :] = left

    def close(self):
        try: self.shm.close()
        except Exception: pass

# -------------------- RealSense 소스 --------------------
class RealSenseSource:
    def __init__(self, width=640, height=480, fps=30):
        self.width, self.height, self.fps = int(width), int(height), int(fps)
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        self.pipeline.start(self.config)
    def read(self):
        frames = self.pipeline.wait_for_frames(1000)
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())
    def close(self):
        try: self.pipeline.stop()
        except Exception: pass

# -------------------- ROS(Kinova) 소스 --------------------
class KinovaColorSource(Node):
    def __init__(self, topic=KINOVA_COLOR_TOPIC):
        super().__init__("kinova_color_source")
        self.bridge = CvBridge()
        self.topic = topic
        self._lock = threading.Lock()
        self._latest = None
        self.create_subscription(Image, self.topic, self._on_color, 10)
    def _on_color(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._latest = img
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
    def latest(self):
        with self._lock:
            return None if self._latest is None else self._latest.copy()

def start_ros_color_thread(topic=KINOVA_COLOR_TOPIC):
    rclpy.init(args=None)
    node = KinovaColorSource(topic)
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    return node

# -------------------- cmd 공유메모리 --------------------
class CmdReader:
    def __init__(self, name=CMD_NAME):
        self.shm = shared_memory.SharedMemory(name=name, create=False)
        self.arr = np.ndarray((12,), dtype=np.float64, buffer=self.shm.buf)
        self.prev_a = 0.0
        self.mode = 0  # 0=RealSense, 1=Kinova
    def poll(self):
        a = float(self.arr[11])  # A버튼
        # 상승엣지로 토글
        if a > 0.5 and self.prev_a <= 0.5:
            self.mode = 1 - self.mode
            print(f"[MUX] Toggle! mode={self.mode} (0=RealSense, 1=Kinova)")
        self.prev_a = a
        return self.mode
    def close(self):
        try: self.shm.close()
        except Exception: pass

# -------------------- 메인 루프 --------------------
def main():
    # 공유메모리 attach (test2.py 실행 후)
    canvas = ShmCanvas(create=False)
    # RealSense 소스
    rs_src = RealSenseSource(640, 480, 30)
    # ROS(Kinova) 소스
    ros_src = start_ros_color_thread(KINOVA_COLOR_TOPIC)
    # cmd 리더
    cmd = CmdReader()

    target_dt = 1.0 / TARGET_FPS
    print("[MUX] Running... A 버튼으로 소스 전환 (0=RS, 1=Kinova). Ctrl+C to exit.")

    try:
        while rclpy.ok():
            t0 = time.time()
            mode = cmd.poll()
            if mode == 0:
                frame = rs_src.read()
            else:
                frame = ros_src.latest()
                if frame is None:
                    # Kinova 프레임 아직 없으면 마지막/검정으로
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)

            # televuer_img에 기록 (좌/우 복제)
            canvas.write_lr(frame)

            # 로컬 미리보기 원하면 주석 해제
            # cv2.imshow("MUX out", cv2.resize(frame, (640,480)))
            # if cv2.waitKey(1) & 0xFF == 27:
            #     break

            # FPS 맞추기
            dt = time.time() - t0
            if dt < target_dt:
                time.sleep(target_dt - dt)

    except KeyboardInterrupt:
        pass
    finally:
        cmd.close()
        rs_src.close()
        canvas.close()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        # cv2.destroyAllWindows()
        print("[MUX] Exit.")

if __name__ == "__main__":
    main()
