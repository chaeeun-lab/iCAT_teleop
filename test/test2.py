#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, numpy as np
from multiprocessing import shared_memory
from scipy.spatial.transform import Rotation as R
from televuer import TeleVuerWrapper
import logging_mp

logger_mp = logging_mp.get_logger(__name__, level=logging_mp.INFO)

def run_producer():
    # 실제 사용하는 스테레오 프레임 버퍼 (공유메모리)
    image_shape = (480, 640 * 2, 3)
    image_bytes = int(np.prod(image_shape) * np.uint8().itemsize)
    # 기존
    # image_shm = shared_memory.SharedMemory(create=True, size=image_bytes)

    # 변경 (이름 고정)
    image_shm = shared_memory.SharedMemory(create=True, name="televuer_img", size=image_bytes)


    # cmd 공유메모리: [vx,vy,vz, wx,wy,wz, trigger, A, B, thumb_x, thumb_y]
    cmd_shm = shared_memory.SharedMemory(create=True, name="cmd", size=12 * 8)
    cmd_array = np.ndarray((12,), dtype=np.float64, buffer=cmd_shm.buf)

    tv = TeleVuerWrapper(
        binocular=True,
        use_hand_tracking=False,
        img_shape=image_shape,
        img_shm_name="televuer_img",
        return_state_data=True,
        return_hand_rot_data=True
    )

    prev_pos = None
    prev_rot = None
    prev_t = None
    target_dt = 0.033  # ~30 Hz

    try:
        input("Press Enter to start TeleVuer producer...")
        while True:
            t0 = time.time()
            tele = tv.get_motion_state_data()

            pose = tele.right_arm_pose
            cur_p = pose[:3, 3]
            cur_R = pose[:3, :3]

            if prev_pos is None:
                lin = np.zeros(3); ang = np.zeros(3)
            else:
                dt = max(1e-3, t0 - prev_t)
                lin = (cur_p - prev_pos) / dt
                r_prev = R.from_matrix(prev_rot)
                r_curr = R.from_matrix(cur_R)
                ang = (r_curr * r_prev.inv()).as_rotvec() / dt

            # 공유메모리에 기록
            cmd_array[0:3] = lin
            cmd_array[3:6] = ang
            cmd_array[6] = tele.right_trigger_value
            cmd_array[7] = 1.0 if tele.tele_state.right_aButton else 0.0
            cmd_array[8] = 1.0 if tele.tele_state.right_bButton else 0.0
            cmd_array[9] = tele.tele_state.right_thumbstick_value[0]
            cmd_array[10] = tele.tele_state.right_thumbstick_value[1]
            cmd_array[11] = 1.0 if  tele.tele_state.left_aButton else 0.0
            prev_pos = cur_p.copy()
            prev_rot = cur_R.copy()
            prev_t = t0

            logger_mp.info(f"[lin]: {lin}, [ang]: {ang}")
            time.sleep(max(0.0, target_dt - (time.time() - t0)))
    except KeyboardInterrupt:
        logger_mp.warning("KeyboardInterrupt — exiting producer...")
    finally:
        try:
            image_shm.unlink(); image_shm.close()
        except Exception:
            pass
        try:
            cmd_shm.close()
        except Exception:
            pass
        logger_mp.warning("Producer exited cleanly.")

if __name__ == "__main__":
    run_producer()
