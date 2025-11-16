#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TOOL 프레임 전송 + TOOL0 적분 + 지연보상식 로깅 + P-보정으로 drift 억제
(Admittance-like 필터 적용 버전)

이번 버전 요약:
- cmd 공유메모리(test2.py 기준):
    cmd[0:3] = lin (vx, vy, vz)              [m/s 개념]
    cmd[3:6] = 원래 ang (wx, wy, wz)         [무시]
    cmd[6]   = right_trigger_value           (>0.5 → gripper CLOSE)
    cmd[7]   = right_aButton (0/1)
    cmd[8]   = right_bButton (0/1)
    cmd[9]   = right_thumbstick_x
    cmd[10]  = right_thumbstick_y
    cmd[11]  = left_aButton (카메라 전환용 등, 여기서는 미사용)

- 선속도는 VR→v_tool_in→필터 구조 그대로 유지.
- 각속도는 VR에서 온 wx,wy,wz는 완전히 무시하고,
  A/B/스틱으로만 생성 (deg/s 단위, TOOL 기준):
    - A > 0.5       → pitch +5 deg/s (y축)
    - B > 0.5       → pitch -5 deg/s (y축)
    - thumb_x>=0.5  → yaw   +5 deg/s (z축)
      thumb_x<=-0.5 → yaw   -5 deg/s (z축)
    - thumb_y>=0.5  → roll  +5 deg/s (x축)
      thumb_y<=-0.5 → roll  -5 deg/s (x축)
- 각속도는 deg/s 단위로 필터/클램프하고, TwistCommand.angular_* 에도 deg/s 그대로 전달.
- 회전 적분(dR)에서는 deg/s → rad/s 변환해서 사용.
- trigger(cmd[6]) > 0.5 → 그리퍼 CLOSE(1.0), 그 외 → OPEN(0.0),
  상태가 바뀔 때(엣지)만 SendGripperCommand 호출.
"""

import os, sys, time, csv, atexit
import numpy as np
from multiprocessing import shared_memory
from scipy.spatial.transform import Rotation as R

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2

# ===== 파라미터 =====
CMD_SHM_NAME = "cmd"

# VR→TOOL 선속도 매핑 (예전 값 유지)
LIN_SCALE = 1.4

# --- Admittance-like filter parameters ---
TAU_LIN = 0.25     # s, linear velocity time constant
G_LIN   = 0.98     # steady-state gain
TAU_ANG = 0.25     # s, angular velocity time constant
G_ANG   = 0.98

# 속도 제한 / Deadband
LIN_MAX       = 0.12          # m/s
ANG_MAX_DEG   = 30.0          # deg/s (각속도 최대값; 필요하면 조절)
DEADBAND      = 0.0005        # m/s (선속도 deadband)

# P-보정 (TOOL0 기준, drift 억제)
KP_POS = 0.45         # s^-1
KP_ORI = 0.22         # —
LOOP_DT = 0.01        # s (100Hz)

# 버튼/스틱 기준 각속도 크기 (deg/s)
BTN_DEG_PER_SEC   = 5.0
STICK_THRESHOLD   = 0.5

def clamp_vec(v, m):
    n = np.linalg.norm(v)
    if n <= m or n < 1e-12:
        return v
    return v * (m / n)

def apply_deadband(v, th):
    n = np.linalg.norm(v)
    if n < th:
        return np.zeros_like(v)
    return v

def move_to_home(base: BaseClient):
    import threading
    m = Base_pb2.ServoingModeInformation()
    m.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(m)
    req = Base_pb2.RequestedActionType()
    req.action_type = Base_pb2.REACH_JOINT_ANGLES
    acts = base.ReadAllActions(req)
    h = None
    for a in acts.action_list:
        if a.name == "Home":
            h = a.handle
            break
    if h is None:
        print("[!] 'Home' 액션 없음 → 스킵")
        return False
    e = threading.Event()
    sub = base.OnNotificationActionTopic(
        lambda n: e.set() if n.action_event in (Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT) else None,
        Base_pb2.NotificationOptions()
    )
    base.ExecuteActionFromReference(h)
    ok = e.wait(20)
    base.Unsubscribe(sub)
    print("[OK] Home 이동" if ok else "[!] Home 실패/타임아웃")
    return ok

def send_zero_twist_tool(base: BaseClient):
    cmd = Base_pb2.TwistCommand()
    cmd.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    cmd.twist.linear_x = cmd.twist.linear_y = cmd.twist.linear_z = 0.0
    cmd.twist.angular_x = cmd.twist.angular_y = cmd.twist.angular_z = 0.0
    base.SendTwistCommand(cmd)
    print("[CMD] zero (TOOL)")

def set_gripper_position(base: BaseClient, pos: float):
    """
    pos: 0.0(완전 open) ~ 1.0(완전 close)
    SendGripperCommand() / GRIPPER_POSITION 모드로 제어
    """
    try:
        pos = float(np.clip(pos, 0.0, 1.0))

        gc = Base_pb2.GripperCommand()
        gc.mode = Base_pb2.GRIPPER_POSITION

        finger = gc.gripper.finger.add()
        finger.value = pos  # 0~1 fraction

        base.SendGripperCommand(gc)
        print(f"[GRIPPER] pos={pos:.2f}")
    except Exception as e:
        print(f"[GRIPPER] command failed: {e}")

# ===== 메인 텔레옵 루프 =====
def teleop_all_tool_with_pcorr(base: BaseClient, base_cyclic: BaseCyclicClient,
                               loop_dt=LOOP_DT, csv_path="pose_toolframe_pcorr.csv"):

    # 0) SHM attach
    cmd_shm = shared_memory.SharedMemory(name=CMD_SHM_NAME)
    n_slots = cmd_shm.size // 8
    if n_slots < 12:
        raise RuntimeError(f"cmd shm too small: {n_slots} (need >= 12)")
    cmd_array = np.ndarray((n_slots,), dtype=np.float64, buffer=cmd_shm.buf)

    # 1) CSV 초기화
    f = open(csv_path, "w", newline="")
    atexit.register(lambda: f.close())
    w = csv.writer(f)
    header = [
        "timestamp",
        "des_x_t0","des_y_t0","des_z_t0","des_roll_t0","des_pitch_t0","des_yaw_t0",
        "act_x_t0","act_y_t0","act_z_t0","act_roll_t0","act_pitch_t0","act_yaw_t0",
        "dt"
    ]
    w.writerow(header)
    print(f"[CSV] {os.path.abspath(csv_path)}")

    # 2) 초기 TOOL0 기준좌표 고정
    fb0 = base_cyclic.RefreshFeedback()
    p0_b = np.array([fb0.base.tool_pose_x, fb0.base.tool_pose_y, fb0.base.tool_pose_z], float)
    R0_b = R.from_euler(
        "xyz",
        [fb0.base.tool_pose_theta_x, fb0.base.tool_pose_theta_y, fb0.base.tool_pose_theta_z],
        degrees=True
    ).as_matrix()

    # 3) desired 내부 상태 (TOOL0)
    des_p_t0 = np.zeros(3)
    des_R_t0 = np.eye(3)

    # 4) 필터 내부 상태
    v_tool_f       = np.zeros(3)  # m/s
    w_tool_f_deg   = np.zeros(3)  # deg/s

    # 5) pending desired 기록용
    pend_des_p_t0 = None
    pend_des_R_t0 = None
    pend_dt = None
    have_pending = False

    # 6) 그리퍼 상태 (CLOSE/OPEN)
    prev_grip_close = None  # True/False/None

    prev_t = None
    print("[Teleop] START")
    print("  - 선속도: VR lin 그대로 + Admittance 필터")
    print("  - 각속도: VR ang는 무시, A/B/스틱으로만 생성 (deg/s)")
    print("  - trigger>0.5: gripper CLOSE, 나머지: OPEN (엣지에서만 명령)")

    try:
        while True:
            tic = time.time()

            # A) actual_{k+1} 읽기
            fb = base_cyclic.RefreshFeedback()
            p_b = np.array([fb.base.tool_pose_x, fb.base.tool_pose_y, fb.base.tool_pose_z], float)
            R_b = R.from_euler(
                "xyz",
                [fb.base.tool_pose_theta_x, fb.base.tool_pose_theta_y, fb.base.tool_pose_theta_z],
                degrees=True
            ).as_matrix()

            # actual in TOOL0
            act_R_t0 = R0_b.T @ R_b
            act_p_t0 = R0_b.T @ (p_b - p0_b)

            # dt
            now = time.time()
            dt = loop_dt if prev_t is None else max(1e-3, now - prev_t)
            prev_t = now

            # B) 이전 desired와 현재 actual 기록
            if have_pending:
                des_eul = R.from_matrix(pend_des_R_t0).as_euler("xyz", degrees=False)
                act_eul = R.from_matrix(act_R_t0).as_euler("xyz", degrees=False)
                w.writerow([
                    now,
                    pend_des_p_t0[0], pend_des_p_t0[1], pend_des_p_t0[2],
                    des_eul[0], des_eul[1], des_eul[2],
                    act_p_t0[0],  act_p_t0[1],  act_p_t0[2],
                    act_eul[0],   act_eul[1],   act_eul[2],
                    pend_dt
                ])
                have_pending = False

            # C) SHM → cmd 읽기
            vx, vy, vz = cmd_array[0:3]        # 선속도 입력 (VR)
            trig       = cmd_array[6]          # right_trigger_value
            a_btn      = cmd_array[7]          # right_aButton (0/1)
            b_btn      = cmd_array[8]          # right_bButton (0/1)
            thumb_x    = cmd_array[9]          # right_thumbstick_x
            thumb_y    = cmd_array[10]         # right_thumbstick_y
            # left_a    = cmd_array[11]        # 여기서는 사용 안 함

            # C-1) 그리퍼: trigger > 0.5 → CLOSE, 그 외 OPEN (엣지에서만)
            grip_close = trig > 0.5
            if (prev_grip_close is None) or (grip_close != prev_grip_close):
                set_gripper_position(base, 1.0 if grip_close else 0.0)
                prev_grip_close = grip_close

            # C-2) 선속도: 기존 방식 유지 (VR→TOOL 매핑)
            # TOOL 축 기준: [vy, vz, vx] 사용 (기존 코드와 동일)
            v_tool_in = np.array([vy, vz, vx], float) * LIN_SCALE
            v_tool_in = apply_deadband(v_tool_in, DEADBAND)

            # C-3) 각속도: VR wx,wy,wz는 무시하고 A/B/스틱으로 생성 (deg/s)
            # roll(x), pitch(y), yaw(z)
            roll_deg  = 0.0
            pitch_deg = 0.0
            yaw_deg   = 0.0

            # pitch: A/B 버튼
            if a_btn > 0.5:
                pitch_deg += BTN_DEG_PER_SEC
            if b_btn > 0.5:
                pitch_deg -= BTN_DEG_PER_SEC

            # yaw: 스틱 x
            if thumb_x >= STICK_THRESHOLD:
                yaw_deg += BTN_DEG_PER_SEC
            elif thumb_x <= -STICK_THRESHOLD:
                yaw_deg -= BTN_DEG_PER_SEC

            # roll: 스틱 y
            if thumb_y >= STICK_THRESHOLD:
                roll_deg += BTN_DEG_PER_SEC
            elif thumb_y <= -STICK_THRESHOLD:
                roll_deg -= BTN_DEG_PER_SEC

            w_tool_in_deg = np.array([roll_deg, pitch_deg, yaw_deg], float)

            # ---- Admittance-like filter ----
            a_lin = np.exp(-dt / TAU_LIN)
            b_lin = G_LIN * (1.0 - a_lin)
            a_ang = np.exp(-dt / TAU_ANG)
            b_ang = G_ANG * (1.0 - a_ang)

            v_tool_f      = a_lin * v_tool_f      + b_lin * v_tool_in
            w_tool_f_deg  = a_ang * w_tool_f_deg  + b_ang * w_tool_in_deg

            # ---- Clamp ----
            v_tool_cmd      = clamp_vec(v_tool_f, LIN_MAX)          # m/s
            w_tool_cmd_deg  = clamp_vec(w_tool_f_deg, ANG_MAX_DEG)  # deg/s

            # D) TOOL 프레임 명령 전송 (angular in deg/s)
            cmd_msg = Base_pb2.TwistCommand()
            cmd_msg.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
            cmd_msg.twist.linear_x, cmd_msg.twist.linear_y, cmd_msg.twist.linear_z = v_tool_cmd.tolist()
            cmd_msg.twist.angular_x, cmd_msg.twist.angular_y, cmd_msg.twist.angular_z = w_tool_cmd_deg.tolist()
            base.SendTwistCommand(cmd_msg)

            # E) desired 예측 + P-보정
            # 적분용으로만 deg/s → rad/s 변환
            w_tool_cmd_rad = np.deg2rad(w_tool_cmd_deg)
            dR = R.from_rotvec(w_tool_cmd_rad * dt).as_matrix()
            des_R_pred = des_R_t0 @ dR
            des_p_pred = des_p_t0 + (des_R_t0 @ (v_tool_cmd * dt))

            pos_err_t0 = act_p_t0 - des_p_pred
            des_p_new  = des_p_pred + KP_POS * pos_err_t0

            R_err = act_R_t0 @ des_R_pred.T
            err_rv = R.from_matrix(R_err).as_rotvec()
            des_R_new = R.from_rotvec(KP_ORI * err_rv).as_matrix() @ des_R_pred

            # F) 다음 틱 로그용 pending 저장
            pend_des_p_t0 = des_p_new
            pend_des_R_t0 = des_R_new
            pend_dt = dt
            have_pending = True

            # G) 내부 desired 상태 갱신
            des_p_t0 = des_p_new
            des_R_t0 = des_R_new

            time.sleep(max(0.0, loop_dt - (time.time() - tic)))

    except KeyboardInterrupt:
        print("\n[Stop] Ctrl+C")
        send_zero_twist_tool(base)
        cmd_shm.close()
        f.close()

def main():
    sys.path.append("/home/icatheon/Kinova-kortex2_Gen3_G3L/api_python/examples")
    import utilities
    args = utilities.parseConnectionArguments()
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        try:
            move_to_home(base)
        except Exception:
            pass
        ts = time.strftime("%Y%m%d_%H%M%S")
        csv_path = f"pose_toolframe_pcorr_{ts}.csv"
        teleop_all_tool_with_pcorr(base, base_cyclic, loop_dt=LOOP_DT, csv_path=csv_path)

if __name__ == "__main__":
    sys.exit(main())
