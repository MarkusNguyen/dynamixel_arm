#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import math
import json
import threading
import dotenv
import streamlit as st

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

from google import genai
from google.genai import types

# ---------------------------------------------------------------------------
# Configuration & Topics
# ---------------------------------------------------------------------------
ARM_ACTION_TOPIC = "/arm_controller/follow_joint_trajectory"
GRIPPER_ACTION_TOPIC = "/gripper_controller/follow_joint_trajectory"
TARGET_XYZ_TOPIC = "/target_xyz"
JOINT_STATES_TOPIC = "/joint_states"
GEMINI_MODEL_NAME = "gemini-3.5-flash"

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"]
GRIPPER_JOINTS = ["gripper"]

SCENES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenes.json")
GOAL_ACCEPT_TIMEOUT_SEC = 2.0

SYSTEM_PROMPT = (
    "You are a control agent for a 5-DOF robotic arm with a gripper. "
    "Use move_arm_joints to move the 5 arm joints, set_gripper_position to move ONLY the gripper, "
    "move_to_cartesian_xyz to send end-effector coordinates, or retrieve_joint_states to read joint positions. "
    "Always confirm the action taken."
)

DEFAULT_SCENES = {
    "Home Position": [
        {
            "joints_deg": [0.0, 0.0, 0.0, 0.0, 0.0],
            "gripper_rad": 0.0,
            "duration_sec": 2.0
        }
    ],
    "Pick and Place Demo": [
        {
            "joints_deg": [0.0, -30.0, 45.0, 15.0, 0.0],
            "gripper_rad": 20.0,
            "duration_sec": 2.0
        },
        {
            "joints_deg": [0.0, -30.0, 45.0, 15.0, 0.0],
            "gripper_rad": -5.0,
            "duration_sec": 1.5
        },
        {
            "joints_deg": [45.0, 0.0, 0.0, 0.0, 0.0],
            "gripper_rad": -5.0,
            "duration_sec": 3.0
        },
        {
            "joints_deg": [45.0, -30.0, 45.0, 15.0, 0.0],
            "gripper_rad": 20.0,
            "duration_sec": 2.0
        }
    ]
}


# ---------------------------------------------------------------------------
# Scene Storage Functions
# ---------------------------------------------------------------------------
def save_scenes(scenes: dict) -> None:
    with open(SCENES_FILE, "w") as f:
        json.dump(scenes, f, indent=2)

def load_scenes() -> dict:
    if not os.path.exists(SCENES_FILE):
        save_scenes(DEFAULT_SCENES)
        return DEFAULT_SCENES
    with open(SCENES_FILE, "r") as f:
        try:
            data = json.load(f)
            data = data if data else DEFAULT_SCENES
        except json.JSONDecodeError:
            return DEFAULT_SCENES
    # Migrate legacy steps saved with "gripper_deg" (old degree-based
    # format) to "gripper_rad". The gripper is a multi-turn lead-screw
    # actuator with a large range (~-65 to 95 rad), so degrees and radians
    # are NOT interchangeable the way they are for the revolute arm
    # joints — a step saved in degrees must be converted once, not just
    # relabeled.
    changed = False
    for steps in data.values():
        for step in steps:
            if "gripper_rad" not in step and "gripper_deg" in step:
                step["gripper_rad"] = math.radians(step.pop("gripper_deg"))
                changed = True
    if changed:
        save_scenes(data)
    return data


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------
class ArmControllerNode(Node):
    def __init__(self):
        super().__init__("arm_streamlit_controller")

        self.arm_client = ActionClient(self, FollowJointTrajectory, ARM_ACTION_TOPIC)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, GRIPPER_ACTION_TOPIC)
        self.xyz_pub = self.create_publisher(Point, TARGET_XYZ_TOPIC, 10)

        self.current_joint_positions = {}
        self.current_gripper_pos = 0.0
        self.joint_states_received = threading.Event()

        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._joint_state_cb, 10)

    def _joint_state_cb(self, msg: JointState) -> None:
        self.current_joint_positions = dict(zip(msg.name, msg.position))
        if "gripper" in self.current_joint_positions:
            self.current_gripper_pos = float(self.current_joint_positions["gripper"])
        self.joint_states_received.set()

    def send_trajectory(self, client: ActionClient, joints: list, positions: list, duration_sec: float) -> bool:
        if not client.wait_for_server(timeout_sec=0.5):
            self.get_logger().error(f"Action server unavailable: {client._action_name}")
            return False

        sec = int(duration_sec)
        nsec = int((duration_sec - sec) * 1e9)

        traj = JointTrajectory()
        traj.joint_names = joints

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        accepted_event = threading.Event()
        result_box = {}

        def _on_response(future):
            result_box["handle"] = future.result()
            accepted_event.set()

        future = client.send_goal_async(goal)
        future.add_done_callback(_on_response)

        if not accepted_event.wait(timeout=GOAL_ACCEPT_TIMEOUT_SEC):
            self.get_logger().error(f"Timed out waiting for goal acceptance on joints={joints}")
            return False

        goal_handle = result_box.get("handle")
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Goal REJECTED by controller for joints={joints}")
            return False
        return True

    def send_arm_only(self, arm_pos: list, time_sec: float):
        return self.send_trajectory(self.arm_client, ARM_JOINTS, arm_pos, time_sec)

    def send_gripper_only(self, gripper_pos: float, time_sec: float):
        return self.send_trajectory(self.gripper_client, GRIPPER_JOINTS, [gripper_pos], time_sec)

    def send_commands(self, arm_pos: list, gripper_pos: float, time_sec: float):
        self.send_arm_only(arm_pos, time_sec)
        self.send_gripper_only(gripper_pos, time_sec)

    def send_target_xyz(self, x: float, y: float, z: float):
        msg = Point(x=float(x), y=float(y), z=float(z))
        self.xyz_pub.publish(msg)
        self.get_logger().info(f"Target XYZ published: ({x:.3f}, {y:.3f}, {z:.3f})")

    def send_step(self, joint_positions_rad: list, gripper_rad: float, duration_sec: float) -> bool:
        """Send arm and gripper together for one scene step. If the arm
        goal is rejected, stop here and do NOT send the gripper goal —
        sending a mismatched gripper move after a failed arm move can
        leave the scene in an inconsistent state."""
        arm_ok = self.send_trajectory(self.arm_client, ARM_JOINTS, joint_positions_rad, duration_sec)
        if not arm_ok:
            return False
        gripper_ok = self.send_trajectory(self.gripper_client, GRIPPER_JOINTS, [gripper_rad], duration_sec)
        return gripper_ok


@st.cache_resource
def init_ros_node():
    if not rclpy.ok():
        rclpy.init()
    node = ArmControllerNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    return node

node = init_ros_node()


# ---------------------------------------------------------------------------
# Gemini Tool Bindings
# ---------------------------------------------------------------------------
def move_arm_joints(joint_positions: list[float], duration_sec: float = 2.0) -> str:
    """Move the 5 arm joints to target positions in radians [j1, j2, j3, j4, j5]."""
    if len(joint_positions) != 5:
        return f"Error: expected 5 joint values, got {len(joint_positions)}."
    ok = node.send_arm_only(joint_positions, duration_sec)
    return f"Arm trajectory sent to {joint_positions} ({duration_sec}s)." if ok else "Error sending arm command."

def set_gripper_position(position: float, duration_sec: float = 2.0) -> str:
    """Move ONLY the gripper joint to target position in radians."""
    ok = node.send_gripper_only(position, duration_sec)
    return f"Gripper target set to {position} rad ({duration_sec}s)." if ok else "Error sending gripper command."

def move_to_cartesian_xyz(x: float, y: float, z: float) -> str:
    """Publish Cartesian (X, Y, Z) coordinates in meters to /target_xyz."""
    node.send_target_xyz(x, y, z)
    return f"Target XYZ published: X={x}, Y={y}, Z={z}"

def retrieve_joint_states() -> str:
    """Read current positions of all joints in radians from /joint_states."""
    if not node.joint_states_received.wait(timeout=2.0):
        return "Error: timeout reading /joint_states"
    known = ARM_JOINTS + GRIPPER_JOINTS
    pos = node.current_joint_positions
    res = ", ".join([f"{j}={pos.get(j, 0.0):.4f}" for j in known if j in pos])
    return f"Current joints (rad): {res}"


def read_current_pose(timeout_sec: float = 2.0):
    """Read the live position of all 6 motors (5 arm joints + gripper) from
    /joint_states, for use as a taught scene step. Arm joints are returned
    in degrees (normal revolute joints); the gripper is returned in
    radians directly, matching the units used on the Joint & Gripper tab,
    since it's a multi-turn lead-screw actuator where degrees don't map
    intuitively.

    Returns (joints_deg, gripper_rad) on success, or None on timeout / if
    any joint is missing from the latest /joint_states message (e.g. the
    hardware hasn't published yet, or a joint name doesn't match)."""
    if not node.joint_states_received.wait(timeout=timeout_sec):
        return None
    pos = node.current_joint_positions
    if not all(j in pos for j in ARM_JOINTS) or GRIPPER_JOINTS[0] not in pos:
        return None
    joints_deg = [math.degrees(pos[j]) for j in ARM_JOINTS]
    gripper_rad = pos[GRIPPER_JOINTS[0]]
    return joints_deg, gripper_rad


# ---------------------------------------------------------------------------
# Gemini messaging — NOT cached with st.cache_resource. The google-genai
# SDK's client holds an HTTP transport that can end up permanently closed
# across Streamlit reruns (especially with the background playback thread
# triggering frequent reruns), so a cached, long-lived client/chat object
# is unreliable here. Instead we create a brand-new client + chat for
# every single message and pass the prior turns in explicitly via
# `history`, so no state depends on a client instance surviving between
# reruns.
# ---------------------------------------------------------------------------
def get_gemini_api_key():
    dotenv.load_dotenv(dotenv.find_dotenv())
    return os.environ.get("GOOGLE_API_KEY")


def send_gemini_message(prompt: str, history: list) -> tuple[str, list]:
    """Send one message to Gemini using a fresh client/chat, seeded with
    the prior conversation `history`. Returns (reply_text, new_history) so
    the caller can persist the updated history in session_state.

    Raises on failure — the caller is responsible for catching and
    displaying errors; no silent retries happen here since a fresh client
    is used every time, which already avoids the stale-client problem."""
    api_key = get_gemini_api_key()
    if not api_key:
        raise RuntimeError("GOOGLE_API_KEY environment variable is not set.")

    client = genai.Client(api_key=api_key)
    chat = client.chats.create(
        model=GEMINI_MODEL_NAME,
        config=types.GenerateContentConfig(
            system_instruction=SYSTEM_PROMPT,
            tools=[move_arm_joints, set_gripper_position, move_to_cartesian_xyz, retrieve_joint_states],
        ),
        history=history,
    )
    response = chat.send_message(prompt)
    new_history = chat.get_history()
    return response.text, new_history


# ---------------------------------------------------------------------------
# Scene playback — runs in a background thread so it never blocks the
# Streamlit UI. Progress/status are written to session_state and polled
# by the main script on each rerun.
# ---------------------------------------------------------------------------
def _playback_worker(scene_name: str, steps: list, status: dict, stop_event: threading.Event) -> None:
    total = len(steps)
    for i, step in enumerate(steps):
        if stop_event.is_set():
            status["state"] = "stopped"
            status["message"] = f"Playback stopped at step {i + 1}/{total}."
            return

        status["state"] = "running"
        status["current_step"] = i + 1
        status["total_steps"] = total

        joints_rad = [math.radians(d) for d in step["joints_deg"]]
        gripper_rad = step["gripper_rad"]
        ok = node.send_step(joints_rad, gripper_rad, step["duration_sec"])
        if not ok:
            status["state"] = "error"
            status["message"] = f"Step {i + 1} failed — playback stopped."
            return

        # Wait out the step duration, but check the stop flag periodically
        # so a "Stop" click takes effect quickly instead of waiting for
        # the full duration to elapse.
        remaining = step["duration_sec"]
        poll_interval = 0.1
        while remaining > 0:
            if stop_event.is_set():
                status["state"] = "stopped"
                status["message"] = f"Playback stopped at step {i + 1}/{total}."
                return
            sleep_for = min(poll_interval, remaining)
            time.sleep(sleep_for)
            remaining -= sleep_for

    status["state"] = "done"
    status["message"] = f"Scene '{scene_name}' finished ({total} step(s))."


def start_playback(scene_name: str, steps: list) -> None:
    stop_event = threading.Event()
    status = {"state": "starting", "current_step": 0, "total_steps": len(steps), "message": ""}
    st.session_state.playback_status = status
    st.session_state.playback_stop_event = stop_event
    thread = threading.Thread(
        target=_playback_worker, args=(scene_name, steps, status, stop_event), daemon=True
    )
    st.session_state.playback_thread = thread
    thread.start()


# ---------------------------------------------------------------------------
# Streamlit Interface
# ---------------------------------------------------------------------------
st.set_page_config(page_title="Dynamixel Arm Controller", layout="centered")
st.title("🤖 Dynamixel Arm Controller")

if "scenes" not in st.session_state:
    st.session_state.scenes = load_scenes()
if "playback_status" not in st.session_state:
    st.session_state.playback_status = None
if "playback_stop_event" not in st.session_state:
    st.session_state.playback_stop_event = None
if "playback_thread" not in st.session_state:
    st.session_state.playback_thread = None

tab1, tab2, tab3, tab4 = st.tabs(["Cartesian (XYZ)", "Joint & Gripper", "🎬 Scene Recorder", "🤖 Gemini AI"])

# --- TAB 1: CARTESIAN (XYZ) CONTROL ---
with tab1:
    st.subheader("Cartesian Target Controls")
    step_size = st.number_input(
        "Step Distance per Click (m):", min_value=0.001, max_value=0.100, value=0.010, step=0.005, format="%.3f"
    )

    if "x_val" not in st.session_state: st.session_state.x_val = 0.20
    if "y_val" not in st.session_state: st.session_state.y_val = 0.00
    if "z_val" not in st.session_state: st.session_state.z_val = 0.15

    def adjust_val(axis, delta):
        if axis == "X": st.session_state.x_val = round(max(-0.5, min(0.5, st.session_state.x_val + delta)), 3)
        elif axis == "Y": st.session_state.y_val = round(max(-0.5, min(0.5, st.session_state.y_val + delta)), 3)
        elif axis == "Z": st.session_state.z_val = round(max(0.0, min(0.67, st.session_state.z_val + delta)), 3)

    cols = st.columns([1, 2, 1])
    cols[0].button("◄ Back (-X)", on_click=adjust_val, args=("X", -step_size), key="btn_x_minus")
    st.session_state.x_val = cols[1].number_input("X Axis", min_value=-0.5, max_value=0.5, value=st.session_state.x_val, step=0.005, format="%.3f", key="input_x")
    cols[2].button("Forward (+X) ►", on_click=adjust_val, args=("X", step_size), key="btn_x_plus")

    cols = st.columns([1, 2, 1])
    cols[0].button("◄ Right (-Y)", on_click=adjust_val, args=("Y", -step_size), key="btn_y_minus")
    st.session_state.y_val = cols[1].number_input("Y Axis", min_value=-0.5, max_value=0.5, value=st.session_state.y_val, step=0.005, format="%.3f", key="input_y")
    cols[2].button("Left (+Y) ►", on_click=adjust_val, args=("Y", step_size), key="btn_y_plus")

    cols = st.columns([1, 2, 1])
    cols[0].button("◄ Down (-Z)", on_click=adjust_val, args=("Z", -step_size), key="btn_z_minus")
    st.session_state.z_val = cols[1].number_input("Z Axis", min_value=0.0, max_value=0.67, value=st.session_state.z_val, step=0.005, format="%.3f", key="input_z")
    cols[2].button("Up (+Z) ►", on_click=adjust_val, args=("Z", step_size), key="btn_z_plus")

    st.markdown("---")
    col_b1, col_b2 = st.columns(2)
    with col_b1:
        if st.button("Send XYZ Target", type="primary", use_container_width=True):
            node.send_target_xyz(st.session_state.x_val, st.session_state.y_val, st.session_state.z_val)
            st.success("Target XYZ sent successfully!")
    with col_b2:
        if st.button("Reset Default Coordinates", use_container_width=True):
            st.session_state.x_val, st.session_state.y_val, st.session_state.z_val = 0.20, 0.00, 0.15
            st.rerun()

# --- TAB 2: JOINT & GRIPPER CONTROL ---
with tab2:
    st.subheader("Arm Joint Positions (rad)")
    joints_config = [("j1", -3.14, 3.14), ("j2", -1.57, 1.57), ("j3", -1.57, 1.57), ("j4", -3.14, 3.14), ("j5", -1.57, 1.57)]
    arm_values = [st.slider(name, min_value=float(mn), max_value=float(mx), value=0.0, step=0.02) for name, mn, mx in joints_config]
    arm_duration = st.slider("Trajectory Duration (s):", min_value=0.5, max_value=10.0, value=3.0, step=0.5)

    st.markdown("---")
    st.subheader("Gripper Control")
    gripper_theta = st.number_input("Gripper Position (rad):", min_value=-65.5, max_value=94.9, value=0.0, step=0.1)

    st.markdown("---")
    cb1, cb2, cb3 = st.columns(3)
    with cb1:
        if st.button("Send Arm Only", use_container_width=True):
            if node.send_arm_only(arm_values, arm_duration):
                st.success("Arm command sent!")
            else:
                st.error("Arm goal was not accepted.")
    with cb2:
        if st.button("Send Gripper Only", use_container_width=True):
            if node.send_gripper_only(gripper_theta, 2.0):
                st.success("Gripper command sent!")
            else:
                st.error("Gripper goal was not accepted.")
    with cb3:
        if st.button("Send All", type="primary", use_container_width=True):
            node.send_commands(arm_values, gripper_theta, arm_duration)
            st.success("All targets sent!")

# --- TAB 3: SCENE RECORDER ---
with tab3:
    st.subheader("Scene")
    scene_names = list(st.session_state.scenes.keys())
    col1, col2 = st.columns([2, 1])
    with col1:
        selected_scene = st.selectbox("Select a scene", ["-- new scene --"] + scene_names)
    with col2:
        if selected_scene != "-- new scene --" and st.button("🗑️ Delete scene", use_container_width=True):
            del st.session_state.scenes[selected_scene]
            save_scenes(st.session_state.scenes)
            st.rerun()

    if selected_scene == "-- new scene --":
        new_name = st.text_input("New scene name")
        if st.button("Create scene") and new_name:
            if new_name not in st.session_state.scenes:
                st.session_state.scenes[new_name] = []
                save_scenes(st.session_state.scenes)
            st.rerun()
    else:
        current_scene = st.session_state.scenes[selected_scene]

        st.subheader(f"Steps in '{selected_scene}' ({len(current_scene)})")
        if not current_scene:
            st.info("No steps yet. Add one below.")
        for i, step in enumerate(current_scene):
            j = step["joints_deg"]
            with st.expander(f"Step {i + 1}: joints={j}, gripper={step['gripper_rad']} rad, {step['duration_sec']}s"):
                if st.button("🗑️ Delete this step", key=f"del_{i}"):
                    current_scene.pop(i)
                    save_scenes(st.session_state.scenes)
                    st.rerun()

        st.markdown("---")
        st.subheader("🖐️ Dạy vị trí (Teach by hand)")
        st.caption(
            "Cầm tay kéo robot đến vị trí mong muốn (nhớ tắt torque nếu cần), "
            "rồi bấm nút bên dưới để đọc vị trí thật của cả 6 motor."
        )

        if "taught_pose" not in st.session_state:
            st.session_state.taught_pose = None

        if st.button("📸 Đọc vị trí hiện tại", use_container_width=True):
            result = read_current_pose()
            if result is None:
                st.error(
                    "Không đọc được vị trí — kiểm tra /joint_states có đang publish "
                    "đủ tên khớp (5 arm joints + gripper) không."
                )
                st.session_state.taught_pose = None
            else:
                joints_deg, gripper_rad = result
                st.session_state.taught_pose = {
                    "joints_deg": [round(v, 2) for v in joints_deg],
                    "gripper_rad": round(gripper_rad, 3),
                }

        if st.session_state.taught_pose is not None:
            tp = st.session_state.taught_pose
            st.success(
                f"Đã đọc: joints={tp['joints_deg']}°, gripper={tp['gripper_rad']} rad"
            )
            teach_duration = st.number_input(
                "Thời gian di chuyển tới step này (s)",
                min_value=0.2, max_value=30.0, value=2.0, step=0.5,
                key="teach_duration",
            )
            col_t1, col_t2 = st.columns(2)
            with col_t1:
                if st.button("➕ Thêm vào scene", type="primary", use_container_width=True, key="add_taught_step"):
                    current_scene.append({
                        "joints_deg": tp["joints_deg"],
                        "gripper_rad": tp["gripper_rad"],
                        "duration_sec": teach_duration,
                    })
                    save_scenes(st.session_state.scenes)
                    st.session_state.taught_pose = None
                    st.rerun()
            with col_t2:
                if st.button("Hủy", use_container_width=True, key="cancel_taught_step"):
                    st.session_state.taught_pose = None
                    st.rerun()

        st.markdown("---")
        st.subheader("Add a step")
        st.caption("Enter joint angles in degrees.")

        cols = st.columns(5)
        joint_vals_deg = [
            cols[k].number_input(f"j{k + 1} (°)", min_value=-180.0, max_value=180.0, value=0.0, step=1.0, key=f"add_j{k}")
            for k in range(5)
        ]
        gripper_val_rad = st.number_input("Gripper (rad)", min_value=-65.5, max_value=94.9, value=0.0, step=0.5)
        duration_sec = st.number_input("Duration (s)", min_value=0.2, max_value=30.0, value=2.0, step=0.5)

        if st.button("➕ Add step", type="primary"):
            current_scene.append({
                "joints_deg": joint_vals_deg,
                "gripper_rad": gripper_val_rad,
                "duration_sec": duration_sec,
            })
            save_scenes(st.session_state.scenes)
            st.rerun()

        st.markdown("---")
        st.subheader("Playback")

        playback_active = (
            st.session_state.playback_thread is not None
            and st.session_state.playback_thread.is_alive()
        )

        col_p1, col_p2 = st.columns(2)
        with col_p1:
            if st.button(
                "▶️ Play scene",
                type="primary",
                disabled=(len(current_scene) == 0 or playback_active),
                use_container_width=True,
            ):
                start_playback(selected_scene, current_scene)
                st.rerun()
        with col_p2:
            if st.button("⏹️ Stop", disabled=not playback_active, use_container_width=True):
                if st.session_state.playback_stop_event is not None:
                    st.session_state.playback_stop_event.set()
                st.rerun()

        status = st.session_state.playback_status
        if status is not None:
            state = status.get("state")
            total = status.get("total_steps", 0)
            current = status.get("current_step", 0)

            if state in ("starting", "running"):
                frac = (current / total) if total else 0.0
                st.progress(frac, text=f"Step {current}/{total}")
            elif state == "done":
                st.success(status.get("message", "Scene finished."))
            elif state == "stopped":
                st.warning(status.get("message", "Playback stopped."))
            elif state == "error":
                st.error(status.get("message", "Playback error."))

            # Auto-refresh the UI while playback is running so the progress
            # bar updates without the user needing to click anything.
            if playback_active:
                time.sleep(0.3)
                st.rerun()

# --- TAB 4: GEMINI AI ASSISTANT ---
with tab4:
    st.subheader("Gemini Natural Language Assistant")

    if not get_gemini_api_key():
        st.error("GOOGLE_API_KEY environment variable is not set.")

    if "chat_history" not in st.session_state:
        st.session_state.chat_history = []
    if "gemini_history" not in st.session_state:
        st.session_state.gemini_history = []

    for msg in st.session_state.chat_history:
        with st.chat_message(msg["role"]):
            st.markdown(msg["content"])

    if prompt := st.chat_input("Enter command for robot..."):
        st.session_state.chat_history.append({"role": "user", "content": prompt})
        with st.chat_message("user"):
            st.markdown(prompt)

        with st.chat_message("assistant"):
            with st.spinner("Gemini is processing command..."):
                try:
                    reply_text, new_history = send_gemini_message(prompt, st.session_state.gemini_history)
                    st.session_state.gemini_history = new_history
                except Exception as e:
                    reply_text = f"API Error: {e}"

            st.markdown(reply_text)
            st.session_state.chat_history.append({"role": "assistant", "content": reply_text})
