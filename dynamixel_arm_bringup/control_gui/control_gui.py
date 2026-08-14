#!/usr/bin/env python3

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QDoubleSpinBox, QPushButton, QGroupBox, QTabWidget
)
from PyQt5.QtCore import Qt, QTimer


class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_gui_controller')

        # Joint action clients
        self.arm = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self.gripper = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )

        # Joint State Subscriber
        self.current_gripper_pos = 0.0
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Cartesian topic publisher for MoveTrajectory node
        self.xyz_pub = self.create_publisher(Point, '/target_xyz', 10)

        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.gripper_joints = ['gripper']

    def joint_state_callback(self, msg: JointState):
        """Updates current position of the gripper directly from /joint_states."""
        if 'gripper' in msg.name:
            idx = msg.name.index('gripper')
            if idx < len(msg.position):
                self.current_gripper_pos = float(msg.position[idx])

    def send_trajectory(self, client, joints, positions, velocities=None, time_sec=1.0):
        if not client.wait_for_server(timeout_sec=0.5):
            self.get_logger().error(f'Action server for {joints} not available')
            return

        sec = int(time_sec)
        nsec = int((time_sec - sec) * 1e9)

        traj = JointTrajectory()
        traj.joint_names = joints

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        
        if velocities is not None:
            point.velocities = [float(v) for v in velocities]

        point.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        client.send_goal_async(goal)

    def send_arm_command(self, arm_pos, time_sec):
        self.send_trajectory(
            self.arm, self.arm_joints, arm_pos, time_sec=time_sec
        )

    def send_gripper_command(self, gripper_pos, time_sec):
        self.send_trajectory(
            self.gripper, self.gripper_joints, 
            positions=[gripper_pos], 
            time_sec=time_sec
        )

    def send_target_xyz(self, x: float, y: float, z: float):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.xyz_pub.publish(msg)
        self.get_logger().info(f'Published target XYZ: ({x:.3f}, {y:.3f}, {z:.3f})')


class JointControlGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Dynamixel Arm Controller (XYZ, Joint & Gripper Velocity)')
        self.setMinimumWidth(600)

        main_layout = QVBoxLayout()
        tabs = QTabWidget()

        # ==================== TAB 1: CARTESIAN (XYZ) CONTROL ====================
        xyz_tab = QWidget()
        xyz_layout = QVBoxLayout()

        xyz_group = QGroupBox('Cartesian Target Controls (Jogging)')
        xyz_grid = QVBoxLayout()

        # Step Size Selector
        step_row = QHBoxLayout()
        step_row.addWidget(QLabel('Step Distance per Click (m):'))
        
        self.xyz_step_spin = QDoubleSpinBox()
        self.xyz_step_spin.setRange(0.001, 0.100)
        self.xyz_step_spin.setValue(0.010)
        self.xyz_step_spin.setSingleStep(0.005)
        self.xyz_step_spin.setDecimals(3)
        self.xyz_step_spin.setFixedWidth(100)
        
        step_row.addWidget(self.xyz_step_spin)
        step_row.addStretch()
        xyz_grid.addLayout(step_row)

        self.xyz_spinboxes = {}
        cart_config = [
            ('X', -0.5, 0.5, 0.0, '◄ Back (-X)', 'Forward (+X) ►'),
            ('Y', -0.5, 0.5, 0.0, '◄ Right (-Y)', 'Left (+Y) ►'),
            ('Z', 0.00, 0.67, 0.67, '◄ Down (-Z)', 'Up (+Z) ►')
        ]

        for axis, mn, mx, default, neg_label, pos_label in cart_config:
            row = QHBoxLayout()
            label = QLabel(f'{axis} Axis:')
            label.setFixedWidth(50)

            spin = QDoubleSpinBox()
            spin.setRange(mn, mx)
            spin.setSingleStep(0.005)
            spin.setDecimals(3)
            spin.setValue(default)
            spin.setFixedWidth(85)
            self.xyz_spinboxes[axis] = spin

            btn_minus = QPushButton(neg_label)
            btn_plus = QPushButton(pos_label)

            btn_minus.clicked.connect(lambda _, s=spin: s.setValue(s.value() - self.xyz_step_spin.value()))
            btn_plus.clicked.connect(lambda _, s=spin: s.setValue(s.value() + self.xyz_step_spin.value()))

            row.addWidget(label)
            row.addWidget(btn_minus)
            row.addWidget(spin)
            row.addWidget(btn_plus)
            xyz_grid.addLayout(row)

        xyz_group.setLayout(xyz_grid)
        xyz_layout.addWidget(xyz_group)

        # Action Buttons for XYZ
        xyz_buttons = QHBoxLayout()
        btn_send_xyz = QPushButton('Send XYZ Target')
        btn_send_xyz.setStyleSheet("background-color: #2b7cff; color: white; font-weight: bold; padding: 6px;")
        btn_send_xyz.clicked.connect(self.send_xyz)

        btn_reset_xyz = QPushButton('Reset XYZ Default')
        btn_reset_xyz.clicked.connect(self.reset_xyz)

        xyz_buttons.addWidget(btn_send_xyz)
        xyz_buttons.addWidget(btn_reset_xyz)

        xyz_layout.addLayout(xyz_buttons)
        xyz_tab.setLayout(xyz_layout)
        tabs.addTab(xyz_tab, "Cartesian (XYZ)")

        # ==================== TAB 2: JOINT & GRIPPER CONTROL ====================
        joint_tab = QWidget()
        joint_layout = QVBoxLayout()

        # 1. Arm Joints Group
        arm_group = QGroupBox('Arm Joint Positions (rad)')
        arm_joints_layout = QVBoxLayout()

        self.arm_config = [
            ('joint1', -3.14, 3.14, 0.0),
            ('joint2', -1.57, 1.57, 0.0),
            ('joint3', -1.57, 1.57, 0.0),
            ('joint4', -3.14, 3.14, 0.0),
            ('joint5', -1.57, 1.57, 0.0)
        ]

        self.arm_spinboxes = []

        for name, mn, mx, default in self.arm_config:
            row = QHBoxLayout()
            label = QLabel(name)
            label.setFixedWidth(70)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(mn * 100), int(mx * 100))
            slider.setValue(int(default * 100))

            spin = QDoubleSpinBox()
            spin.setRange(mn, mx)
            spin.setSingleStep(0.02)
            spin.setDecimals(2)
            spin.setValue(default)
            spin.setFixedWidth(80)

            spin.valueChanged.connect(lambda v, s=slider: s.setValue(round(v * 100)))
            slider.valueChanged.connect(lambda v, s=spin: s.setValue(v / 100.0))

            self.arm_spinboxes.append(spin)

            row.addWidget(label)
            row.addWidget(slider)
            row.addWidget(spin)
            arm_joints_layout.addLayout(row)

        arm_time_row = QHBoxLayout()
        arm_time_row.addWidget(QLabel('Arm Trajectory Duration (s):'))
        self.arm_time = QDoubleSpinBox()
        self.arm_time.setRange(0.5, 10.0)
        self.arm_time.setValue(4.0)
        self.arm_time.setSingleStep(0.5)
        self.arm_time.setDecimals(1)
        self.arm_time.setFixedWidth(80)
        arm_time_row.addWidget(self.arm_time)
        arm_time_row.addStretch()

        arm_joints_layout.addLayout(arm_time_row)
        arm_group.setLayout(arm_joints_layout)
        joint_layout.addWidget(arm_group)

        # 2. Gripper Group (Velocity Command Interface)
        gripper_group = QGroupBox('Gripper Control (Theta & Velocity)')
        gripper_layout = QVBoxLayout()

        # Gripper Theta Control (-65.5269905 to 94.98867187 rad)
        g_theta_row = QHBoxLayout()
        g_theta_row.addWidget(QLabel('Theta (rad):'))
        
        self.gripper_spin = QDoubleSpinBox()
        self.gripper_spin.setRange(-65.5269905, 94.98867187) # Gripper Multiplier
        self.gripper_spin.setValue(0.0)
        self.gripper_spin.setSingleStep(0.1)
        self.gripper_spin.setDecimals(3)
        self.gripper_spin.setFixedWidth(100)

        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setRange(int(-65.5269905 * 1000), int(94.98867187 * 1000)) # Gripper Multiplier
        self.gripper_slider.setValue(0)

        self.gripper_spin.valueChanged.connect(
            lambda v: self.gripper_slider.setValue(round(v * 1000))
        )
        self.gripper_slider.valueChanged.connect(
            lambda v: self.gripper_spin.setValue(v / 1000.0)
        )
        self.gripper_spin.valueChanged.connect(self.update_gripper_duration_info)

        g_theta_row.addWidget(self.gripper_slider)
        g_theta_row.addWidget(self.gripper_spin)
        gripper_layout.addLayout(g_theta_row)

        # Gripper Velocity Control (0.01 to 10.0 rad/s)
        g_vel_row = QHBoxLayout()
        g_vel_row.addWidget(QLabel('Velocity (rad/s):'))

        self.gripper_vel_spin = QDoubleSpinBox()
        self.gripper_vel_spin.setRange(0.01, 10.0)
        self.gripper_vel_spin.setValue(2.0)
        self.gripper_vel_spin.setSingleStep(0.2)
        self.gripper_vel_spin.setDecimals(2)
        self.gripper_vel_spin.setFixedWidth(100)

        self.gripper_vel_slider = QSlider(Qt.Horizontal)
        self.gripper_vel_slider.setRange(1, 1000)  # 0.01 to 10.00
        self.gripper_vel_slider.setValue(200)

        self.gripper_vel_spin.valueChanged.connect(
            lambda v: self.gripper_vel_slider.setValue(round(v * 100))
        )
        self.gripper_vel_slider.valueChanged.connect(
            lambda v: self.gripper_vel_spin.setValue(v / 100.0)
        )
        self.gripper_vel_spin.valueChanged.connect(self.update_gripper_duration_info)

        g_vel_row.addWidget(self.gripper_vel_slider)
        g_vel_row.addWidget(self.gripper_vel_spin)
        gripper_layout.addLayout(g_vel_row)

        # Dynamic feedback & duration label
        self.gripper_dur_label = QLabel('Current Theta: 0.000 rad | Est. Duration: 0.50 s')
        self.gripper_dur_label.setStyleSheet("color: #333333; font-weight: bold;")
        gripper_layout.addWidget(self.gripper_dur_label)

        gripper_group.setLayout(gripper_layout)
        joint_layout.addWidget(gripper_group)

        # Action Buttons
        buttons = QHBoxLayout()

        btn_send_arm = QPushButton('Send Arm Only')
        btn_send_arm.clicked.connect(self.send_arm_target)

        btn_send_gripper = QPushButton('Send Gripper Only')
        btn_send_gripper.clicked.connect(self.send_gripper_target)

        btn_send_all = QPushButton('Send All Targets')
        btn_send_all.setStyleSheet("background-color: #2b7cff; color: white; font-weight: bold; padding: 6px;")
        btn_send_all.clicked.connect(self.send_all_targets)

        home = QPushButton('Reset Home')
        home.clicked.connect(self.home)

        buttons.addWidget(btn_send_arm)
        buttons.addWidget(btn_send_gripper)
        buttons.addWidget(btn_send_all)
        buttons.addWidget(home)
        joint_layout.addLayout(buttons)

        joint_tab.setLayout(joint_layout)
        tabs.addTab(joint_tab, "Joint & Gripper Control")

        # Set central widget
        main_layout.addWidget(tabs)
        widget = QWidget()
        widget.setLayout(main_layout)
        self.setCentralWidget(widget)

        # Periodic timer (100 ms) to refresh real-time position from /joint_states
        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self.update_gripper_duration_info)
        self.gui_timer.start(100)

    # --- Calculation Helpers ---
    def update_gripper_duration_info(self):
        target_theta = self.gripper_spin.value()
        velocity = max(0.01, self.gripper_vel_spin.value())
        current_pos = self.node.current_gripper_pos
        delta_theta = abs(target_theta - current_pos)
        duration = max(0.1, delta_theta / velocity) if delta_theta > 1e-4 else 0.5
        
        self.gripper_dur_label.setText(
            f'Current Theta: {current_pos:.3f} rad | Est. Duration: {duration:.2f} s'
        )

    # --- XYZ Control Methods ---
    def send_xyz(self):
        x = self.xyz_spinboxes['X'].value()
        y = self.xyz_spinboxes['Y'].value()
        z = self.xyz_spinboxes['Z'].value()
        self.node.send_target_xyz(x, y, z)

    def reset_xyz(self):
        defaults = {'X': 0.0, 'Y': 0.0, 'Z': 0.67}
        for axis, default in defaults.items():
            self.xyz_spinboxes[axis].setValue(default)

    # --- Joint & Gripper Control Methods ---
    def send_arm_target(self):
        arm_pos = [s.value() for s in self.arm_spinboxes]
        duration = self.arm_time.value()
        self.node.send_arm_command(arm_pos, duration)

    def send_gripper_target(self):
        target_theta = self.gripper_spin.value()
        velocity = self.gripper_vel_spin.value()
        current_pos = self.node.current_gripper_pos
        delta_theta = abs(target_theta - current_pos)
        
        # Calculate duration based on distance from real-time current position
        duration = max(0.2, delta_theta / max(velocity, 0.01)) if delta_theta > 1e-4 else 0.5
        
        self.node.send_gripper_command(target_theta, duration)

    def send_all_targets(self):
        self.send_arm_target()
        self.send_gripper_target()

    def home(self):
        for i, (_, _, _, default) in enumerate(self.arm_config):
            self.arm_spinboxes[i].setValue(default)
        self.gripper_spin.setValue(0.0)
        self.gripper_vel_spin.setValue(2.0)

    def closeEvent(self, event):
        if rclpy.ok():
            self.node.destroy_node()
            rclpy.shutdown()
        event.accept()


def main():
    rclpy.init()
    node = ArmControllerNode()

    threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    ).start()

    app = QApplication(sys.argv)
    gui = JointControlGUI(node)
    gui.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()