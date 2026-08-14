#!/usr/bin/env python3

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point
from builtin_interfaces.msg import Duration

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QDoubleSpinBox, QPushButton, QGroupBox, QTabWidget
)
from PyQt5.QtCore import Qt


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

        # Cartesian topic publisher for MoveTrajectory node
        self.xyz_pub = self.create_publisher(Point, '/target_xyz', 10)

        self.arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.gripper_joints = ['gripper']

    def send_trajectory(self, client, joints, positions, time_sec):
        if not client.wait_for_server(timeout_sec=0.5):
            self.get_logger().error('Action server not available')
            return

        sec = int(time_sec)
        nsec = int((time_sec - sec) * 1e9)

        traj = JointTrajectory()
        traj.joint_names = joints

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=sec, nanosec=nsec)

        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        client.send_goal_async(goal)

    def send_commands(self, arm_pos, gripper_pos, time_sec):
        self.send_trajectory(
            self.arm, self.arm_joints, arm_pos, time_sec
        )
        self.send_trajectory(
            self.gripper, self.gripper_joints, gripper_pos, time_sec
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
        self.setWindowTitle('Dynamixel Arm Controller (XYZ & Joint)')
        self.setMinimumWidth(580)

        main_layout = QVBoxLayout()
        tabs = QTabWidget()

        # ==================== TAB 1: CARTESIAN (XYZ) ARROW CONTROL ====================
        xyz_tab = QWidget()
        xyz_layout = QVBoxLayout()

        xyz_group = QGroupBox('Cartesian Target Controls (Jogging)')
        xyz_grid = QVBoxLayout()

        # Step Size Selector
        step_row = QHBoxLayout()
        step_row.addWidget(QLabel('Step Distance per Click (m):'))
        
        self.xyz_step_spin = QDoubleSpinBox()
        self.xyz_step_spin.setRange(0.001, 0.100)
        self.xyz_step_spin.setValue(0.010)  # Default 1 cm step
        self.xyz_step_spin.setSingleStep(0.005)
        self.xyz_step_spin.setDecimals(3)
        self.xyz_step_spin.setFixedWidth(100)
        
        step_row.addWidget(self.xyz_step_spin)
        step_row.addStretch()
        xyz_grid.addLayout(step_row)

        self.xyz_spinboxes = {}
        # Format: (axis, min, max, default, neg_label, pos_label)
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

            # Button click callbacks using current step size
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

        # ==================== TAB 2: JOINT POSITIONS CONTROL ====================
        joint_tab = QWidget()
        joint_layout = QVBoxLayout()

        group = QGroupBox('Joint Positions (rad)')
        joints_layout = QVBoxLayout()

        self.config = [
            ('joint1', -3.14, 3.14, 0.0),
            ('joint2', -1.57, 1.57, 0.0),
            ('joint3', -1.57, 1.57, 0.0),
            ('joint4', -3.14, 3.14, 0.0),
            ('joint5', -1.57, 1.57, 0.0),
            ('gripper', -0.57595, 0.38397, 0.0)
        ]

        self.spinboxes = []

        for name, mn, mx, default in self.config:
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

            spin.valueChanged.connect(
                lambda v, s=slider: s.setValue(round(v * 100))
            )
            slider.valueChanged.connect(
                lambda v, s=spin: s.setValue(v / 100)
            )

            self.spinboxes.append(spin)

            row.addWidget(label)
            row.addWidget(slider)
            row.addWidget(spin)
            joints_layout.addLayout(row)

        group.setLayout(joints_layout)
        joint_layout.addWidget(group)

        # Joint Movement Time Settings
        time_row = QHBoxLayout()
        time_row.addWidget(QLabel('Trajectory Movement Duration (s):'))

        self.time = QDoubleSpinBox()
        self.time.setRange(0.5, 10.0)
        self.time.setValue(2.0)  # Default 3.0s duration per command
        self.time.setSingleStep(0.5)
        self.time.setDecimals(1)
        self.time.setFixedWidth(80)

        time_row.addWidget(self.time)
        time_row.addStretch()
        joint_layout.addLayout(time_row)

        # Joint Control Buttons
        buttons = QHBoxLayout()

        btn_send_joints = QPushButton('Send Joint Target')
        btn_send_joints.setStyleSheet("background-color: #2b7cff; color: white; font-weight: bold; padding: 6px;")
        btn_send_joints.clicked.connect(self.send_joint_target)

        home = QPushButton('Reset Joints to Home')
        home.clicked.connect(self.home)

        buttons.addWidget(btn_send_joints)
        buttons.addWidget(home)
        joint_layout.addLayout(buttons)

        joint_tab.setLayout(joint_layout)
        tabs.addTab(joint_tab, "Joint Control")

        # Set central widget
        main_layout.addWidget(tabs)
        widget = QWidget()
        widget.setLayout(main_layout)
        self.setCentralWidget(widget)

    # --- XYZ Control Methods ---
    def send_xyz(self):
        """Sends XYZ coordinate goal to MoveTrajectory node via /target_xyz topic once."""
        x = self.xyz_spinboxes['X'].value()
        y = self.xyz_spinboxes['Y'].value()
        z = self.xyz_spinboxes['Z'].value()
        self.node.send_target_xyz(x, y, z)

    def reset_xyz(self):
        defaults = {'X': 0.0, 'Y': 0.0, 'Z': 0.67}
        for axis, default in defaults.items():
            self.xyz_spinboxes[axis].setValue(default)

    # --- Joint Control Methods ---
    def send_joint_target(self):
        """Sends joint command once on demand."""
        duration = self.time.value()
        pos = [s.value() for s in self.spinboxes]
        self.node.send_commands(
            pos[:5],
            pos[5:],
            duration
        )

    def home(self):
        for i, (_, _, _, default) in enumerate(self.config):
            self.spinboxes[i].setValue(default)

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