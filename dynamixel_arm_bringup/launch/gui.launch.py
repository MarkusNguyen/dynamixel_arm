#!/usr/bin/env python3

import sys, threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QDoubleSpinBox, QPushButton, QGroupBox
)
from PyQt5.QtCore import Qt


class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_gui_controller')

        self.arm = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self.gripper = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )

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


class JointControlGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Dynamixel Arm Controller')
        self.setMinimumWidth(480)

        layout = QVBoxLayout()
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
        layout.addWidget(group)

        # Time from start
        time_row = QHBoxLayout()
        time_row.addWidget(QLabel('Time to target (s):'))

        self.time = QDoubleSpinBox()
        self.time.setRange(0.2, 10.0)
        self.time.setValue(2.0)
        self.time.setSingleStep(0.5)
        self.time.setDecimals(2)

        time_row.addWidget(self.time)
        layout.addLayout(time_row)

        # Buttons
        buttons = QHBoxLayout()

        home = QPushButton('Reset to Home')
        home.clicked.connect(self.home)

        send = QPushButton('Send Trajectory')
        send.clicked.connect(self.send)

        buttons.addWidget(home)
        buttons.addWidget(send)
        layout.addLayout(buttons)

        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

    def send(self):
        pos = [s.value() for s in self.spinboxes]
        self.node.send_commands(
            pos[:5],
            pos[5:],
            self.time.value()
        )

    def home(self):
        for i, (_, _, _, default) in enumerate(self.config):
            self.spinboxes[i].setValue(default)
        self.send()

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