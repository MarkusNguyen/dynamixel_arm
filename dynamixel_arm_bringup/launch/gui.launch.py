#!/usr/bin/env python3

import sys
import threading
import rclpy
from rclpy.node import Node
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
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10
        )
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper']

    def send_trajectory(self, positions, time_sec=2.0):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        
        sec = int(time_sec)
        nanosec = int((time_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published target positions: {positions}')


class JointControlGUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Dynamixel Arm - Trajectory Controller')
        self.setMinimumWidth(480)

        central_widget = QWidget()
        main_layout = QVBoxLayout()

        # Joint config: (Name, Min (rad), Max (rad), Default)
        self.joint_configs = [
            ('joint1', -3.14, 3.14, 0.0),
            ('joint2', -1.57, 1.57, 0.0),
            ('joint3', -1.57, 1.57, 0.0),
            ('joint4', -3.14, 3.14, 0.0),
            ('joint5', -1.57, 1.57, 0.0),
            ('gripper', -0.57595, 0.38397, 0.0)
        ]

        self.sliders = []
        self.spinboxes = []

        group_box = QGroupBox("Joint Positions (radians)")
        group_layout = QVBoxLayout()

        for name, min_val, max_val, default_val in self.joint_configs:
            row_layout = QHBoxLayout()
            
            label = QLabel(name)
            label.setFixedWidth(70)

            spinbox = QDoubleSpinBox()
            spinbox.setRange(min_val, max_val)
            spinbox.setSingleStep(0.02)
            spinbox.setDecimals(2)
            spinbox.setValue(default_val)
            spinbox.setFixedWidth(80)

            slider = QSlider(Qt.Horizontal)
            # Scale float range to integer steps for QSlider
            slider.setRange(int(min_val * 100), int(max_val * 100))
            slider.setValue(int(default_val * 100))

            # Synchronize slider and spinbox
            spinbox.valueChanged.connect(
                lambda val, s=slider: s.setValue(int(val * 100))
            )
            slider.valueChanged.connect(
                lambda val, sb=spinbox: sb.setValue(val / 100.0)
            )

            self.spinboxes.append(spinbox)
            self.sliders.append(slider)

            row_layout.addWidget(label)
            row_layout.addWidget(slider)
            row_layout.addWidget(spinbox)
            group_layout.addLayout(row_layout)

        group_box.setLayout(group_layout)
        main_layout.addWidget(group_box)

        # Trajectory execution duration parameter
        duration_layout = QHBoxLayout()
        duration_label = QLabel("Time to reach target (seconds):")
        self.duration_spinbox = QDoubleSpinBox()
        self.duration_spinbox.setRange(0.2, 10.0)
        self.duration_spinbox.setValue(2.0)
        self.duration_spinbox.setSingleStep(0.5)
        duration_layout.addWidget(duration_label)
        duration_layout.addWidget(self.duration_spinbox)
        main_layout.addLayout(duration_layout)

        # Action buttons
        button_layout = QHBoxLayout()
        
        self.home_button = QPushButton("Reset to Home")
        self.home_button.clicked.connect(self.on_home_clicked)

        self.send_button = QPushButton("Send Trajectory")
        self.send_button.setStyleSheet("font-weight: bold;")
        self.send_button.clicked.connect(self.on_send_clicked)

        button_layout.addWidget(self.home_button)
        button_layout.addWidget(self.send_button)
        main_layout.addLayout(button_layout)

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def on_send_clicked(self):
        positions = [sb.value() for sb in self.spinboxes]
        duration = self.duration_spinbox.value()
        self.node.send_trajectory(positions, duration)

    def on_home_clicked(self):
        for i, (_, _, _, default_val) in enumerate(self.joint_configs):
            self.spinboxes[i].setValue(default_val)
        self.on_send_clicked()

    def closeEvent(self, event):
        rclpy.shutdown()
        event.accept()


def main():
    rclpy.init()
    node = ArmControllerNode()

    # Spin ROS 2 executor in a background thread to prevent blocking PyQt main loop
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = JointControlGUI(node)
    gui.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()  