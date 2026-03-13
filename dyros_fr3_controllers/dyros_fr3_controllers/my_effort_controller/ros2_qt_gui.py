#!/usr/bin/env python3
import sys
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QLineEdit,
    QPushButton,
    QGridLayout,
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QSpacerItem,
    QSizePolicy,
)
from PyQt5.QtCore import QTimer

CONTROL_MODE_TOPIC = "/fr3_1/my_effort_controller/control_mode"
JOINT_STATES_TOPIC = "/fr3_1/joint_states"
EE_POSE_TOPIC = "/fr3_1/franka_robot_state_broadcaster/current_pose"


class Fr3QtRosNode(Node):
    def __init__(self) -> None:
        super().__init__("fr3_qt_guimy_effort_controller")

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publisher: control mode
        self.control_mode_pub = self.create_publisher(
            Int32,
            CONTROL_MODE_TOPIC,
            10,
        )

        # Subscriber: joint states
        self.joint_states_sub = self.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self.joint_states_callback,
            sensor_qos,
        )

        # Subscriber: EE pose
        self.ee_pose_sub = self.create_subscription(
            PoseStamped,
            EE_POSE_TOPIC,
            self.ee_pose_callback,
            sensor_qos,
        )

        self.latest_joint_data: Dict[str, Tuple[float, float, float]] = {}
        self.latest_ee_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.latest_ee_orientation: Tuple[float, float, float, float] = (
            0.0,
            0.0,
            0.0,
            1.0,
        )

    def joint_states_callback(self, msg: JointState) -> None:
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        target_joint_names = ["fr3_joint{}".format(i) for i in range(1, 8)]

        for jname in target_joint_names:
            if jname in name_to_index:
                idx = name_to_index[jname]
                pos = msg.position[idx] if len(msg.position) > idx else 0.0
                vel = msg.velocity[idx] if len(msg.velocity) > idx else 0.0
                eff = msg.effort[idx] if len(msg.effort) > idx else 0.0
                self.latest_joint_data[jname] = (pos, vel, eff)

    def ee_pose_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.latest_ee_position = (p.x, p.y, p.z)

        q = msg.pose.orientation
        self.latest_ee_orientation = (q.x, q.y, q.z, q.w)

    def publish_control_mode(self, value: int) -> None:
        msg = Int32()
        msg.data = value
        self.control_mode_pub.publish(msg)
        self.get_logger().info("Published control_mode: {}".format(value))


class Fr3QtGui(QWidget):
    def __init__(self, ros_node: Fr3QtRosNode) -> None:
        super().__init__()
        self.ros_node = ros_node

        self.joint_labels: Dict[str, Dict[str, QLabel]] = {}
        self.ee_pos_labels: Dict[str, QLabel] = {}
        self.ee_ori_labels: Dict[str, QLabel] = {}

        self.init_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_from_ros)
        self.timer.start(100)

    def init_ui(self) -> None:
        self.setWindowTitle("FR3 Control & Monitor GUI (my_effort_controller)")

        # --- Control mode group ---
        control_group = QGroupBox("Control Mode Sender")
        control_layout = QHBoxLayout()

        control_label = QLabel("Control mode (Int32):")
        self.control_mode_input = QLineEdit()
        send_button = QPushButton("Send")
        self.control_mode_status_label = QLabel("")

        send_button.clicked.connect(self.on_send_control_mode_clicked)

        control_layout.addWidget(control_label)
        control_layout.addWidget(self.control_mode_input)
        control_layout.addWidget(send_button)
        control_layout.addWidget(self.control_mode_status_label)

        control_group.setLayout(control_layout)

        # --- Joint states group ---
        joint_group = QGroupBox("FR3 Joint States (fr3_joint1..7)")
        joint_layout = QGridLayout()

        joint_layout.addWidget(QLabel("Joint"), 0, 0)
        joint_layout.addWidget(QLabel("Position"), 0, 1)
        joint_layout.addWidget(QLabel("Velocity"), 0, 2)
        joint_layout.addWidget(QLabel("Effort"), 0, 3)

        row = 1
        for joint_idx in range(1, 8):
            joint_name = "fr3_joint{}".format(joint_idx)

            name_label = QLabel(joint_name)
            pos_label = QLabel("0.0")
            vel_label = QLabel("0.0")
            eff_label = QLabel("0.0")

            self.joint_labels[joint_name] = {
                "pos": pos_label,
                "vel": vel_label,
                "eff": eff_label,
            }

            joint_layout.addWidget(name_label, row, 0)
            joint_layout.addWidget(pos_label, row, 1)
            joint_layout.addWidget(vel_label, row, 2)
            joint_layout.addWidget(eff_label, row, 3)
            row += 1

        joint_group.setLayout(joint_layout)

        # --- EE pose group ---
        ee_group = QGroupBox("End-Effector Pose")
        ee_layout = QGridLayout()

        ee_layout.addWidget(QLabel("Component"), 0, 0)
        ee_layout.addWidget(QLabel("Value"), 0, 1)

        pos_components = ["x", "y", "z"]
        row = 1
        for comp in pos_components:
            label_name = QLabel("Position {}:".format(comp))
            value_label = QLabel("0.0")
            self.ee_pos_labels[comp] = value_label
            ee_layout.addWidget(label_name, row, 0)
            ee_layout.addWidget(value_label, row, 1)
            row += 1

        spacer = QSpacerItem(0, 10, QSizePolicy.Minimum, QSizePolicy.Minimum)
        ee_layout.addItem(spacer, row, 0)
        row += 1

        ori_components = ["x", "y", "z", "w"]
        for comp in ori_components:
            label_name = QLabel("Orientation {}:".format(comp))
            value_label = QLabel("0.0")
            self.ee_ori_labels[comp] = value_label
            ee_layout.addWidget(label_name, row, 0)
            ee_layout.addWidget(value_label, row, 1)
            row += 1

        ee_group.setLayout(ee_layout)

        # --- Main layout ---
        main_layout = QVBoxLayout()
        main_layout.addWidget(control_group)
        main_layout.addWidget(joint_group)
        main_layout.addWidget(ee_group)

        self.setLayout(main_layout)
        self.resize(800, 400)

    def on_send_control_mode_clicked(self) -> None:
        text = self.control_mode_input.text().strip()
        if text == "":
            self.control_mode_status_label.setText("Ignored: empty input")
            return

        try:
            value = int(text)
        except ValueError:
            # int가 아닌 이상한 값이면 무시
            self.control_mode_status_label.setText("Ignored: not an integer")
            return

        self.ros_node.publish_control_mode(value)
        self.control_mode_status_label.setText("Sent: {}".format(value))

    def update_from_ros(self) -> None:
        # Joint states
        for joint_name, labels in self.joint_labels.items():
            if joint_name in self.ros_node.latest_joint_data:
                pos, vel, eff = self.ros_node.latest_joint_data[joint_name]
                labels["pos"].setText("{:.4f}".format(pos))
                labels["vel"].setText("{:.4f}".format(vel))
                labels["eff"].setText("{:.4f}".format(eff))

        # EE position
        px, py, pz = self.ros_node.latest_ee_position
        self.ee_pos_labels["x"].setText("{:.4f}".format(px))
        self.ee_pos_labels["y"].setText("{:.4f}".format(py))
        self.ee_pos_labels["z"].setText("{:.4f}".format(pz))

        # EE orientation (quaternion)
        ox, oy, oz, ow = self.ros_node.latest_ee_orientation
        self.ee_ori_labels["x"].setText("{:.4f}".format(ox))
        self.ee_ori_labels["y"].setText("{:.4f}".format(oy))
        self.ee_ori_labels["z"].setText("{:.4f}".format(oz))
        self.ee_ori_labels["w"].setText("{:.4f}".format(ow))


def main() -> None:
    rclpy.init(args=sys.argv)
    ros_node = Fr3QtRosNode()
    app = QApplication(sys.argv)

    gui = Fr3QtGui(ros_node)
    gui.show()

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    ros_timer.start(10)

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
