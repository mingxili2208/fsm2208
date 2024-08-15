#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ackermann_msgs.msg import AckermannDriveStamped

import sys
import os
from threading import Lock, Thread
import math

from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QDial, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt5.QtCore import Qt, QTimer, QObject, pyqtSignal
from PyQt5.QtGui import QPainter, QPen, QBrush

UP = "w"
LEFT = "a"
DOWN = "s"
RIGHT = "d"
QUIT = "q"

state = [False, False, False, False]
state_lock = Lock()
control = False

# 创建一个信号类，用于发送键盘中断信号
class KeyboardInterruptSignal(QObject):
    interrupt_signal = pyqtSignal()

class GaugeWidget(QWidget):
    def __init__(self, parent=None, gauge_name="Gauge", min_value=0, max_value=100, major_ticks=5, minor_ticks=10):
        super().__init__(parent)
        self.value = 0
        self.gauge_name = gauge_name
        self.min_value = min_value
        self.max_value = max_value
        self.major_ticks = major_ticks
        self.minor_ticks = minor_ticks

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Define geometry
        width = self.width()
        height = self.height()
        center_x = width / 2
        center_y = height / 2
        radius = min(center_x, center_y) * 0.8

        # Convert to integers
        center_x = int(center_x)
        center_y = int(center_y)
        radius = int(radius)

        # Draw gauge background
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(Qt.lightGray))
        painter.drawEllipse(center_x - radius, center_y - radius, 2 * radius, 2 * radius)

        # Draw ticks and labels
        painter.setPen(QPen(Qt.black, 1))
        for i in range(self.major_ticks + 1):
            angle = (i / self.major_ticks) * 270 - 135
            x1 = int(center_x + radius * 0.7 * math.cos(math.radians(angle)))  # 转换为整数
            y1 = int(center_y + radius * 0.7 * math.sin(math.radians(angle)))  # 转换为整数
            x2 = int(center_x + radius * 0.8 * math.cos(math.radians(angle)))  # 转换为整数
            y2 = int(center_y + radius * 0.8 * math.sin(math.radians(angle)))  # 转换为整数
            painter.drawLine(x1, y1, x2, y2)
            value = self.min_value + (i / self.major_ticks) * (self.max_value - self.min_value)
            painter.drawText(x2, y2, f"{value:.0f}")

        for i in range(self.minor_ticks):
            angle = (i / self.minor_ticks) * 270 - 135
            x1 = int(center_x + radius * 0.75 * math.cos(math.radians(angle)))  # 转换为整数
            y1 = int(center_y + radius * 0.75 * math.sin(math.radians(angle)))  # 转换为整数
            x2 = int(center_x + radius * 0.8 * math.cos(math.radians(angle)))  # 转换为整数
            y2 = int(center_y + radius * 0.8 * math.sin(math.radians(angle)))  # 转换为整数
            painter.drawLine(x1, y1, x2, y2)

        # Draw needle
        painter.setPen(QPen(Qt.red, 2))
        value_normalized = (self.value - self.min_value) / (self.max_value - self.min_value)
        angle = value_normalized * 270 - 135
        x = int(center_x + radius * 0.6 * math.cos(math.radians(angle)))  # 转换为整数
        y = int(center_y + radius * 0.6 * math.sin(math.radians(angle)))  # 转换为整数
        painter.drawLine(center_x, center_y, x, y)

        # Draw gauge name
        painter.setPen(QPen(Qt.black, 2))
        painter.drawText(int(center_x), int(center_y + radius * 0.3), self.gauge_name)

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Parameters
        self.max_velocity = 2.0  # 设置默认值
        self.max_steering_angle = 0.34  # 设置默认值

        # Initialize speed and turn attributes
        self.speed = 0.0
        self.turn = 0.0

        # Publisher
        self.state_pub = self.create_publisher(
            AckermannDriveStamped,
            "/vesc/low_level/ackermann_cmd_mux/input/teleop",
            QoSProfile(depth=1)
        )

        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_cb)

        # GUI setup
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("Keyboard Teleop")
        self.window.setGeometry(100, 100, 600, 400)  # 设置窗口初始大小

        # Layout
        layout = QVBoxLayout()
        controls_layout = QHBoxLayout()
        gauges_layout = QHBoxLayout()  # 创建仪表盘布局

        # Gauges
        self.speed_gauge = GaugeWidget(gauge_name="Speed", min_value=-2.0, max_value=2.0)
        self.turn_gauge = GaugeWidget(gauge_name="Turn", min_value=-0.34, max_value=0.34)
        gauges_layout.addWidget(self.speed_gauge)  # 添加仪表盘到布局
        gauges_layout.addWidget(self.turn_gauge)

        # Buttons
        self.forward_button = QPushButton("Forward")
        self.forward_button.pressed.connect(lambda: self.set_key_state(UP, True))
        self.forward_button.released.connect(lambda: self.set_key_state(UP, False))
        controls_layout.addWidget(self.forward_button)

        self.backward_button = QPushButton("Backward")
        self.backward_button.pressed.connect(lambda: self.set_key_state(DOWN, True))
        self.backward_button.released.connect(lambda: self.set_key_state(DOWN, False))
        controls_layout.addWidget(self.backward_button)

        self.left_button = QPushButton("Left")
        self.left_button.pressed.connect(lambda: self.set_key_state(LEFT, True))
        self.left_button.released.connect(lambda: self.set_key_state(LEFT, False))
        controls_layout.addWidget(self.left_button)

        self.right_button = QPushButton("Right")
        self.right_button.pressed.connect(lambda: self.set_key_state(RIGHT, True))
        self.right_button.released.connect(lambda: self.set_key_state(RIGHT, False))
        controls_layout.addWidget(self.right_button)

        layout.addLayout(gauges_layout)  # 添加仪表盘布局到主布局
        layout.addLayout(controls_layout)

        self.quit_button = QPushButton("Quit")
        self.quit_button.clicked.connect(self.shutdown)
        layout.addWidget(self.quit_button)

        self.window.setLayout(layout)
        self.window.show()

        # Create a timer to update the GUI
        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.update_gui)
        self.gui_timer.start(50)  # Update every 50ms

        # 创建信号对象
        self.keyboard_interrupt_signal = KeyboardInterruptSignal()
        # 连接信号到槽函数
        self.keyboard_interrupt_signal.interrupt_signal.connect(self.handle_keyboard_interrupt)

    def set_key_state(self, key, pressed):
        global state, control
        with state_lock:
            if key == QUIT and pressed:  # 处理退出键
                self.keyboard_interrupt_signal.interrupt_signal.emit()  # 发送键盘中断信号
                return
            index = {"w": 0, "a": 1, "s": 2, "d": 3}[key]
            state[index] = pressed
            control = any(state)

    def publish_cb(self):
        global state, control
        with state_lock:
            if not control:
                return
            ack = AckermannDriveStamped()
            if state[0]:
                ack.drive.speed = self.max_velocity
            elif state[2]:
                ack.drive.speed = -self.max_velocity
            else:
                ack.drive.speed = 0.0

            if state[1]:
                ack.drive.steering_angle = self.max_steering_angle
            elif state[3]:
                ack.drive.steering_angle = -self.max_steering_angle
            else:
                ack.drive.steering_angle = 0.0

            self.state_pub.publish(ack)
            self.speed = ack.drive.speed
            self.turn = ack.drive.steering_angle

    def shutdown(self):
        self.timer.cancel()
        self.gui_timer.stop()
        self.window.close()
        rclpy.shutdown()

    def update_gui(self):
        self.speed_gauge.value = (self.speed / self.max_velocity) * 100
        self.turn_gauge.value = (self.turn / self.max_steering_angle) * 100
        self.speed_gauge.repaint()  # Force immediate repaint
        self.turn_gauge.repaint()

    def handle_keyboard_interrupt(self):
        self.get_logger().info("Keyboard interrupt received, shutting down...")
        self.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    # Instead of rclpy.spin, let the Qt event loop run
    sys.exit(node.app.exec_())

if __name__ == '__main__':
    main()