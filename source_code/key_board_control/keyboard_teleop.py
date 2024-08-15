#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ackermann_msgs.msg import AckermannDriveStamped

import sys, select, termios, tty
import atexit
import os
from threading import Lock
import tkinter as tk
from tkinter import Frame, Label

UP = "w"
LEFT = "a"
DOWN = "s"
RIGHT = "d"
QUIT = "q"

state = [False, False, False, False]
state_lock = Lock()
control = False

class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Parameters
        self.max_velocity = self.get_parameter_or('~speed', 2.0)
        self.max_steering_angle = self.get_parameter_or('~max_steering_angle', 0.34)
        
        # Publisher
        self.state_pub = self.create_publisher(
            AckermannDriveStamped, 
            "/vesc/low_level/ackermann_cmd_mux/input/teleop", 
            QoSProfile(depth=1)
        )
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_cb)
        
        # GUI setup
        self.root = tk.Tk()
        self.frame = Frame(self.root, width=100, height=100)
        self.frame.bind("<KeyPress>", self.keydown)
        self.frame.bind("<KeyRelease>", self.keyup)
        self.frame.pack()
        self.frame.focus_set()
        
        self.label = Label(
            self.frame,
            height=10,
            width=30,
            text="Focus on this window\nand use the WASD keys\nto drive the car.\n\nPress Q to quit",
        )
        self.label.pack()
        print("Press %c to quit" % QUIT)
        
        # Disable X server auto-repeat
        atexit.register(lambda: os.system("xset r on"))
        os.system("xset r off")

    def keyeq(self, e, c):
        return e.char == c or e.keysym == c

    def keyup(self, e):
        global state, control
        with state_lock:
            if self.keyeq(e, UP):
                state[0] = False
            elif self.keyeq(e, LEFT):
                state[1] = False
            elif self.keyeq(e, DOWN):
                state[2] = False
            elif self.keyeq(e, RIGHT):
                state[3] = False
            control = sum(state) > 0

    def keydown(self, e):
        global state, control
        with state_lock:
            if self.keyeq(e, QUIT):
                self.shutdown()
            elif self.keyeq(e, UP):
                state[0] = True
                state[2] = False
            elif self.keyeq(e, LEFT):
                state[1] = True
                state[3] = False
            elif self.keyeq(e, DOWN):
                state[2] = True
                state[0] = False
            elif self.keyeq(e, RIGHT):
                state[3] = True
                state[1] = False
            control = sum(state) > 0

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

            if state[1]:
                ack.drive.steering_angle = self.max_steering_angle
            elif state[3]:
                ack.drive.steering_angle = -self.max_steering_angle

            self.state_pub.publish(ack)

    def shutdown(self):
        self.root.destroy()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.shutdown()

if __name__ == '__main__':
    main()