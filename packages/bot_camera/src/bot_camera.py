#!/usr/bin/env python3

from typing import Optional, Any
import cv2
import sys
import rospy
import numpy as np
import os
from sensor_msgs.msg import CompressedImage, Image
from duckietown.dtros import DTROS, DTParam, NodeType, TopicType
from cv_bridge import CvBridge
import apriltag  # from dt_apriltags import Detector
from duckietown_msgs.msg import Twist2DStamped, BoolStamped, FSMState


##  Try to find best values v-omega for revers and riding
class BotCamera(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name, node_type=NodeType.PERCEPTION)
        self.bridge = CvBridge()

        # subscribers
        self.sub_img = rospy.Subscriber("~image_in", CompressedImage, self.cb_image, queue_size=1, buff_size="10MB")
        self.sub_start_parking = rospy.Subscriber("~parking_start", BoolStamped, self.parking_start, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("fsm_node/mode", FSMState, self.cbMode, queue_size=1)
        self.sub_connection_status = rospy.Subscriber("~connection_status", BoolStamped, self.get_connection_status,
                                                      queue_size=1)

        # publishers
        self.pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_state = rospy.Publisher("fsm_node/mode", FSMState, queue_size=1, latch=True)

        # logic helper
        self.mode = None  # change state for bot
        self.bool_start = False  # false if press button "J" on joystick and true if press button "F" on joystick
        self.last_command = "None"  # save last command to revers riding
        self.back_riding_counter = 0  # for count revers riding publishing
        self.cant_find_counter = 0  # for count how many times we can's tag
        self.is_connected = False  # for checking connection status
        self.delta_to_find = 5  # delta value to ride ahead

    def parking_start(self, msg):  # init joystick messages
        self.bool_start = msg.data
        new_state = FSMState()
        if msg.data:
            new_state.state = "PARKING"
        else:
            new_state.state = "NORMAL_JOYSTICK_CONTROL"
        self.pub_state.publish(new_state)  # set a new state

    def cbMode(self, fsm_state_msg):  # init joystick shielding
        self.mode = fsm_state_msg.state  # String of the current FSM state

    def cb_image(self, msg):  # init image input
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        if self.bool_start and not self.is_connected:
            self.marker_detecting(img)

    def get_connection_status(self, msg):  # init connection status
        self.is_connected = msg.data
        if self.is_connected:
            self.message_print(0, 0, "Connected ura! ura! ura!")
            new_state = FSMState()
            new_state.state = "NORMAL_JOYSTICK_CONTROL"
            self.pub_state.publish(new_state)
        else:
            rospy.loginfo("NOT_connected")

    def marker_detecting(self, in_image):  # marker detecting method
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        gray_img = cv2.cvtColor(in_image, cv2.COLOR_RGB2GRAY)
        x_center_image = in_image.shape[1] // 2
        tags = detector.detect(gray_img)
        if len(tags) == 0:
            self.cant_find()
            return
        for tag in tags:
            if tag.tag_id == 20:  # there gotta be special value
                self.cant_find_counter = 0
                self.back_riding_counter = 0
                coordinates = tuple(map(int, tag.center))
                x_center_marker = coordinates[0]
                rospy.loginfo(f"center image {x_center_image} \t center marker {x_center_marker} \n")
                if x_center_image - self.delta_to_find < x_center_marker < x_center_image + self.delta_to_find:
                    self.go_ahead()
                elif x_center_image > x_center_marker:
                    self.turn_left()
                elif x_center_image <= x_center_marker:
                    self.turn_right()
        return

    def go_ahead(self):
        self.last_command = "Ahead"
        self.message_print(0.2, 0, "\t\tgo ahead")

    def turn_right(self):
        self.last_command = "Right"
        self.message_print(0.1, -1, "\t\tturning right")

    def turn_left(self):
        self.last_command = "Left"
        self.message_print(0.1, 1, "\t\tturning left")

    def message_print(self, v, omega, message):
        msg = Twist2DStamped()
        msg.v = v
        msg.omega = omega
        rospy.loginfo(message)
        self.pub.publish(msg)

    def cant_find(self):
        if self.back_riding_counter == 0:
            self.back_riding_counter += 1
            rospy.sleep(0.1)
            self.message_print(0.3, 0, "\ttry to connect")
            rospy.sleep(0.2)
            self.revers_riding()

        elif self.back_riding_counter == 1:
            self.back_riding_counter += 1
            self.revers_riding()
        else:
            rospy.sleep(1)
            self.back_riding_counter = 0
            self.message_print(0, 0, "\tstop back riding")

    def revers_riding(self):
        if self.cant_find_counter < 3:
            if self.last_command == "None":
                self.message_print(0, 1, "\t\tStart; try to find marker")
            elif self.last_command == "Right":
                self.message_print(-0.5, -0.6, "\t\tBack riding turning right")
                rospy.sleep(0.5)
                self.cant_find_counter += 1
            elif self.last_command == "Left":
                self.message_print(-0.5, 0.6, "\t\tBack riding turning left")
                rospy.sleep(0.5)
                self.cant_find_counter += 1
            else:
                self.message_print(-0.5, 0, "\tBack riding ahead")
                self.cant_find_counter += 1
        else:
            rospy.sleep(1)
            self.message_print(0, 2, "\tStop")


if __name__ == "__main__":
    node_cmd = BotCamera("bot_camera")
    rospy.spin()
    # if rospy.is_shutdown():
    #     new_state = FSMState()
    #     new_state.state = "NORMAL_JOYSTICK_CONTROL"
    #     node_cmd.pub_state.publish(new_state)

