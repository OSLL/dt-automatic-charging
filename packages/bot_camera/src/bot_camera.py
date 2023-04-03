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
import apriltag
from duckietown_msgs.msg import Twist2DStamped, BoolStamped, FSMState


class BotCamera(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name, node_type=NodeType.PERCEPTION)
        # publishers
        self.pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_state = rospy.Publisher("fsm_node/mode", FSMState, queue_size=1, latch=True)
        self.bridge = CvBridge()
        
        # subscribers
        self.sub_img = rospy.Subscriber("~image_in", CompressedImage, self.cb_image, queue_size=1, buff_size="10MB")
        self.sub_start_parking = rospy.Subscriber("~parking_start", BoolStamped, self.parking_start, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("fsm_node/mode",FSMState, self.cbMode, queue_size=1)
        
        self.mode = None
        self.bool_start = False
        self.last_command = "None"
        self.back_riding_counter = 0
        self.cant_find_counter = 0

    def parking_start(self, msg):
        if msg.data == True:
            self.bool_start = True
            new_state = FSMState()
            new_state.state = "PARKING"
            self.pub_state.publish(new_state) # set a new state
            
        elif msg.data == False:
            self.bool_start = False
            new_state = FSMState()
            new_state.state = "NORMAL_JOYSTICK_CONTROL"
            self.pub_state.publish(new_state) # set a default state
        else:
            rospy.loginfo("Error in topic \'parking_start\'")

    def cb_image(self, msg):
        # make sure this matters to somebody
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # track the contour of detected markers
        if self.mode == "PARKING":
            # self.turn_left()
            self.marker_detecting(img)

    def marker_detecting(self, in_image):
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        gray_img = cv2.cvtColor(in_image, cv2.COLOR_RGB2GRAY)
        x_center_image = in_image.shape[1] // 2
        tags = detector.detect(gray_img)
        if len(tags) == 0:
            self.cant_find()
            return
        for tag in tags:
            if tag.tag_id == 20:
                self.cant_find_counter = 0
                self.back_riding_counter = 0
                coordinates = tuple(map(int, tag.center))
                x_center_marker = coordinates[0]
                if x_center_image > x_center_marker:
                    self.turn_left()
                elif x_center_image <= x_center_marker:
                    self.turn_right()
        return

    def turn_right(self):
        self.last_command = "Right"
        self.message_print(0.1, -1, "\t\tturning right")

    def turn_left(self):
        self.last_command = "Left"
        self.message_print(0.1, 1, "\t\tturning left")

    def message_print(self, v, omega, message=None):
        msg = Twist2DStamped()
        msg.v = v
        msg.omega = omega
        self.pub.publish(msg)

    def cant_find(self):
        if self.back_riding_counter == 0:
            self.back_riding_counter += 1
            self.message_print(0, 0, "\tstopping bot before back riding")
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
        if self.cant_find_counter < 5:
            if self.last_command == "None":
                self.message_print(0, 1, "\t\tThinks that's can be in start")
            elif self.last_command == "Right":
                self.message_print(-0.5, -1, "\t\tBack riding turning right")
                rospy.sleep(0.3)
                self.cant_find_counter += 1
            elif self.last_command == "Left":
                self.message_print(-0.5, 1, "\t\tBack riding turning left")
                rospy.sleep(0.3)
                self.cant_find_counter += 1
            else:
                self.message_print(0, 0, "\tError last_command value!")
        else:
            rospy.sleep(1)
            self.message_print(0, 2, "\tStop")

    def cbMode(self, fsm_state_msg):
        self.mode = fsm_state_msg.state  # String of the current FSM state


if __name__ == "__main__":
    node_cmd = BotCamera("test_node")
    rospy.spin()
