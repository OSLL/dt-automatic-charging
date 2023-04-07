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

import RPi.GPIO as GPIO #@uncomment


class BotCamera(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name, node_type=NodeType.PERCEPTION)
        self.pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_state = rospy.Publisher("fsm_node/mode", FSMState, queue_size=1, latch=True)
        self.bridge = CvBridge()
        # subscribers
        self.sub_img = rospy.Subscriber("~image_in", CompressedImage, self.cb_image, queue_size=1, buff_size="10MB")
        self.sub_start_parking = rospy.Subscriber("~parking_start", BoolStamped, self.parking_start, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("fsm_node/mode", FSMState, self.cbMode, queue_size=1)
        # publishers

        # logic helper
        self.mode = None
        self.bool_start = False
        self.last_command = "None"
        self.back_riding_counter = 0
        self.cant_find_counter = 0

        self.is_conected = False

    def parking_start(self, msg):
        rospy.loginfo("INIT park")

        if msg.data == True:
            self.bool_start = True
        elif msg.data == False:
            self.bool_start = False
        else:
            rospy.loginfo("Error of Booltype in topik \'parking_start\'")

    def parking_start(self, msg):
        if msg.data == True:
            self.bool_start = True
            new_state = FSMState()
            new_state.state = "PARKING"
            self.pub_state.publish(new_state)  # set a new state

        elif msg.data == False:
            self.bool_start = False
            new_state = FSMState()
            new_state.state = "NORMAL_JOYSTICK_CONTROL"
            self.pub_state.publish(new_state)  # set a default state
        else:
            rospy.loginfo("Error in topic \'parking_start\'")

    # @uncomment
    def get_connection_status(self):
        BUTTON_GPIO = 24
        try:
            GPIO.setup(BUTTON_GPIO, GPIO.IN)#swap from down
            GPIO.setmode(GPIO.BCM)        #swap from up
            gpio_state = GPIO.input(BUTTON_GPIO)
            if gpio_state:
                self.is_conected = True
                self.message_print(0, 0, "Get connection!")
            else:
                self.is_conected = False
        except:
            rospy.loginfo("Can't check connection")

    def cbMode(self, fsm_state_msg):
        self.mode = fsm_state_msg.state  # String of the current FSM state

    def cb_image(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # @uncomment
        self.get_connection_status()
        if self.bool_start and not self.is_conected:
            self.marker_detecting(img)

    def marker_detecting(self, in_image):

        rospy.loginfo("start detection")

        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        gray_img = cv2.cvtColor(in_image, cv2.COLOR_RGB2GRAY)  # при комменте не падаем
        x_center_image = in_image.shape[1] // 2
        tags = detector.detect(gray_img)
        rospy.loginfo("after getting tags")
        if len(tags) == 0:
            self.cant_find()
            return  # тут бот упал
        for tag in tags:
            if tag.tag_id == 20:  # there gotta be special value
                rospy.loginfo("in tags")
                self.cant_find_counter = 0
                self.back_riding_counter = 0
                coordinates = tuple(map(int, tag.center))
                x_center_marker = coordinates[0]
                if x_center_image > x_center_marker:
                    rospy.loginfo("\tgo left")
                    self.turn_left()
                elif x_center_image <= x_center_marker:
                    rospy.loginfo("\tgo right")
                    self.turn_right()
        rospy.loginfo("before return from detecting")  # бот тут падает(
        return

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
        if self.cant_find_counter < 3:
            if self.last_command == "None":
                self.message_print(0, 1, "\t\tStart; try to find marker")
            elif self.last_command == "Right":
                self.message_print(-0.3, -1, "\t\tBack riding turning right")
                rospy.sleep(0.3)
                self.cant_find_counter += 1
            elif self.last_command == "Left":
                self.message_print(-0.3, 1, "\t\tBack riding turning left")
                rospy.sleep(0.3)
                self.cant_find_counter += 1
            else:
                self.message_print(0, 0, "\tError last_command value!")
        else:
            rospy.sleep(1)
            self.message_print(0, 2, "\tStop")


if __name__ == "__main__":
    node_cmd = BotCamera("test_node")
    rospy.spin()
