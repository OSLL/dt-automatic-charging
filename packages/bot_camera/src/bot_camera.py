#!/usr/bin/env python3
 
from typing import Optional, Any
import cv2
import sys
import rospy
import numpy as np
import os
from sensor_msgs.msg import CompressedImage, Image
from duckietown.dtros import DTROS, DTParam, NodeType, TopicType
from dt_class_utils import DTReminder
#from turbojpeg import TurboJPEG
from cv_bridge import CvBridge
from dt_apriltags import Detector
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
 
 
# temp tag id == 20
 
class BotCamera(DTROS):
    def __init__(self, node_name):
 
        super().__init__(node_name, node_type=NodeType.PERCEPTION)
        self.pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        print(os.getcwd())
        # parameters
        self.publish_freq = DTParam("~publish_freq", -1)
        # utility objects
        self.bridge = CvBridge()
        self.reminder = DTReminder(frequency=self.publish_freq.value)
        # subscribers
        self.sub_img = rospy.Subscriber("~image_in", CompressedImage, self.cb_image, queue_size=1, buff_size=10485760)
        self.sub_start_parking = rospy.Subscriber("~parking_start", BoolStamped, self.parking_start, queue_size=1)
        # publishers
        self.pub_img = rospy.Publisher(
            "~image_out",
            Image,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION,
            dt_healthy_freq=self.publish_freq.value,
            dt_help="Raw image",
        )
        # logik helper
        self.bool_start = False
        self.last_command = "None"
        self.back_riding_counter = 0
        self.cant_find_counter = 0
 
 
    def parking_start(self, msg):
        if msg.data == True:
            self.bool_start = True
        elif msg.data == False:
            self.bool_start = False
        else:
            rospy.loginfo("Error of Booltype in topik \'parking_start\'")
 
    def cb_image(self, msg):
        # make sure this matters to somebody
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # track the contour of detected markers
        if self.bool_start:
            img = self.marker_detecting(img)
        # turn 'raw image' into 'raw image message'
        out_msg = self.bridge.cv2_to_imgmsg(img, "rgb8")
        # maintain original header
        out_msg.header = msg.header
        # publish image
        self.pub_img.publish(out_msg)
 
    def marker_detecting(self, in_image):
        # more info about the dt-apriltag package you can see here:
        # https://github.com/duckietown/lib-dt-apriltags
 
        # init AprilTag detector
 
        tag_detector = Detector(families="tag36h11",
                                nthreads=1,
                                quad_decimate=2.0,
                                quad_sigma=0.0,
                                refine_edges=1,
                                decode_sharpening=0.25,
                                debug=0)
        gray_img = cv2.cvtColor(in_image, cv2.COLOR_RGB2GRAY)
        x_center_image = in_image.shape[1]
        x_center_image = x_center_image // 2
        cv2.circle(in_image, tuple(map(int, [gray_img.shape[1] // 2, gray_img.shape[0] // 2])), 4, (255, 255, 0),
                   -1)  # drow centre
        tags = tag_detector.detect(gray_img)
        if len(tags) == 0:
            self.cant_find()
            return in_image #тут бот упал 1 или 2 раза 
        for tag in tags:
            if tag.tag_id == 20:  # there gotta be special value
                self.cant_find_counter = 0
                # self.back_riding_counter = 0
                (topLeft, topRight, bottomRight, bottomLeft) = tag.corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # draw the bounding box
                line_width = 2
                line_color = (0, 255, 0)
                cv2.line(in_image, topLeft, topRight, line_color, line_width)
                cv2.line(in_image, topRight, bottomRight, line_color, line_width)
                cv2.line(in_image, bottomRight, bottomLeft, line_color, line_width)
                cv2.line(in_image, bottomLeft, topLeft, line_color, line_width)
                # draw the center of marker
                cv2.circle(in_image, tuple(map(int, tag.center)), 2, (0, 0, 255), -1)
                # parking
                coordinates = tuple(map(int, tag.center))
                x_center_marker = coordinates[0]
                # x_center_marker = int(topLeft[0]) + (int(topRight[0]) - int(topLeft[0])) // 2
                if x_center_image > x_center_marker:
                    rospy.loginfo("\tgo left")
                    self.turn_left()
 
                elif x_center_image <= x_center_marker:
                    rospy.loginfo("\tgo right")
                    self.turn_right()
 
                # draw the marker ID on the image
                cv2.putText(in_image, str(tag.tag_id), org=(topLeft[0], topLeft[1] - 15),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255, 0, 0))
 
        return in_image
 
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
        sys.stdout.flush()
 
 
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
            self.message_print(0, 2, "\tStop plastic pollution!")
 
if __name__ == "__main__":
    node_cmd = BotCamera("test_node")
    rospy.spin()
 
