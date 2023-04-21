#!/usr/bin/env python3

from typing import Optional, Any

import cv2

import rospy

import numpy as np
import os
from sensor_msgs.msg import CompressedImage, Image
from duckietown.dtros import DTROS, DTParam, NodeType, TopicType
from dt_class_utils import DTReminder
from turbojpeg import TurboJPEG
from cv_bridge import CvBridge
from dt_apriltags import Detector


class BotCamera(DTROS):
    def __init__(self, node_name):
        super().__init__(node_name, node_type=NodeType.PERCEPTION)

        print(os.getcwd())

        # parameters
        self.publish_freq = DTParam("~publish_freq", -1)

        # utility objects
        self.bridge = CvBridge()
        self.reminder = DTReminder(frequency=self.publish_freq.value)

        # subscribers
        self.sub_img = rospy.Subscriber(
            "~image_in", CompressedImage, self.cb_image, queue_size=1, buff_size="10MB"
        )

        # publishers
        self.pub_img = rospy.Publisher(
            "~image_out",
            Image,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION,
            dt_healthy_freq=self.publish_freq.value,
            dt_help="Raw image",
        )

    def cb_image(self, msg):
        # make sure this matters to somebody

        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # track the contour of detected markers
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
        tag_detector = Detector(families = "tag36h11",
                        nthreads = 1,
                        quad_decimate = 2.0,
                        quad_sigma = 0.0,
                        refine_edges = 1,
                        decode_sharpening = 0.25,
                        debug = 0)
        
        gray_img = cv2.cvtColor(in_image, cv2.COLOR_RGB2GRAY)
        tags = tag_detector.detect(gray_img)
        
        for tag in tags:
            (topLeft, topRight, bottomRight, bottomLeft) = tag.corners
            
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            
            #draw the bounding box
            line_width = 2
            line_color = (0, 255, 0)
            cv2.line(in_image, topLeft, topRight, line_color, line_width)
            cv2.line(in_image, topRight, bottomRight, line_color, line_width)
            cv2.line(in_image, bottomRight, bottomLeft, line_color, line_width)
            cv2.line(in_image, bottomLeft, topLeft, line_color, line_width)
            
            #draw the center of marker
            cv2.circle(in_image, tuple(map(int, tag.center)), 2, (0, 0, 255), -1)
            
            #draw the marker ID on the image
            cv2.putText(in_image, str(tag.tag_id), org=(topLeft[0], topLeft[1] - 15),
                        fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(255,0,0))
        
        return in_image


if __name__ == "__main__":
    node = BotCamera("test_node")
    rospy.spin()
