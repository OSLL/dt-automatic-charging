#!/usr/bin/env python3

import sys
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped


# std_msgs/Header header | не передано
# float32 v              | передано
# float32 omega          | передано


class MyNode(DTROS):

    def __init__(self, node_name):
        super(MyNode, self).__init__(node_name=node_name, node_type=NodeType.DEBUG)
        self.pub = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

    def run(self):
        rate = rospy.Rate(1)  # 1 message per second
        while (True):
            msg = Twist2DStamped()
            msg.v = 0.1
            msg.omega = 1.0
            rospy.loginfo("v;omega as 0.1;1.0")
            self.pub.publish(msg)
            rate.sleep()
            #msg.omega = -1.0
            #rospy.loginfo("v;omega as 0.1;-1.0")
            #self.pub.publish(msg)
            #rate.sleep()
            sys.stdout.flush()


if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='circle_drive')
    # run node
    node.run()
    # keep spinning
    rospy.spin()

