#!/usr/bin/env python3

import cv2
import os
import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import BoolStamped, VehicleCorners, WheelsCmdStamped
from geometry_msgs.msg import Point32
from sensor_msgs.msg import CompressedImage

vehicle = os.environ['VEHICLE_NAME']
wheels_cmd = f'/{vehicle}/wheels_driver_node/wheels_cmd'
cmd = f'/{vehicle}/vehicle_cmd_switch_node/cmd'
wheels_cmd_executed = f'/{vehicle}/wheels_driver_node/wheels_cmd_executed'
image = f"/{vehicle}/camera_node/image/compressed"

class VehicleDetectionNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(VehicleDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        # Subscribers
        self.sub_image = rospy.Subscriber(
            image,
            CompressedImage,
            self.cb_image,
            queue_size=1
        )

        self.pub_detection_flag = rospy.Publisher(
            "~detection",
            BoolStamped,
            queue_size=1
        )

        self.name = node_name
        self.bridge = CvBridge()
        self.last_stamp = rospy.Time.now()
    
        rospy.loginfo(f"[{node_name}] Initialization completed.")

    def cb_image(self, image_msg):
        detection_flag_msg_out = BoolStamped()

        image_cv = self.bridge.compressed_imgmsg_to_cv2(image_msg, "bgr8")
        (detection, centers) = cv2.findCirclesGrid(image_cv, patternSize=(7, 3))

        detection_flag_msg_out.header = image_msg.header
        detection_flag_msg_out.data = detection > 0

        self.pub_detection_flag.publish(detection_flag_msg_out)
    
    def on_shutdown(self):
        rospy.loginfo(f'[{self.name}] Shutting down...')
        
if __name__ == "__main__":
    node = VehicleDetectionNode("vehicle_detection_node")
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.on_shutdown(node.on_shutdown)