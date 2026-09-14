#!/usr/bin/env python3
import rospy
from image_processing.msg import TargetDetection, TargetDetectionArray
from geometry_msgs.msg import Point

rospy.init_node('fake_detection_publisher')
pub = rospy.Publisher('/target_detections/yolo', TargetDetectionArray, queue_size=10)
rospy.sleep(1.0)  # let the publisher actually register before sending

rate = rospy.Rate(5)
for _ in range(15):  # well past the CONFIRM_COUNT=5 threshold
    msg = TargetDetectionArray()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'camera'

    det = TargetDetection()
    det.label = 'person'
    det.marker_id = -1
    det.confidence = 0.9
    det.position = Point(x=0.0, y=0.0, z=1.0)  # same position each time -> passes the 30cm spread check
    msg.detections.append(det)

    pub.publish(msg)
    rate.sleep()
