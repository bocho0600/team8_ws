#!/usr/bin/env python3

import math

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import PoseStamped, TransformStamped

# Global Variables
tfbr = None
tfsbr = None

uav_name = "uavteam8"
camera_name = "camera"

def send_tf_camera():
	# Create a static transform that is slightly
	# below the UAV and pointing downwards.
	#
	# The mount geometry is exposed as private params so it can be re-measured
	# or the camera re-clocked without editing code, e.g. in a launch file:
	#   <param name="camera_yaw_deg" value="-90"/>
	# Defaults reproduce the original hardcoded mount exactly.
	t = TransformStamped()
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = rospy.get_param('~uav_frame', uav_name)
	t.child_frame_id = rospy.get_param('~camera_frame', camera_name)

	t.transform.translation.x = rospy.get_param('~camera_x', 0.1)
	t.transform.translation.y = rospy.get_param('~camera_y', 0.0)
	t.transform.translation.z = rospy.get_param('~camera_z', -0.15)

	# quaternion_from_euler defaults to 'sxyz': static/extrinsic axes, applied
	# x then y then z about the PARENT (UAV) frame. The pi about y is what
	# points the camera down; the yaw then spins it about the UAV's z.
	#
	# Sign trap: because the camera looks down, its optical z points opposite
	# the UAV's z. A yaw of -90 here is -90 about the UAV's z, which reads as
	# +90 when looking through the lens. Negate it if you're matching what you
	# see in the image rather than the airframe.
	yaw = math.radians(rospy.get_param('~camera_yaw_deg', 0.0))
	q = tf_conversions.transformations.quaternion_from_euler(0, math.pi, yaw)
	t.transform.rotation.x = q[0]
	t.transform.rotation.y = q[1]
	t.transform.rotation.z = q[2]
	t.transform.rotation.w = q[3]

	# Send the static transformation
	tfsbr.sendTransform(t)

"""
This is step typically done by the same program that outputs the pose

def callback_pose( msg_in ):
	# Create a transform at the time
	# from the pose message for where
	# the UAV is in the map
	t = TransformStamped()
	t.header = msg_in.header
	t.child_frame_id = uav_name

	t.transform.translation = msg_in.pose.position
	t.transform.rotation = msg_in.pose.orientation

	# Send the transformation
	tfbr.sendTransform(t)
"""

if __name__ == '__main__':
	rospy.init_node('tf2_broadcaster_frames')

	# Setup pose subscriber
	# This functionality is provided by the emulator
	#rospy.Subscriber('/emulated_uav/pose', PoseStamped, callback_pose)

	# Setup tf2 broadcasters
	#    Broadcaster sends a transformation that
	#    can be found at specific time
	#tfbr = tf2_ros.TransformBroadcaster()
	#    Static broadcaster sends a transformation
	#    that is true for all time
	tfsbr = tf2_ros.StaticTransformBroadcaster()

	send_tf_camera()

	rospy.loginfo("tf2_broadcaster_frames running.")

	try:
		rospy.spin()
	except rospy.exceptions.ROSInterruptException:
		pass
	finally:
		rospy.loginfo("tf2_broadcaster_frames shutting down")