#!/usr/bin/env python3
"""
target_tf2_listener.py

Consumes the merged camera-frame detections from target_tf2_broadcaster.py,
looks up each one's pose in "map" via TF2, and:

  1. ArUco markers (marker_id >= 0) are trusted immediately: on the FIRST
     detection, a PERMANENT static "map -> target_aruco_<id>" transform is
     broadcast via StaticTransformBroadcaster - this is the "standard"
     (suffix-free) annotation, and it is what NAV should look up later.
     target_tf2_broadcaster.py's own live frame for the same marker is
     always named "target_aruco_<id>_temp" (note the suffix) and keeps
     being broadcast independently forever - since the two names never
     collide, no coordination between the two nodes is required, and this
     node's permanent frame is self-healing: if this node restarts, the
     next detection of that marker simply re-locks and re-broadcasts it
     from scratch. It is world-fixed and survives forever regardless of
     where the drone flies afterwards. NAV can look it up at the end of
     the breadcrumb pattern to fly back and land on a specific marker
     (e.g. tfBuffer.lookup_transform("map", "target_aruco_3",
     rospy.Time(0))).

  2. Person/backpack (marker_id == -1) still go through the false-positive
     filter - several consecutive, spatially-consistent sightings (30 cm
     consistency) are required before a target is trusted. Once CONFIRMED,
     a ONE-SHOT ROI is published on /target_detection/roi (PoseStamped, map
     frame) for NAV to trigger the descend -> hover -> deploy -> resume
     diversion. No permanent frame is kept for these - the action happens
     once and the mission continues.

  3. Publishes a running world-frame estimate of everything tracked, on
     /target_detections/world (for the GCS 3D display + voice callout).

  4. Publishes the confirmation timestamp on /emulated_uav/target_found for
     compatibility with the tutorial's tf2_listener pattern.

Run as:
    rosrun image_processing target_tf2_listener.py
"""

import rospy
import tf2_ros
from collections import deque
from std_msgs.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped

from image_processing.msg import TargetDetection, TargetDetectionArray

MAP_FRAME = "map"

# --- Confirmation / filtering tuning (person/backpack only - ArUco is
# trusted on first detection, see callback()) ---
BUFFER_LEN = 5          # sightings kept per target
MIN_CONFIDENCE = 0.5    # ignore low-confidence YOLO boxes outright
CONFIRM_COUNT = 5       # sightings required before a target is trusted
MAX_SPREAD_M = 0.3      # max pairwise spread allowed in buffer (30 cm)


def standard_frame_name(detection):
    """Standard, permanent/base frame name - no suffix.

    This is the "final" name a CONFIRMED ArUco marker gets locked in as, in
    the map frame - the one NAV should use to fly back and land later.
    """
    if detection.marker_id != -1:
        return "target_aruco_{}".format(detection.marker_id)
    return "target_{}".format(detection.label)


def temp_frame_name(detection):
    """Live, camera-relative frame name - must exactly match
    target_tf2_broadcaster.py's target_frame_name(). Used only to look up
    a detection's current position before it's confirmed/locked."""
    return "{}_temp".format(standard_frame_name(detection))


def distance(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


class TargetWorldEstimator():

    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfln = tf2_ros.TransformListener(self.tfBuffer)

        # Publishes permanent, world-fixed frames for confirmed ArUco
        # markers - held forever by every listener, independent of camera
        # motion, so NAV can return to a specific marker at any later time.
        self.static_tfbr = tf2_ros.StaticTransformBroadcaster()

        self.buffers = {}       # frame_name -> deque of (x, y, z) in map frame (YOLO only)
        self.confirmed = set()  # frame_names already saved/actioned - never reprocessed

        self.pub_roi = rospy.Publisher(
            '/target_detection/roi', PoseStamped, queue_size=10)
        self.pub_world = rospy.Publisher(
            '/target_detections/world', TargetDetectionArray, queue_size=10)
        self.pub_found_time = rospy.Publisher(
            '/emulated_uav/target_found', Time, queue_size=10)

        rospy.Subscriber('/target_detections/camera_frame', TargetDetectionArray,
                         self.callback, queue_size=10)

    def callback(self, msg_in):
        world_msg = TargetDetectionArray()
        world_msg.header.stamp = msg_in.header.stamp
        world_msg.header.frame_id = MAP_FRAME

        for det in msg_in.detections:
            is_aruco = (det.marker_id != -1)
            std_name = standard_frame_name(det)   # permanent/standard name
            live_name = temp_frame_name(det)       # live "_temp" name to look up

            # --- ArUco: trust on first detection, then never touch again ---
            if is_aruco:
                if std_name in self.confirmed:
                    # Already saved permanently on an earlier detection -
                    # nothing more to do, don't republish or relook it up.
                    continue

                try:
                    t = self.tfBuffer.lookup_transform(
                        MAP_FRAME, live_name, msg_in.header.stamp, rospy.Duration(0.5))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn_throttle(
                        5, "TF lookup failed for {}: {}".format(live_name, e))
                    continue

                pos = (t.transform.translation.x,
                       t.transform.translation.y,
                       t.transform.translation.z)

                self.confirmed.add(std_name)
                self.broadcast_static_target(std_name, pos, msg_in.header.stamp)
                rospy.loginfo(
                    "ArUco marker saved permanently as '%s' (live frame is "
                    "'%s') at map (%.2f, %.2f, %.2f)",
                    std_name, live_name, pos[0], pos[1], pos[2])

                world_det = TargetDetection()
                world_det.label = det.label
                world_det.marker_id = det.marker_id
                world_det.confidence = det.confidence
                world_det.position.x, world_det.position.y, world_det.position.z = pos
                world_msg.detections.append(world_det)

                continue

            # --- Person/backpack: existing consistency-filtered pipeline ---
            if det.confidence < MIN_CONFIDENCE:
                continue

            try:
                t = self.tfBuffer.lookup_transform(
                    MAP_FRAME, live_name, msg_in.header.stamp, rospy.Duration(0.5))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                # Common cause: map -> emulated_uav isn't being published yet
                # by the emulator/localisation, or the buffer hasn't caught up.
                rospy.logwarn_throttle(
                    5, "TF lookup failed for {}: {}".format(live_name, e))
                continue

            pos = (t.transform.translation.x,
                   t.transform.translation.y,
                   t.transform.translation.z)

            buf = self.buffers.setdefault(std_name, deque(maxlen=BUFFER_LEN))
            buf.append(pos)

            # Rolling average smooths per-frame jitter.
            avg = tuple(sum(c) / len(buf) for c in zip(*buf))

            world_det = TargetDetection()
            world_det.label = det.label
            world_det.marker_id = det.marker_id
            world_det.confidence = det.confidence
            world_det.position.x, world_det.position.y, world_det.position.z = avg
            world_msg.detections.append(world_det)

            if std_name not in self.confirmed and self.is_confirmed(buf):
                self.confirmed.add(std_name)

                # One-shot action target: tell NAV to descend, hover,
                # deploy payload, then resume - no permanent frame kept.
                self.send_roi(avg, msg_in.header.stamp)
                self.pub_found_time.publish(msg_in.header.stamp)
                rospy.loginfo(
                    "Target CONFIRMED, ROI sent for descent/drop: %s "
                    "at map (%.2f, %.2f, %.2f)",
                    std_name, avg[0], avg[1], avg[2])

        if world_msg.detections:
            self.pub_world.publish(world_msg)

    @staticmethod
    def is_confirmed(buf):
        if len(buf) < CONFIRM_COUNT:
            return False
        # Reject if any pair of recent sightings disagrees by more than
        # MAX_SPREAD_M - a cheap "consistent enough to be real" check.
        pts = list(buf)
        for i in range(len(pts)):
            for j in range(i + 1, len(pts)):
                if distance(pts[i], pts[j]) > MAX_SPREAD_M:
                    return False
        return True

    def send_roi(self, position, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = MAP_FRAME
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        pose.pose.orientation.w = 1.0
        self.pub_roi.publish(pose)

    def broadcast_static_target(self, frame_name, position, stamp):
        # A static transform is latched and held forever by every listener,
        # regardless of subsequent camera/drone motion - this is what makes
        # the marker's location a permanent, revisitable point in "map".
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = MAP_FRAME
        t.child_frame_id = frame_name
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = position
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tfbr.sendTransform(t)


def main():
    rospy.init_node('target_tf2_listener')
    TargetWorldEstimator()
    rospy.loginfo("target_tf2_listener running.")
    rospy.spin()


if __name__ == '__main__':
    main()
