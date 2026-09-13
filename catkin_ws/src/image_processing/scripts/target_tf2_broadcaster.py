#!/usr/bin/env python3
"""
target_tf2_broadcaster.py

Bridges RGB-only detections into the TF2 tree:
  - YOLO person/backpack from dai_publisher_yolov11_runner.py
      topic: /target_detections/yolo   (marker_id == -1)
  - ArUco markers from aruco_subscriber.py
      topic: /target_detections/aruco  (marker_id >= 0, label == "aruco")

For each detection it broadcasts a "camera -> target_<...>_temp" transform
(note the "_temp" suffix - see naming note below), and republishes a
merged, de-duplicated stream on /target_detections/camera_frame for
target_tf2_listener.py to resolve into the map frame.

A target frame is only added to the persistent "keep-alive" cache once it
has been seen in CONFIRM_COUNT consecutive detection messages (with no gap
longer than MAX_SIGHTING_GAP_SEC between them) - this stops one-off false
positives from being latched into TF forever. Once confirmed, the cached
transform is continuously re-broadcast at REPUBLISH_RATE_HZ so it survives
brief gaps in detection. If a confirmed target then goes STALE_TIMEOUT_SEC
without a fresh real detection, it is dropped from the cache and stops
being republished.

Naming: every frame this node broadcasts carries a "_temp" suffix
(e.g. "target_aruco_3_temp") to make clear it is the live, camera-relative,
constantly-moving version. target_tf2_listener.py separately locks a
CONFIRMED ArUco marker into a permanent, suffix-free "standard annotation"
frame (e.g. "target_aruco_3") directly under the map frame. Because the two
names are always different, this node keeps broadcasting its "_temp" frame
forever regardless of whether the listener has locked that marker in - no
coordination between the two nodes is needed, and neither node's state
depends on the other's, so a restart of either one is harmless: the
listener will simply re-lock the marker the next time it's detected.
YOLO person/backpack detections never get a permanent frame, so their
"_temp" frame is the only one that ever exists for them.

TF chain (see tf2_broadcaster_frames.py):
    map -> emulated_uav   (dynamic - published by the emulator/localisation)
    emulated_uav -> camera (static  - tf2_broadcaster_frames.py)
    camera -> target_*_temp (dynamic - THIS node, always)

Run as:
    rosrun image_processing target_tf2_broadcaster.py
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from image_processing.msg import TargetDetectionArray

# Must match camera_name in tf2_broadcaster_frames.py
CAMERA_FRAME = "camera"

# How often to re-send cached (confirmed) target transforms so they don't
# expire out of a listener's TF buffer between detections.
REPUBLISH_RATE_HZ = 10.0

# Ignore very low-confidence YOLO boxes outright (ArUco is always 1.0).
MIN_CONFIDENCE = 0.5

# Consecutive detection messages required (with no gap > MAX_SIGHTING_GAP_SEC)
# before a target is trusted enough to be added to the persistent cache.
CONFIRM_COUNT = 3
MAX_SIGHTING_GAP_SEC = 1.0

# If a confirmed target hasn't been re-detected for this long, drop it from
# the cache so it stops being republished (prevents stale/old targets
# lingering in TF forever).
STALE_TIMEOUT_SEC = 2.0


def target_frame_name(detection):
    """Build the TF child frame name for a LIVE, camera-relative detection.

    Always carries a "_temp" suffix - see the naming note in the module
    docstring. ArUco detections carry a real marker_id (>= 0); YOLO
    detections use -1. target_tf2_listener.py rebuilds this exact name
    (as temp_frame_name()) to do its lookup - if you change the scheme,
    change it in both files.
    """
    if detection.marker_id != -1:
        return "target_aruco_{}_temp".format(detection.marker_id)
    return "target_{}_temp".format(detection.label)


class TargetTFBroadcaster():

    def __init__(self):
        self.tfbr = tf2_ros.TransformBroadcaster()

        # Persistent cache of CONFIRMED target transforms, kept alive by the
        # republish timer. frame_name -> TransformStamped
        self.last_transforms = {}

        # frame_name -> rospy.Time of the most recent real detection.
        # Used both to reset the confirmation streak on a gap, and to expire
        # stale entries out of last_transforms.
        self.last_seen = {}

        # frame_name -> consecutive-sighting streak, used to gate entry into
        # last_transforms (false-positive filter).
        self.streak = {}

        # Merged, TF-tagged detections for target_tf2_listener.py
        self.pub_merged = rospy.Publisher(
            '/target_detections/camera_frame', TargetDetectionArray, queue_size=10)

        rospy.Subscriber('/target_detections/yolo', TargetDetectionArray,
                         self.callback, queue_size=10)
        rospy.Subscriber('/target_detections/aruco', TargetDetectionArray,
                         self.callback, queue_size=10)

        # Keeps confirmed target frames alive in TF between detections, and
        # drops ones that have gone stale.
        rospy.Timer(rospy.Duration(1.0 / REPUBLISH_RATE_HZ), self.republish_cached)

    def callback(self, msg_in):
        if not msg_in.detections:
            return

        # Ignore low-confidence YOLO boxes outright (ArUco confidence is
        # always 1.0, so this never filters ArUco).
        detections = [d for d in msg_in.detections if d.confidence >= MIN_CONFIDENCE]
        if not detections:
            return

        # If the same frame name shows up twice in one message (e.g. two
        # "person" boxes from a false positive), keep only the highest-
        # confidence one so we don't publish an ambiguous TF frame.
        best_by_key = {}
        for det in detections:
            key = target_frame_name(det)
            if key not in best_by_key or det.confidence > best_by_key[key].confidence:
                best_by_key[key] = det

        now = msg_in.header.stamp

        out_msg = TargetDetectionArray()
        out_msg.header = msg_in.header
        out_msg.header.frame_id = CAMERA_FRAME

        for frame_name, det in best_by_key.items():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = CAMERA_FRAME
            t.child_frame_id = frame_name
            t.transform.translation.x = det.position.x
            t.transform.translation.y = det.position.y
            t.transform.translation.z = det.position.z
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Always broadcast the live detection immediately - this keeps
            # things responsive even for not-yet-confirmed targets.
            self.tfbr.sendTransform(t)

            # Update / reset the consecutive-sighting streak.
            prev_seen = self.last_seen.get(frame_name)
            if prev_seen is not None and (now - prev_seen).to_sec() <= MAX_SIGHTING_GAP_SEC:
                self.streak[frame_name] = self.streak.get(frame_name, 0) + 1
            else:
                self.streak[frame_name] = 1
            self.last_seen[frame_name] = now

            # Only latch into the persistent keep-alive cache once confirmed.
            if self.streak[frame_name] >= CONFIRM_COUNT:
                self.last_transforms[frame_name] = t

            out_msg.detections.append(det)

        self.pub_merged.publish(out_msg)

    def republish_cached(self, event):
        now = rospy.Time.now()

        # Drop anything that hasn't had a real detection recently, so stale
        # targets stop being republished instead of lingering forever.
        stale = [name for name, seen in self.last_seen.items()
                 if (now - seen).to_sec() > STALE_TIMEOUT_SEC]
        for name in stale:
            self.last_transforms.pop(name, None)
            self.last_seen.pop(name, None)
            self.streak.pop(name, None)

        # Re-stamp and re-send every still-active, confirmed target transform.
        for frame_name, cached in self.last_transforms.items():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = cached.header.frame_id
            t.child_frame_id = cached.child_frame_id
            t.transform = cached.transform
            self.tfbr.sendTransform(t)


def main():
    rospy.init_node('target_tf2_broadcaster')
    TargetTFBroadcaster()
    rospy.loginfo("target_tf2_broadcaster running.")
    rospy.spin()


if __name__ == '__main__':
    main()
