#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from image_processing.msg import TargetDetection, TargetDetectionArray


class ArucoDetector():
    # ArUco dictionary and parameters
    # Confirmed running on OpenCV 5.0.0, which removed the entire legacy
    # ArUco free-function API. This class now uses the ArucoDetector class API
    # throughout (detection, dictionary, params) plus solvePnP/drawFrameAxes
    # for pose estimation, none of which depend on removed functions.
    # OpenCV 5.0 removed the old free-function ArUco API entirely
    # (Dictionary_get, DetectorParameters_create, and the free-function
    # detectMarkers) in favour of this ArucoDetector class.
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    aruco_params = cv2.aruco.DetectorParameters()

    frame_sub_topic = '/depthai_node/image/compressed'
    cam_info_topic = '/depthai_node/camera/camera_info'

    def __init__(self):
        # Real marker side length in metres - per customer needs spec, markers
        # are printed from the 5x5 dictionary at 200mm. Exposed as a ROS param
        # so it can be tuned without touching code if the printed size changes.
        self.marker_length = rospy.get_param('~marker_length', 0.2)

        # Publisher: annotated image (for viewing in rqt/Rviz)
        self.aruco_pub = rospy.Publisher(
            '/processed_aruco/image/compressed', CompressedImage, queue_size=10)

        # Publisher: structured detections (label, id, confidence, 3D position)
        # consumed by NAV for landing-marker selection / waypoint storage.
        self.detection_pub = rospy.Publisher(
            '/target_detections/aruco', TargetDetectionArray, queue_size=10)

        # OpenCV bridge to convert ROS images
        self.br = CvBridge()

        # Built once and reused every frame, rather than rebuilt each call
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Camera calibration - populated from the real CameraInfo topic instead
        # of hardcoded placeholder values. Starts as None; detection is skipped
        # until the first CameraInfo message arrives.
        self.camera_matrix = None
        self.dist_coeffs = None

        # Keep the latest known position for each marker ID seen. Useful if
        # NAV wants a "best known" landing-marker position rather than only
        # the most recent single-frame detection.
        self.marker_positions = {}

        self.cam_info_sub = rospy.Subscriber(
            self.cam_info_topic, CameraInfo, self.cam_info_callback,
            queue_size=1)

        if not rospy.is_shutdown():
            # queue_size=1 + a large buff_size means "always process the
            # newest frame, drop anything that arrives while we're still
            # working on the last one" instead of queuing up and falling
            # further and further behind. Without an explicit queue_size,
            # rospy defaults to an unbounded queue, which is what was
            # causing the growing lag - the callback was working through
            # a backlog of old frames rather than the current one.
            # buff_size is bumped well above ROS's default 64KB TCP buffer
            # so a full compressed frame doesn't get fragmented/stalled at
            # the transport layer before it even reaches the queue.
            self.frame_sub = rospy.Subscriber(
                self.frame_sub_topic, CompressedImage, self.img_callback,
                queue_size=1, buff_size=2**24)

    def cam_info_callback(self, msg_in):
        # K is the 3x3 intrinsic matrix, row-major, from sensor_msgs/CameraInfo
        self.camera_matrix = np.array(msg_in.K, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg_in.D, dtype=np.float64)

    def img_callback(self, msg_in):
        if self.camera_matrix is None:
            # No real calibration received yet - skip rather than use guessed values
            rospy.logwarn_throttle(5, "Waiting for camera_info before running ArUco detection...")
            return

        try:
            frame = self.br.compressed_imgmsg_to_cv2(msg_in)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        aruco_frame, detections = self.find_aruco(frame)

        self.publish_to_ros(aruco_frame)
        self.publish_detections(detections, msg_in.header)

    def estimate_pose(self, marker_corner):
        """
        Replacement for cv2.aruco.estimatePoseSingleMarkers, which was removed
        in OpenCV 4.7+. This is the same underlying math (solvePnP against the
        4 known marker corners in the marker's own coordinate frame) that the
        removed convenience function used internally, so results are identical -
        just not dependent on a function that may or may not exist depending on
        whose machine this runs on.
        """
        half = self.marker_length / 2.0
        # Object points in the marker's own frame: top-left, top-right,
        # bottom-right, bottom-left - matching the corner order detectMarkers
        # returns. Z=0 since the marker is flat.
        obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        img_points = marker_corner.reshape((4, 2)).astype(np.float32)

        success, rvec, tvec = cv2.solvePnP(
            obj_points, img_points, self.camera_matrix, self.dist_coeffs)

        return rvec, tvec

    def find_aruco(self, frame):
        detections = []

        (corners, ids, _) = self.detector.detectMarkers(frame)

        if len(corners) > 0:
            ids = ids.flatten()

            for i, (marker_corner, marker_ID) in enumerate(zip(corners, ids)):
                corners_reshaped = marker_corner.reshape((4, 2)).astype(int)
                top_left, top_right, bottom_right, bottom_left = corners_reshaped

                # Marker length now correctly matches the printed 200mm markers
                rvec, tvec = self.estimate_pose(marker_corner)

                cv2.polylines(frame, [corners_reshaped], isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                cv2.putText(frame, str(marker_ID), (top_left[0], top_left[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                center_x = int(np.mean(corners_reshaped[:, 0]))
                center_y = int(np.mean(corners_reshaped[:, 1]))

                # tvec is the marker's position in the camera's optical frame, in metres
                tvec = tvec.flatten()
                rospy.loginfo(
                    "AruCo Marker Detected, ID: %d, Position (m): x=%.2f y=%.2f z=%.2f",
                    int(marker_ID), tvec[0], tvec[1], tvec[2])

                center_text = f"({center_x}, {center_y})"
                cv2.putText(frame, center_text, (top_left[0], top_left[1] - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                detection = TargetDetection()
                detection.label = "aruco"
                detection.marker_id = int(marker_ID)
                detection.confidence = 1.0
                detection.position = Point(x=float(tvec[0]), y=float(tvec[1]), z=float(tvec[2]))
                detections.append(detection)

                self.marker_positions[int(marker_ID)] = detection.position

        return frame, detections

    def publish_to_ros(self, frame):
        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"
        msg_out.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()

        self.aruco_pub.publish(msg_out)

    def publish_detections(self, detections, header):
        msg_out = TargetDetectionArray()
        msg_out.header = header
        msg_out.detections = detections

        self.detection_pub.publish(msg_out)


def main():
    rospy.init_node('EGB349_vision', anonymous=True)
    rospy.loginfo("Processing images...")

    aruco_detect = ArucoDetector()

    rospy.spin()


if __name__ == "__main__":
    main()
