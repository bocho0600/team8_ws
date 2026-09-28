#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from image_processing.msg import TargetDetection, TargetDetectionArray

# OpenCV 4.7 replaced the free-function ArUco API with the ArucoDetector class
# and renamed DetectorParameters_create() -> DetectorParameters(); 5.0 deleted
# the old one outright. This node was written against 5.0, but ROS Noetic ships
# OpenCV 4.2, which only has the old API - and cv_bridge is compiled against
# that 4.2, so upgrading it out from under ROS is not an option. Support both
# rather than pinning to either.
_HAS_ARUCO_DETECTOR = hasattr(cv2.aruco, 'ArucoDetector')


class ArucoDetector():
    # ArUco dictionary and parameters. Everything else this node uses
    # (solvePnP, drawFrameAxes, polylines) is present in both versions.
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    if _HAS_ARUCO_DETECTOR:
        aruco_params = cv2.aruco.DetectorParameters()
    else:
        aruco_params = cv2.aruco.DetectorParameters_create()

    frame_sub_topic = '/depthai_node/image/compressed'
    cam_info_topic = '/depthai_node/camera/camera_info'

    def __init__(self):
        # Real marker side length in metres - per customer needs spec, markers
        # are printed from the 5x5 dictionary at 200mm. Exposed as a ROS param
        # so it can be tuned without touching code if the printed size changes.
        self.marker_length = rospy.get_param('~marker_length', 0.2)

        # The actual TF frame these detections are in. MUST match camera_name
        # in tf2_broadcaster_frames.py (default "camera") and
        # dai_publisher_yolov11_runner.py's own ~camera_frame_id - keep all
        # three in sync. This used to NOT exist: publish_detections() below
        # inherited the incoming image's header verbatim, which carries
        # frame_id "home" (see dai_publisher_yolov11_runner.py's
        # publish_to_ros()) - a frame that is very unlikely to be rigidly
        # attached to the camera, so every ArUco position was being resolved
        # against the wrong parent frame rather than failing loudly.
        self.camera_frame_id = rospy.get_param('~camera_frame_id', 'camera')

        # Publisher: annotated image (for viewing in rqt/Rviz)
        # queue_size=1: live video, so a backlog is never worth keeping -
        # matching the queue_size=1 on the subscriber below for the same reason.
        self.aruco_pub = rospy.Publisher(
            '/processed_aruco/image/compressed', CompressedImage, queue_size=1)

        # The annotated stream is for display only (GCS Rviz, the Pi's bag),
        # so it is published smaller and slower than detection runs: drawing,
        # colour decode and JPEG encode of every full-res frame was a large
        # share of this node's CPU on the Pi. Detection itself still runs on
        # every full-resolution frame.
        self.viz_width = rospy.get_param('~viz_width', 640)
        self.viz_period = 1.0 / rospy.get_param('~viz_rate', 5.0)
        self.last_viz_time = 0.0

        # Publisher: structured detections (label, id, confidence, 3D position)
        # consumed by NAV for landing-marker selection / waypoint storage.
        self.detection_pub = rospy.Publisher(
            '/target_detections/aruco', TargetDetectionArray, queue_size=10)

        # OpenCV bridge to convert ROS images
        self.br = CvBridge()

        # Built once and reused every frame, rather than rebuilt each call.
        # Only the 4.7+ class API has an object to build; on 4.2 there is
        # nothing to hold onto and detect_markers() calls the free function.
        self.detector = (cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
                         if _HAS_ARUCO_DETECTOR else None)

        # Camera calibration - populated from the real CameraInfo topic instead
        # of hardcoded placeholder values. Starts as None; detection is skipped
        # until the first CameraInfo message arrives.
        self.camera_matrix = None
        self.dist_coeffs = None
        # (width, height) the intrinsics above were computed for.
        self.cam_info_size = None

        # Marker corners in the marker's own frame, in the order detectMarkers
        # returns them (TL, TR, BR, BL). Z=0 since the marker is flat. Fixed
        # for the node's lifetime, so built once rather than per detection.
        half = self.marker_length / 2.0
        self.marker_obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        # Keep the latest known position for each marker ID seen. Useful if
        # NAV wants a "best known" landing-marker position rather than only
        # the most recent single-frame detection.
        self.marker_positions = {}

        self.cam_info_sub = rospy.Subscriber(
            self.cam_info_topic, CameraInfo, self.cam_info_callback,
            queue_size=1)

        rospy.loginfo("ArUco detections will be stamped with TF frame: {}".format(
            self.camera_frame_id))

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
        self.cam_info_size = (msg_in.width, msg_in.height)

    def img_callback(self, msg_in):
        if self.camera_matrix is None:
            # No real calibration received yet - skip rather than use guessed values
            rospy.logwarn_throttle(5, "Waiting for camera_info before running ArUco detection...")
            return

        # The annotated stream is only for humans in rqt/Rviz. When nobody is
        # subscribed, or it was published less than viz_period ago, skip the
        # colour decode, the drawing and the JPEG re-encode - together these
        # cost more than detection itself. Detection only needs grayscale
        # (detectMarkers converts to gray internally anyway), and libjpeg
        # decodes grayscale noticeably faster because it can skip chroma
        # upsampling/colour conversion.
        now = rospy.get_time()
        annotate = (self.aruco_pub.get_num_connections() > 0
                    and now - self.last_viz_time >= self.viz_period)
        if annotate:
            self.last_viz_time = now

        if annotate:
            try:
                frame = self.br.compressed_imgmsg_to_cv2(msg_in)
            except CvBridgeError as e:
                rospy.logerr(e)
                return
        else:
            frame = cv2.imdecode(np.frombuffer(msg_in.data, np.uint8),
                                 cv2.IMREAD_GRAYSCALE)
            if frame is None:
                rospy.logerr("Failed to decode compressed image")
                return

        # Intrinsics only hold for the image size they were computed for. A
        # mismatch (e.g. the video stream resized without updating
        # camera_info) gives confidently wrong positions rather than an
        # error, so refuse to publish poses until the two agree.
        frame_size = (frame.shape[1], frame.shape[0])
        if frame_size != self.cam_info_size:
            rospy.logerr_throttle(
                5, "Image size %dx%d does not match camera_info %dx%d - "
                "skipping ArUco detection, poses would be wrong"
                % (frame_size + self.cam_info_size))
            return

        aruco_frame, detections = self.find_aruco(frame, annotate)

        if annotate:
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
        img_points = marker_corner.reshape((4, 2)).astype(np.float32)

        # IPPE_SQUARE is the solver purpose-built for exactly this case (4
        # coplanar points of a square in this corner order): analytic, so
        # faster than the default iterative LM solve, and more accurate.
        success, rvec, tvec = cv2.solvePnP(
            self.marker_obj_points, img_points, self.camera_matrix,
            self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

        return rvec, tvec

    def detect_markers(self, frame):
        """Detect markers through whichever ArUco API this OpenCV provides."""
        if self.detector is not None:
            return self.detector.detectMarkers(frame)
        return cv2.aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.aruco_params)

    def find_aruco(self, frame, annotate=True):
        detections = []

        (corners, ids, _) = self.detect_markers(frame)

        if len(corners) > 0:
            ids = ids.flatten()

            for i, (marker_corner, marker_ID) in enumerate(zip(corners, ids)):
                corners_reshaped = marker_corner.reshape((4, 2)).astype(int)
                top_left, top_right, bottom_right, bottom_left = corners_reshaped

                # Marker length now correctly matches the printed 200mm markers
                rvec, tvec = self.estimate_pose(marker_corner)

                if annotate:
                    # Drawn at full resolution, then the whole frame is shrunk
                    # to viz_width in publish_to_ros(), so line widths, text
                    # sizes and offsets are scaled up by the same factor to
                    # stay readable after the downscale.
                    s = max(1.0, frame.shape[1] / float(self.viz_width))
                    thick = max(1, int(round(2 * s)))
                    cv2.polylines(frame, [corners_reshaped], isClosed=True, color=(0, 255, 0), thickness=thick)
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                    cv2.putText(frame, str(marker_ID), (top_left[0], top_left[1] - int(10 * s)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5 * s, (0, 255, 0), thick)

                    center_x = int(np.mean(corners_reshaped[:, 0]))
                    center_y = int(np.mean(corners_reshaped[:, 1]))
                    center_text = f"({center_x}, {center_y})"
                    cv2.putText(frame, center_text, (top_left[0], top_left[1] - int(30 * s)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5 * s, (0, 0, 255), thick)

                # tvec is the marker's position in the camera's optical frame, in metres
                tvec = tvec.flatten()
                # Throttled: at 30 fps an unthrottled loginfo per marker per
                # frame floods stdout and /rosout. The full stream is on
                # /target_detections/aruco for anything that needs it.
                rospy.loginfo_throttle(
                    1, "AruCo Marker Detected, ID: %d, Position (m): x=%.2f y=%.2f z=%.2f"
                    % (int(marker_ID), tvec[0], tvec[1], tvec[2]))

                detection = TargetDetection()
                detection.label = "aruco"
                detection.marker_id = int(marker_ID)
                detection.confidence = 1.0
                detection.position = Point(x=float(tvec[0]), y=float(tvec[1]), z=float(tvec[2]))
                detections.append(detection)

                self.marker_positions[int(marker_ID)] = detection.position

        return frame, detections

    def publish_to_ros(self, frame):
        # Display-only, so shrink before encoding: at 640 wide the JPEG
        # encode and the message are roughly a quarter of full 720p.
        # INTER_AREA is the right filter for downscaling (no aliasing).
        if frame.shape[1] > self.viz_width:
            h = int(round(frame.shape[0] * self.viz_width / float(frame.shape[1])))
            frame = cv2.resize(frame, (self.viz_width, h), interpolation=cv2.INTER_AREA)

        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"
        # Quality 60 to match the source stream from the depthai node. Left at
        # the default (95) this re-encode came out heavier than the frame it
        # was decoded from, so annotating the image cost more bandwidth over
        # the WiFi link than the original camera feed did.
        msg_out.data = np.array(
            cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])[1]).tobytes()

        self.aruco_pub.publish(msg_out)

    def publish_detections(self, detections, header):
        msg_out = TargetDetectionArray()
        # Was `msg_out.header = header` - this blindly inherited the source
        # image's header, INCLUDING its frame_id ("home" - see
        # dai_publisher_yolov11_runner.py's publish_to_ros()), which is not
        # a frame rigidly attached to the camera. The timestamp is still
        # copied (needed to look up the UAV's pose in TF at the moment the
        # frame was captured), but frame_id is now stamped explicitly with
        # the real camera TF frame instead of inherited.
        msg_out.header.stamp = header.stamp
        msg_out.header.seq = header.seq
        msg_out.header.frame_id = self.camera_frame_id
        msg_out.detections = detections

        self.detection_pub.publish(msg_out)


def main():
    rospy.init_node('EGB349_vision', anonymous=True)
    rospy.loginfo("Processing images...")

    aruco_detect = ArucoDetector()

    rospy.spin()


if __name__ == "__main__":
    main()
