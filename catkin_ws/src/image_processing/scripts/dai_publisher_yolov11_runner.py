#!/usr/bin/env python3

'''
Run as:
# check model path line ~30
rosrun image_processing dai_publisher_yolov11_runner.py
'''
############################### Libraries ###############################
from pathlib import Path
import time
import json
import cv2
import numpy as np
import depthai as dai
import rospy
import rospkg
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

from image_processing.msg import TargetDetection, TargetDetectionArray

############################### Parameters ###############################
cam_source = 'rgb'
syncNN = True

# model path - update these to match your converted YOLOv11 blob folder
rospack = rospkg.RosPack()
configPath = Path(rospack.get_path('image_processing') + '/models/best_openvino_2022.1_6shave/best.json')
modelName = "best_openvino_2022.1_6shave"

################################  Yolo Config File
if not configPath.exists():
    raise ValueError("Path {} does not exist!".format(configPath))

with configPath.open() as f:
    config = json.load(f)
nnConfig = config.get("nn_config", {})

metadata = nnConfig.get("NN_specific_metadata", {})
classes = metadata.get("classes", {})
coordinates = metadata.get("coordinates", {})
anchors = metadata.get("anchors", {})
anchorMasks = metadata.get("anchor_masks", {})
iouThreshold = metadata.get("iou_threshold", {})
confidenceThreshold = metadata.get("confidence_threshold", {})

nnMappings = config.get("mappings", {})
labels = nnMappings.get("labels", {})


class DepthaiCamera():
    fps = 10

    pub_topic = '/depthai_node/image/compressed'
    pub_topic_raw = '/depthai_node/image/raw'
    pub_topic_detect = '/depthai_node/detection/compressed'
    pub_topic_cam_inf = '/depthai_node/camera/camera_info'
    pub_topic_targets = '/target_detections/yolo'

    def __init__(self):
        self.pipeline = dai.Pipeline()

        if "input_size" in nnConfig:
            self.nn_shape_w, self.nn_shape_h = tuple(map(int, nnConfig.get("input_size").split('x')))

        self.pub_image = rospy.Publisher(self.pub_topic, CompressedImage, queue_size=10)
        self.pub_image_raw = rospy.Publisher(self.pub_topic_raw, Image, queue_size=10)
        self.pub_image_detect = rospy.Publisher(self.pub_topic_detect, CompressedImage, queue_size=10)
        self.pub_cam_inf = rospy.Publisher(self.pub_topic_cam_inf, CameraInfo, queue_size=10)
        self.pub_targets = rospy.Publisher(self.pub_topic_targets, TargetDetectionArray, queue_size=10)

        # Populated once the device connects and real calibration is read (see run()).
        # The camera_info timer skips publishing until these are set, rather than
        # sending placeholder values.
        self.camera_matrix = None
        self.dist_coeffs = None

        self.timer = rospy.Timer(rospy.Duration(1.0 / 10), self.publish_camera_info, oneshot=False)

        rospy.loginfo("Publishing images to rostopic: {}".format(self.pub_topic))

        self.br = CvBridge()

        rospy.on_shutdown(lambda: self.shutdown())

    def read_calibration(self, device):
        """Read the OAK-D's real factory calibration instead of using hardcoded values."""
        calibData = device.readCalibration()

        intrinsics = calibData.getCameraIntrinsics(
            dai.CameraBoardSocket.CAM_A, self.nn_shape_w, self.nn_shape_h)
        self.camera_matrix = [v for row in intrinsics for v in row]  # flatten 3x3 -> 9

        self.dist_coeffs = calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)

        rospy.loginfo("Loaded real camera calibration from device.")

    def publish_camera_info(self, timer=None):
        if self.camera_matrix is None:
            # Calibration not read yet (device not connected, or was cleared
            # after a device error) - skip rather than publish placeholder
            # or stale intrinsics.
            return

        if rospy.is_shutdown():
            # Avoid racing node teardown - publishing after the node starts
            # shutting down raises "publish() to a closed topic".
            return

        camera_info_msg = CameraInfo()
        camera_info_msg.header.frame_id = "camera_frame"
        camera_info_msg.height = self.nn_shape_h
        camera_info_msg.width = self.nn_shape_w

        camera_info_msg.K = self.camera_matrix
        camera_info_msg.D = list(self.dist_coeffs)
        camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info_msg.P = [
            self.camera_matrix[0], 0.0, self.camera_matrix[2], 0.0,
            0.0, self.camera_matrix[4], self.camera_matrix[5], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        # OAK-D calibration typically returns more than 5 distortion coeffs
        # (rational/polynomial model) rather than the simpler plumb_bob model.
        camera_info_msg.distortion_model = "rational_polynomial" if len(self.dist_coeffs) > 5 else "plumb_bob"
        camera_info_msg.header.stamp = rospy.Time.now()

        self.pub_cam_inf.publish(camera_info_msg)

    def run(self):
        modelPathName = f'{rospack.get_path("image_processing")}/models/{modelName}/{modelName}.blob'
        print(metadata)
        nnPath = str((Path(__file__).parent / Path(modelPathName)).resolve().absolute())
        print(nnPath)

        pipeline = self.createPipeline(nnPath)

        try:
            with dai.Device() as device:
                cams = device.getConnectedCameras()
                depth_enabled = dai.CameraBoardSocket.CAM_B in cams and dai.CameraBoardSocket.CAM_C in cams
                if not depth_enabled:
                    raise RuntimeError(
                        "Stereo pair not available on this device - spatial detection needs LEFT+RIGHT mono cameras. Available cameras: {}".format(cams))

                self.read_calibration(device)

                device.startPipeline(pipeline)

                q_nn_input = device.getOutputQueue(name="nn_input", maxSize=4, blocking=False)
                q_nn = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

                frame = None
                detections = []
                start_time = time.time()
                counter = 0
                fps = 0

                while not rospy.is_shutdown():
                    found_classes = []
                    inRgb = q_nn_input.get()
                    inDet = q_nn.get()

                    if inRgb is not None:
                        frame = inRgb.getCvFrame()
                    else:
                        print("Cam Image empty, trying again...")
                        continue

                    if inDet is not None:
                        detections = inDet.detections
                        for detection in detections:
                            found_classes.append(detection.label)
                        found_classes = np.unique(found_classes)
                        overlay = self.show_yolo(frame, detections)
                    else:
                        print("Detection empty, trying again...")
                        continue

                    if frame is not None:
                        cv2.putText(overlay, "NN fps: {:.2f}".format(fps), (2, overlay.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255, 0, 0))
                        cv2.putText(overlay, "Found classes {}".format(found_classes), (2, 10), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255, 0, 0))
                        self.publish_to_ros(frame)
                        self.publish_detect_to_ros(overlay)
                        self.publish_targets(detections)

                    counter += 1
                    if (time.time() - start_time) > 1:
                        fps = counter / (time.time() - start_time)
                        counter = 0
                        start_time = time.time()

        except RuntimeError as e:
            # Covers X_LINK_ERROR and similar - typically the OAK-D dropping
            # off the bus mid-stream due to USB brownout/undervoltage on the
            # Pi. Stop publishing camera_info with now-stale calibration
            # (read_calibration() won't run again until reconnect succeeds)
            # and let main()'s loop call run() again to attempt reconnect,
            # rather than letting the exception kill the node outright.
            rospy.logerr("DepthAI device error (likely brownout/disconnect): %s", e)
            self.camera_matrix = None
            self.dist_coeffs = None
            rospy.sleep(2.0)  # brief backoff before main() retries run()

    def publish_to_ros(self, frame):
        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"
        msg_out.header.frame_id = "home"
        msg_out.data = np.array(cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])[1]).tobytes()
        self.pub_image.publish(msg_out)

        # NOTE: cv_bridge's cv2_to_imgmsg() hits a KeyError on some
        # ROS Noetic + numpy combinations (numpy dtype hashing changed in
        # numpy 1.24+, breaking cv_bridge's internal type lookup table).
        # Building the Image message manually sidesteps cv_bridge entirely
        # so this works regardless of the numpy version installed.
        msg_img_raw = Image()
        msg_img_raw.header.stamp = msg_out.header.stamp
        msg_img_raw.header.frame_id = "home"
        msg_img_raw.height = frame.shape[0]
        msg_img_raw.width = frame.shape[1]
        msg_img_raw.encoding = "bgr8"
        msg_img_raw.is_bigendian = 0
        msg_img_raw.step = frame.shape[1] * frame.shape[2]
        msg_img_raw.data = frame.tobytes()
        self.pub_image_raw.publish(msg_img_raw)

    def publish_detect_to_ros(self, frame):
        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"
        msg_out.header.frame_id = "home"
        msg_out.data = np.array(cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])[1]).tobytes()
        self.pub_image_detect.publish(msg_out)

    def publish_targets(self, detections):
        msg_out = TargetDetectionArray()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.header.frame_id = "camera_frame"

        for detection in detections:
            target = TargetDetection()
            target.label = labels[detection.label]
            target.marker_id = -1  # not applicable for YOLO detections
            target.confidence = float(detection.confidence)
            # spatialCoordinates are in millimetres - convert to metres to match
            # the ArUco node's units
            target.position = Point(
                x=detection.spatialCoordinates.x / 1000.0,
                y=detection.spatialCoordinates.y / 1000.0,
                z=detection.spatialCoordinates.z / 1000.0)
            msg_out.detections.append(target)

        self.pub_targets.publish(msg_out)

    def frameNorm(self, frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    def show_yolo(self, frame, detections):
        color = (255, 0, 0)
        overlay = frame.copy()
        for detection in detections:
            bbox = self.frameNorm(overlay, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
            cv2.putText(overlay, labels[detection.label], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(overlay, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            # Spatial detections also carry real-world coordinates - show Z (depth) on-screen
            cv2.putText(overlay, f"Z: {detection.spatialCoordinates.z/1000:.2f}m", (bbox[0] + 10, bbox[1] + 60), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.rectangle(overlay, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)

        return overlay

    def createPipeline(self, nnPath):
        pipeline = dai.Pipeline()
        pipeline.setOpenVINOVersion(version=dai.OpenVINO.Version.VERSION_2022_1)

        # Spatial detection network combines YOLO inference with stereo depth,
        # giving x/y/z per detection instead of just a 2D pixel bbox.
        detection_nn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        detection_nn.setConfidenceThreshold(confidenceThreshold)
        detection_nn.setNumClasses(classes)
        detection_nn.setCoordinateSize(coordinates)
        detection_nn.setAnchors(anchors)
        detection_nn.setAnchorMasks(anchorMasks)
        detection_nn.setIouThreshold(iouThreshold)
        detection_nn.setBlobPath(nnPath)
        detection_nn.setNumPoolFrames(2)
        detection_nn.input.setBlocking(False)
        detection_nn.setNumInferenceThreads(2)
        # How much of the detected bbox to sample depth from (0.5 = middle 50%,
        # avoids sampling background pixels at the box edges)
        detection_nn.setBoundingBoxScaleFactor(0.5)
        detection_nn.setDepthLowerThreshold(100)     # 0.1m
        detection_nn.setDepthUpperThreshold(10000)   # 10m - beyond your flight arena size

        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(self.nn_shape_w, self.nn_shape_h)
        cam.setInterleaved(False)
        cam.preview.link(detection_nn.input)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(10)

        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        mono_left.setFps(10)
        mono_right.setFps(10)
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(False)   # extended disparity is compute-heavy and unneeded at your range
        stereo.setSubpixel(False)            # subpixel refinement adds compute for accuracy you don't need here
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        stereo.depth.link(detection_nn.inputDepth)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("nn_input")
        xout_rgb.input.setBlocking(False)
        detection_nn.passthrough.link(xout_rgb.input)

        xinDet = pipeline.create(dai.node.XLinkOut)
        xinDet.setStreamName("nn")
        xinDet.input.setBlocking(False)
        detection_nn.out.link(xinDet.input)

        return pipeline

    def shutdown(self):
        self.timer.shutdown()
        cv2.destroyAllWindows()


def main():
    rospy.init_node('depthai_node')
    dai_cam = DepthaiCamera()

    while not rospy.is_shutdown():
        dai_cam.run()

    dai_cam.shutdown()


if __name__ == "__main__":
    main()
