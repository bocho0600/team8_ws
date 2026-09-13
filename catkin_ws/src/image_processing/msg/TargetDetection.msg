# A single detected target - used for both ArUco markers and YOLO detections
# (person, backpack, etc.) so NAV only needs to handle one message type.

string label            # "aruco", "person", "backpack", ...
int32 marker_id         # ArUco ID if label == "aruco", otherwise -1
float32 confidence      # 1.0 for ArUco (geometric detection), YOLO conf score otherwise
geometry_msgs/Point position   # x, y, z in metres, camera optical frame
