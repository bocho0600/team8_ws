#!/usr/bin/env python3
import rospy
import pigpio
import threading
import time
from std_srvs.srv import Trigger, TriggerResponse

srv_a = None
srv_b = None

# Each drop is a blocking there-and-back rack move, so two overlapping calls
# would fight over the same servo. A service is synchronous, but rospy still
# serves calls in parallel, so the lock is what actually prevents overlap.
drop_lock = None

chan_list = [11, 12]  # Set the channels you want to use (see RPi.GPIO docs!)

# === CONFIG ===
PWM_FORWARD = 1800    # >1500 = forward
PWM_REVERSE = 1200    # <1500 = reverse
PWM_STOP = 1500       # Stop signal.
MM_PER_SEC = 80       # Rack speed (mm/sec) — adjust to your servo + gear

def move_rack(distance_mm):
    """
    Move rack a set distance in mm.
    Positive = forward, Negative = backward.
    """
    direction = 1 if distance_mm > 0 else -1
    duration = abs(distance_mm) / MM_PER_SEC

    if direction > 0:
        pi.set_servo_pulsewidth(SERVO_GPIO, PWM_FORWARD)
    else:
        pi.set_servo_pulsewidth(SERVO_GPIO, PWM_REVERSE)

    time.sleep(duration)

    pi.set_servo_pulsewidth(SERVO_GPIO, PWM_STOP)
    time.sleep(0.5)  # Allow settling

def drop_rect():
    """
    Drops Payload A — requires 44 mm forward movement
    """
    print("Dropping Payload A...")
    move_rack(-44)
    time.sleep(2)
    move_rack(37)

def drop_square():
    """
    Drops Payload B — requires 34 mm backward movement
    """
    print("Dropping Payload B...")
    move_rack(30)
    time.sleep(2)
    move_rack(-33)

def _serve_drop(name, action):
    """Run one drop under the lock and report what happened to the caller.

    The whole point of using a service rather than a topic: the mission node
    blocks until the rack has finished moving and finds out whether it worked,
    instead of publishing into the void and guessing with a sleep.
    """
    if not drop_lock.acquire(blocking=False):
        msg = "A drop is already in progress - ignoring {}".format(name)
        rospy.logwarn(msg)
        return TriggerResponse(success=False, message=msg)

    try:
        rospy.loginfo("Firing %s", name)
        action()
        return TriggerResponse(success=True, message="{} complete".format(name))
    except Exception as e:
        rospy.logerr("%s failed: %s", name, e)
        return TriggerResponse(success=False, message=str(e))
    finally:
        drop_lock.release()


def handle_drop_a(req):
    return _serve_drop("drop_a (Payload A)", drop_rect)


def handle_drop_b(req):
    return _serve_drop("drop_b (Payload B)", drop_square)

def shutdown():
    # Clean up our ROS services if they were set, avoids error messages in logs
    for srv in (srv_a, srv_b):
        if srv is not None:
            srv.shutdown()

    # Stop the pigpio daemon connection
    pi.stop()

# === SETUP ===
if __name__ == '__main__':
    # Setup the ROS backend for this node
    rospy.init_node('actuator_controller', anonymous=True)

    # Setup the pigpio connection (make sure pigpiod is running on your Pi)
    pi = pigpio.pi()

    if not pi.connected:
        rospy.logfatal("Unable to connect to pigpio daemon. Ensure 'sudo pigpiod' is running!")
        exit(1)

    # Define the GPIO pin for the servo
    SERVO_GPIO = 13  # Adjust this to your GPIO pin
    # Setup the GPIO as output (done automatically when setting pulsewidth)

    drop_lock = threading.Lock()

    # Two explicit services rather than one topic carrying a bool. The old
    # /actuator_control/actuator_a topic meant "false = Payload A, true =
    # Payload B", which read as an on/off switch at every call site.
    #
    # Deliberately absolute, not private (~), names: the node is started with
    # anonymous=True, so a private name would resolve to a different service
    # every run and nothing could find it.
    srv_a_name = rospy.get_param('~drop_a_service', '/actuator_control/drop_a')
    srv_b_name = rospy.get_param('~drop_b_service', '/actuator_control/drop_b')

    srv_a = rospy.Service(srv_a_name, Trigger, handle_drop_a)
    srv_b = rospy.Service(srv_b_name, Trigger, handle_drop_b)
    rospy.loginfo("Actuator services ready: %s, %s", srv_a_name, srv_b_name)

    # Register the shutdown hook
    rospy.on_shutdown(shutdown)

    # Spin forever
    rospy.spin()
