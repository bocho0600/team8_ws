#!/usr/bin/env python3
import rospy
import pigpio
import time
from std_msgs.msg import Bool

sub_a = None
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

def callback_a(msg_in):
    # A bool message contains one field called "data" which can be true or false
    # http://docs.ros.org/melodic/api/std_msgs/html/msg/Bool.html
    if msg_in.data:
        rospy.loginfo("Setting output high!")
        drop_square()
    else:
        rospy.loginfo("Setting output low!")
        drop_rect()

def shutdown():
    # Clean up our ROS subscriber if they were set, avoids error messages in logs
    if sub_a is not None:
        sub_a.unregister()

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

    # Setup the subscriber for the actuator control topic
    sub_a = rospy.Subscriber('/actuator_control/actuator_a', Bool, callback_a)

    # Register the shutdown hook
    rospy.on_shutdown(shutdown)

    # Spin forever
    rospy.spin()
