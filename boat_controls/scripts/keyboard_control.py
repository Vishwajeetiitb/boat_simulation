#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from pynput import keyboard

# Initial thrust values
thrust_forward = 10.0
thrust_turn = 5.0

# ROS publishers for thrusters
left_thruster_pub = None
right_thruster_pub = None

# Global variables to track movement states
move_forward = False
move_backward = False
turn_left = False
turn_right = False

def update_movement():
    """
    Updates the boat's movement based on the current state of key presses.
    """
    left_thrust = 0.0
    right_thrust = 0.0

    # Forward and backward logic
    if move_forward and not move_backward:
        left_thrust += thrust_forward
        right_thrust += thrust_forward
    elif move_backward and not move_forward:
        left_thrust -= thrust_forward
        right_thrust -= thrust_forward

    # Turning logic
    if turn_left and not (move_forward or move_backward):
        # Turn in place to the left
        left_thrust -= thrust_turn
        right_thrust += thrust_turn
    elif turn_right and not (move_forward or move_backward):
        # Turn in place to the right
        left_thrust += thrust_turn
        right_thrust -= thrust_turn
    elif turn_left and move_forward:
        # Forward-left movement
        left_thrust = 0.0
        right_thrust = thrust_forward
    elif turn_right and move_forward:
        # Forward-right movement
        left_thrust = thrust_forward
        right_thrust = 0.0

    # Publish the thrust values
    control_boat(left_thrust, right_thrust)

def control_boat(left_thrust, right_thrust):
    """
    Sends thrust commands to the boat's thrusters.
    """
    rospy.loginfo(f"Left Thrust: {left_thrust}, Right Thrust: {right_thrust}")
    left_thruster_pub.publish(left_thrust)
    right_thruster_pub.publish(right_thrust)

def stop_boat():
    """
    Stops the boat by setting thrust to 0 for both thrusters.
    """
    control_boat(0.0, 0.0)
    rospy.loginfo("Boat stopped")

# Key press event handler
def on_press(key):
    global move_forward, move_backward, turn_left, turn_right
    try:
        if key.char == 'w':  # Forward
            move_forward = True
        elif key.char == 's':  # Backward
            move_backward = True
        elif key.char == 'a':  # Turn Left
            turn_left = True
        elif key.char == 'd':  # Turn Right
            turn_right = True
    except AttributeError:
        pass

# Key release event handler to stop movement in that direction
def on_release(key):
    global move_forward, move_backward, turn_left, turn_right
    try:
        if key.char == 'w':  # Stop Forward
            move_forward = False
        elif key.char == 's':  # Stop Backward
            move_backward = False
        elif key.char == 'a':  # Stop Turning Left
            turn_left = False
        elif key.char == 'd':  # Stop Turning Right
            turn_right = False
    except AttributeError:
        pass
    # Stop the boat if no movement keys are pressed
    if not (move_forward or move_backward or turn_left or turn_right):
        stop_boat()

def main():
    global left_thruster_pub, right_thruster_pub

    rospy.init_node('keyboard_boat_controller', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    # Initialize publishers
    left_thruster_pub = rospy.Publisher('/wamv/thrusters/left_thrust_cmd', Float32, queue_size=10)
    right_thruster_pub = rospy.Publisher('/wamv/thrusters/right_thrust_cmd', Float32, queue_size=10)

    rospy.loginfo("Use W/A/S/D to control the boat and release keys to stop.")

    # Start the keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # Control loop
    while not rospy.is_shutdown():
        update_movement()
        rate.sleep()

    # Stop listener on shutdown
    listener.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
