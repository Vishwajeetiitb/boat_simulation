#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Global variable to hold the camera feed
camera_feed = None

# Function to control the boat's thrusters
def control_boat(left_thrust, right_thrust):
    """
    Sends thrust commands to the boat's thrusters.
    """
    rospy.loginfo(f"Left Thrust: {left_thrust}, Right Thrust: {right_thrust}")
    
    # Publishers for left and right thruster commands
    left_thruster_pub = rospy.Publisher('/wamv/thrusters/left_thrust_cmd', Float32, queue_size=10)
    right_thruster_pub = rospy.Publisher('/wamv/thrusters/right_thrust_cmd', Float32, queue_size=10)

    # Publish the thrust values to both thrusters
    rospy.sleep(0.1)  # Sleep to ensure publishers are set up
    left_thruster_pub.publish(left_thrust)
    right_thruster_pub.publish(right_thrust)

# Camera callback function to update the global camera_feed variable
def camera_callback(msg):
    global camera_feed
    bridge = CvBridge()
    
    # Convert ROS Image message to OpenCV format
    try:
        camera_feed = bridge.imgmsg_to_cv2(msg, "bgr8")
        # Display the image (optional for debugging)
        cv2.imshow("Camera Feed", camera_feed)
        cv2.waitKey(1)  # Allow OpenCV to process GUI events

    except Exception as e:
        rospy.logerr("Failed to convert image: %s", str(e))

# Main function with while loop and image processing
def main():
    rospy.init_node('boat_controller', anonymous=True)

    # Subscribe to the camera topic
    rospy.Subscriber('/wamv/sensors/cameras/front_camera/image_raw', Image, camera_callback)

    # Set thrust values
    left_thrust = 0.1
    right_thrust = 10.0

    rate = rospy.Rate(10)  # 10 Hz loop

    # Loop to control the boat and process images
    while not rospy.is_shutdown():
        # Control the boat
        control_boat(left_thrust, right_thrust)

        # Optional: Process the global camera feed if it’s available
        if camera_feed is not None:
            # Example processing: convert to grayscale
            gray_image = cv2.cvtColor(camera_feed, cv2.COLOR_BGR2GRAY)
            # Additional image processing can go here

        # Sleep to maintain loop rate
        # rate.sleep()

    # Ensure all OpenCV windows are closed when done
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()  # Run the main function
    except rospy.ROSInterruptException:
        pass
