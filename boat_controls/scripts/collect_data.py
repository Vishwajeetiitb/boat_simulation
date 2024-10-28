#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import rospkg

# Initialize CvBridge
bridge = CvBridge()
data_folder = None

# Get the path for saving images in the `boat_controls` package
rospack = rospkg.RosPack()
package_path = os.path.join(rospack.get_path('boat_controls'), 'dataset')

# Function to create a new folder for each run
def create_folder():
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    folder_path = os.path.join(package_path, timestamp)
    os.makedirs(folder_path, exist_ok=True)
    return folder_path

# Camera callback function to save images
def camera_callback(msg):
    # Convert ROS Image message to OpenCV format
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Save image with a timestamp
        img_filename = f"{datetime.now().strftime('%H-%M-%S-%f')}.jpg"
        img_path = os.path.join(data_folder, img_filename)
        cv2.imwrite(img_path, cv_image)
        rospy.loginfo(f"Image saved: {img_path}")
    
    except Exception as e:
        rospy.logerr("Failed to convert image: %s", str(e))

# Timer callback to capture image every second
def timer_callback(event):
    # Request the latest camera image
    rospy.wait_for_message('/wamv/sensors/cameras/front_camera/image_raw', Image)
    # Trigger the image saving function
    camera_callback(rospy.wait_for_message('/wamv/sensors/cameras/front_camera/image_raw', Image))

# Main function
def main():
    global data_folder

    rospy.init_node('data_collector', anonymous=True)
    rospy.loginfo("Starting data collection at 1 Hz...")

    # Create folder for this run
    data_folder = create_folder()

    # Set up a 1-second timer to capture images
    rospy.Timer(rospy.Duration(1), timer_callback)  # 1 Hz timer

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
