#!/usr/bin/env python3

import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time

# Define your classes and corresponding colors here
classes = ["bottle", "can", "crocodile", "turtle"]
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]  # Colors for each class

# Path to the YOLO configuration and weights
rospack = rospkg.RosPack()
package_path = rospack.get_path('boat_controls')
cfg_path = f"{package_path}/model_files/yolov4_tiny.cfg"
weights_path = f"{package_path}/model_files/yolov4_tiny.weights"

# Toggle for using GPU (CUDA) or CPU
use_cuda = False  # Set to False to use CPU instead of GPU

# YOLO Detector class
class YoloDetector:
    def __init__(self, cfg_path, weights_path, use_cuda=True):
        self.net = cv2.dnn.readNetFromDarknet(cfg_path, weights_path)
        
        if use_cuda:
            # Use CUDA backend for GPU
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            rospy.loginfo("Using GPU (CUDA) for computation.")
        else:
            # Use default backend for CPU
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            rospy.loginfo("Using CPU for computation.")

    def get_output_layers(self):
        layer_names = self.net.getLayerNames()
        output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers().flatten()]
        return output_layers

    def detect(self, frame):
        blob = cv2.dnn.blobFromImage(
            frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        return self.net.forward(self.get_output_layers())

# Function to draw bounding boxes
# Function to draw bounding boxes
def draw_bounding_boxes(frame, outs, conf_threshold=0.65, nms_threshold=0.4):
    width, height = frame.shape[1], frame.shape[0]
    class_ids, confidences, boxes = [], [], []

    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > conf_threshold:
                center_x, center_y = int(detection[0] * width), int(detection[1] * height)
                w, h = int(detection[2] * width), int(detection[3] * height)
                x, y = int(center_x - w / 2), int(center_y - h / 2)
                class_ids.append(class_id)
                confidences.append(float(confidence))
                boxes.append([x, y, w, h])

    # Apply non-maximum suppression to remove duplicate boxes
    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
    
    # Handle different formats of indices
    if len(indices) > 0:
        if isinstance(indices, np.ndarray):  # Newer versions of OpenCV (NumPy array)
            indices = indices.flatten()
        elif isinstance(indices[0], list):  # Older versions of OpenCV (list of lists)
            indices = [i[0] for i in indices]

        # Draw bounding boxes
        for i in indices:
            x, y, w, h = boxes[i]
            label = f"{classes[class_ids[i]]}: {confidences[i]:.2f}"
            color = colors[class_ids[i] % len(colors)]  # Choose color based on class ID
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    

def callback(data, args):
    yolo_detector, bridge = args

    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        outs = yolo_detector.detect(frame)
        draw_bounding_boxes(frame, outs, conf_threshold=0.65, nms_threshold=0.4)

        # Display the frame with bounding boxes in real-time
        cv2.imshow("Object Detection", frame)
        cv2.waitKey(1)

    except CvBridgeError as e:
        rospy.logerr(e)

def main():
    rospy.init_node('simple_yolo_detection', anonymous=True)
    bridge = CvBridge()

    # Load YOLO model with the use_cuda toggle
    yolo_detector = YoloDetector(cfg_path, weights_path, use_cuda=use_cuda)

    # Subscribe to camera feed
    rospy.Subscriber("/wamv/sensors/cameras/front_camera/image_raw", Image, callback, callback_args=(yolo_detector, bridge))
    rospy.spin()

if __name__ == '__main__':
    main()
