#!/usr/bin/env python3

import os
import shutil
import rospkg

# Initialize ROS package path
rospack = rospkg.RosPack()
package_path = rospack.get_path('boat_controls')

# Paths for dataset and compiled dataset
dataset_path = os.path.join(package_path, 'dataset')
compiled_dataset_path = os.path.join(package_path, 'compiled_dataset')

# Function to compile images
def compile_images():
    # Remove existing compiled_dataset folder if it exists
    if os.path.exists(compiled_dataset_path):
        shutil.rmtree(compiled_dataset_path)

    # Create a new compiled_dataset directory
    os.makedirs(compiled_dataset_path)

    # Initialize image counter
    image_counter = 1

    # Walk through dataset directory and gather images
    for root, dirs, files in os.walk(dataset_path):
        for file in files:
            if file.endswith(('.jpg', '.jpeg', '.png')):  # Adjust based on file types you expect
                src_path = os.path.join(root, file)
                dest_filename = f"data_{image_counter}.jpg"
                dest_path = os.path.join(compiled_dataset_path, dest_filename)

                # Copy image to compiled_dataset and rename
                shutil.copy2(src_path, dest_path)
                image_counter += 1

    print(f"Compiled {image_counter - 1} images into {compiled_dataset_path}")

if __name__ == '__main__':
    compile_images()
