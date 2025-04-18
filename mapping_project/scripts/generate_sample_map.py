#!/usr/bin/env python3
import numpy as np
import cv2
import os

def create_sample_map():
    # Create a blank image (white background)
    map_size = 400  # pixels
    map_img = np.ones((map_size, map_size), dtype=np.uint8) * 255

    # Draw walls (black)
    cv2.rectangle(map_img, (50, 50), (350, 350), 0, 2)
    
    # Draw some obstacles
    cv2.rectangle(map_img, (100, 100), (150, 150), 0, -1)  # Filled rectangle
    cv2.rectangle(map_img, (250, 100), (300, 150), 0, -1)  # Filled rectangle
    cv2.rectangle(map_img, (100, 250), (150, 300), 0, -1)  # Filled rectangle
    cv2.rectangle(map_img, (250, 250), (300, 300), 0, -1)  # Filled rectangle

    # Draw a central obstacle
    cv2.rectangle(map_img, (175, 175), (225, 225), 0, -1)

    # Save the map
    if not os.path.exists('maps'):
        os.makedirs('maps')
    cv2.imwrite('maps/sample_map.pgm', map_img)
    print("Sample map generated at maps/sample_map.pgm")

if __name__ == '__main__':
    create_sample_map() 