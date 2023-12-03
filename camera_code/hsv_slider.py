#!/usr/bin/env python3

import numpy as np
import cv2

def nothing(x):
    pass

if __name__ == "__main__":
    # Setup  camera
    camera = cv2.VideoCapture(0)
    # Create window for HSV sliders
    cv2.namedWindow('HSV Color Palette with RPi Camera Feed')
    cv2.createTrackbar('H_l', 'HSV Color Palette with RPi Camera Feed', 0, 179, nothing)
    cv2.createTrackbar('S_l', 'HSV Color Palette with RPi Camera Feed', 0, 255, nothing)
    cv2.createTrackbar('V_l', 'HSV Color Palette with RPi Camera Feed', 0, 255, nothing)

    cv2.createTrackbar('H_u', 'HSV Color Palette with RPi Camera Feed', 0, 179, nothing)
    cv2.createTrackbar('S_u', 'HSV Color Palette with RPi Camera Feed', 0, 255, nothing)
    cv2.createTrackbar('V_u', 'HSV Color Palette with RPi Camera Feed', 0, 255, nothing)


    try:
        while True:
            _, color_image = camera.read()
            if color_image is not None:
                hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

                # Get slider values
                h_l = cv2.getTrackbarPos('H_l', 'HSV Color Palette with RPi Camera Feed')
                s_l = cv2.getTrackbarPos('S_l', 'HSV Color Palette with RPi Camera Feed')
                v_l = cv2.getTrackbarPos('V_l', 'HSV Color Palette with RPi Camera Feed')

                h_u = cv2.getTrackbarPos('H_u', 'HSV Color Palette with RPi Camera Feed')
                s_u = cv2.getTrackbarPos('S_u', 'HSV Color Palette with RPi Camera Feed')
                v_u = cv2.getTrackbarPos('V_u', 'HSV Color Palette with RPi Camera Feed')


                lower_bound = np.array([h_l, s_l, v_l])  # Adjust as needed
                upper_bound = np.array([h_u,s_u, v_u])  # Adjust as needed
                mask = cv2.inRange(hsv, lower_bound, upper_bound)
                result = cv2.bitwise_and(color_image, color_image, mask=mask)

                cv2.imshow('HSV Color Palette with RPi Camera Feed', result)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    finally:
        # camera.stop()
        cv2.destroyAllWindows()