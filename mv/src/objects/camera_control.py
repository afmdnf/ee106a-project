#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImgCapture:
    def __init__(self):
        self.corr_x, self.corr_y = None, None
        self.skImg = rospy.Subscriber("/cameras/right_hand_camera/image", Image, self.callback)

    def callback(self, data):
        print("Processing RGB")
        try:
            img = CvBridge().imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite("hand.png", img)

            masked_img = np.copy(img)
            masked_img = cv2.cvtColor(masked_img, cv2.COLOR_BGR2HSV)

            mask1 = cv2.inRange(masked_img, (0, 0, 200), (255, 100, 255))
            mask2 = cv2.inRange(masked_img, (0, 150, 50), (255, 255, 255))
            mask = cv2.bitwise_or(mask1, mask2)
            mask = mask / 255

            x_vals, y_vals = [], []
            for x in range(len(mask)):
                for y in range(len(mask[0])):
                    if mask[x][y]:
                        x_vals.append(x)
                        y_vals.append(y)
            # print("x_vals:", min(x_vals), max(x_vals))
            # print("y_vals:", min(y_vals), max(y_vals))
            img_center_x, img_center_y = np.mean(x_vals), np.mean(y_vals)
            # print("img_centre:", img_center_x, img_center_y)
            center_x, center_y = 125.23536709645974, 346.5249678762905
            pixels_to_dist = 0.0005
            self.corr_x, self.corr_y = (img_center_x - center_x)*pixels_to_dist, (img_center_y - center_y)*pixels_to_dist
            # plt.imshow(mask, cmap='gray')
            # plt.show()
            #y increasing when going left, x incr going front
        except CvBridgeError as e:
            print(e)
        self.skImg.unregister()

def get_correction():
    ic = ImgCapture()
    cv2.destroyAllWindows()
    while ic.corr_x == None or ic.corr_y == None:
        continue 
    return ic.corr_x, ic.corr_y
