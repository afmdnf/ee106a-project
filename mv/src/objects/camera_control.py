#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

center_x, center_y = 336.532, 665.306
pixels_to_dist = 0.0005
# y increases going left, x increases going front

class ImgCapture:
    def __init__(self, obj):
        self.obj = obj
        self.corr_x, self.corr_y = None, None
        self.skImg = rospy.Subscriber("/cameras/right_hand_camera/image", Image, self.callback)

    def callback(self, data):
        #try:
        img = CvBridge().imgmsg_to_cv2(data, "bgr8")
        #cv2.imwrite("hand.png", img)

        masked_img = np.copy(img)
        masked_img = cv2.cvtColor(masked_img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(masked_img, (0, 0, 200), (255, 100, 255))
        mask2 = cv2.inRange(masked_img, (0, 150, 50), (255, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
        mask = mask / 255

        blank = np.zeros(mask.shape)
        masks = []
        contours, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2:]
        for c in contours:
            area = cv2.contourArea(c)
            #if area: print(area)
            if area > 5000 and area < 12000 and self.obj == "spoon":
                mask = cv2.drawContours(np.copy(blank), [c], -1, (255, 255, 255), thickness=-1)
                masks.append(mask)
                # plt.imshow(mask, cmap='gray')
                # plt.show()
            elif area >= 15000 and area < 30000 and self.obj == "cup":
                mask = cv2.drawContours(np.copy(blank), [c], -1, (255, 255, 255), thickness=-1)
                masks.append(mask)
                # plt.imshow(mask, cmap='gray')
                # plt.show()

        centers = []
        for mask in masks:
            centers.append(np.mean(np.argwhere(mask > 0), axis=0))
        if not centers:
            raise Exception("[ERROR]: Could not find object")
        img_center_x, img_center_y = min(centers, key=lambda x:((x[0] - (center_x + 64.0))**2 + (x[1] - center_y)**2))

        # print("x_vals:", min(x_vals), max(x_vals))
        # print("y_vals:", min(y_vals), max(y_vals))
        # print("img_centre:", img_center_x, img_center_y)

        self.corr_x, self.corr_y = (img_center_x - center_x)*pixels_to_dist, (img_center_y - center_y)*pixels_to_dist
        # except CvBridgeError as e:
        #     print(e)
        self.skImg.unregister()


def get_correction(obj):
    ic = ImgCapture(obj)
    while ic.corr_x == None or ic.corr_y == None:
        continue
    cv2.destroyAllWindows()
    return ic.corr_x, ic.corr_y
