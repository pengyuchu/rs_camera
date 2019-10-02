import numpy as np
import cv2, os
import time
import scipy
from math import sqrt
from skimage import data
import skimage
from skimage.feature import blob_dog, blob_log, blob_doh
from skimage.color import rgb2gray

import json


def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)


def filter_rg(img):
    # img = cv2.cvtColor(img, cv2 .COLOR_BGR2RGB)
    b, g, r = cv2.split(img)
    tmp = r.astype(int) - g.astype(int)

    b[b<=38] = 1
    b[b>38] = 0
    tmp[tmp < 60] = 0
    tmp[tmp != 0] = 200
    tmp = tmp.astype(type(r[0, 0]))
    return tmp


def hough_circles():
    img = cv2.imread('output/IMG_2693.JPG',0)
    edges = cv2.Canny(img,10,200)
    cimg = cv2.cvtColor(edges,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(edges,cv2.HOUGH_GRADIENT,1,30,
                                param1=100,param2=15,minRadius=20,maxRadius=100)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

    cv2.imshow('detected circles',cimg)
    cv2.waitKey(15000)
    cv2.destroyAllWindows()


def blob_detection_with_dog(img):

        # read the binary image
        color_img = img
        image_gray = filter_rg(color_img)


        # denoise
        image_gray = cv2.fastNlMeansDenoising(image_gray, h=150, templateWindowSize=9, searchWindowSize=21)
        image_gray[image_gray <= 70] = 0
        image_gray[image_gray > 70] = 255

        # cv2.imshow('detected blobs', image_gray)
        # cv2.waitKey(1)

        blobs_dog = blob_dog(image_gray,min_sigma=12, max_sigma=30, threshold=.2)
        blobs_dog[:, 2] = blobs_dog[:, 2] * sqrt(2)

        bbox = []

        for blob in blobs_dog:
            y, x, r = blob
            rx = np.arange(0, image_gray.shape[1])
            ry = np.arange(0, image_gray.shape[0])
            circle_mask = (rx[np.newaxis, :] - int(x)) ** 2 + (ry[:, np.newaxis] - int(y)) ** 2 < int(r) ** 2
            # rect_area = image_gray[np.maximum(0, int(y-r)):np.minimum(image_gray.shape[0], int(y+r)), np.maximum(0, int(x-r)):np.minimum(image_gray.shape[1], int(x+r))]
            area = image_gray[circle_mask]
            # print(np.sum(area)/255, area.size)
            if np.sum(area)/255 < area.size / 3:
                continue

            bbox.append(int(y-r))
            bbox.append(int(x-r))
            bbox.append(int(y+r))
            bbox.append(int(x+r))

        bbox = np.asarray(bbox)
        return bbox


    


if __name__ == '__main__':

        start = time.time()
        blob_detection_with_dog(img)
        end = time.time() - start
        print('Run time: ', end)


