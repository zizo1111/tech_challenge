import time
import numpy as np
import math
import cv2

def get_time():
    localtime = time.localtime()
    capturetime = time.strftime("%Y%m%d%H%M%S", localtime)
    return capturetime

def rotatePoint(pt, mat):
    newX = pt[0] * mat[0][0] + pt[1] * mat[0][1] + mat[0][2]
    newY = pt[0] * mat[1][0] + pt[1] * mat[1][1] + mat[1][2]
    return (newX, newY)

# calculate new image size after rotation
def newBouncingBox(size, deg):
    width = size[0]
    height = size[1]

    rad = math.radians(deg)
    sin = abs(math.sin(rad))
    cos = abs(math.cos(rad))

    newWidth = int(width * cos + height * sin) + 1
    newHeight = int(width * sin + height * cos) + 1

    return (newWidth, newHeight)

def crop_bb(points, image):
    # Load input image

    W, H = 200, 50
    # Define corresponding points in output image
    pts1 = np.float32([[0,0],[H,0],[H,W],[0,W]])

    # Get perspective transform and apply it
    M = cv2.getPerspectiveTransform(points,pts1)
    result = cv2.warpPerspective(image,M,(H,W))
    
    cv2.imshow("bb", result)
    return result


def crop_bb1(points, image):
    # Load input image
    min_x = np.amin(points[:,:,0])
    max_x = np.amax(points[:,:,0])
    min_y = np.amin(points[:,:,1])
    max_y = np.amax(points[:,:,1])

    H_extra = int((max_y - min_y)/2)
    W_extra = int((max_x - min_x)/2)
    result = image[max(int(min_y - H_extra), 0): min(int(max_y + H_extra), image.shape[0]), max(int(min_x - W_extra), 0): min(int(max_x + W_extra), image.shape[1])]
    # kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    # result = cv2.filter2D(result, -1, kernel)
    cv2.imshow("bb", result)
    return result

def resize(image, scale_percent = 200):
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)

    # resize image
    return cv2.resize(image, dim, interpolation = cv2.INTER_CUBIC)