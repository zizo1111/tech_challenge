import time
import numpy as np
import math
import cv2
import json

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
    cv2.imshow("res", result)
    # kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    # result = cv2.filter2D(result, -1, kernel)
    return result

def crop_bb2(points, image):
    min_x = points[0]
    min_y = points[1]
    max_x = points[2]
    max_y = points[3]

    H_extra = int((max_y - min_y)/2)
    W_extra = int((max_x - min_x)/2)
    result = image[max(int(min_y - H_extra), 0): min(int(max_y + H_extra), image.shape[0]), max(int(min_x - W_extra), 0): min(int(max_x + W_extra), image.shape[1])]
    # kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    # result = cv2.filter2D(result, -1, kernel)
    return result

def resize(image, scale_percent = 200):
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    dim = (width, height)

    # resize image
    return cv2.resize(image, dim, interpolation = cv2.INTER_CUBIC)

def load_json_database(database_path):
    # Opening JSON file
    f = open(database_path)    
    # returns JSON object as 
    # a dictionary
    data = json.load(f)
    # Closing file
    f.close()

    return data

def find_in_database(database, ean):
    for entry in database:
        if int(database[entry]["ean"]) == ean:
            return True, database[entry]
    return False, None

def threshold_image(frame):
    closed = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (21, 1)))

    # #------------------------
    # # Statistics
    # #========================
    dens = np.sum(frame, axis=0)
    mean = np.mean(dens)

    #------------------------
    # Thresholding
    #========================
    thresh = closed.copy()
    for idx, val in enumerate(dens):
        if val< 10800:
            thresh[:,idx] = 0

    (_, thresh2) = cv2.threshold(thresh, 128, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    cv2.imshow("input thresh", thresh2)
    return thresh2

def anotate_frame(objects, value, img):
    text = 'Number of Objects {}'.format(len(objects))
    cv2.putText(img, text, (5, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 0, 255), 3)
    text = 'Current Collected Value {} Ct'.format(value)
    cv2.putText(img, text, (5, 80), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 0, 255), 3)
    
    text = 'Objects'
    cv2.putText(img, text, (5, 140), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 0, 0), 3)
    curr_h = 140
    for key, obj in objects.items():
        curr_h +=60
        text = '{} EAN: {}'.format(obj['data']['name'], obj['data']['ean'])
        cv2.putText(img, text, (5, curr_h), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 0, 0), 3)
        curr_h += 40
        text = 'Material: {} Value/Kg.: {} Eur'.format(obj['data']['packaging']['material'], obj['data']['packaging']['selling_price_per_kg_in_eur'])
        cv2.putText(img, text, (5, curr_h), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 0, 0), 3)
    return img