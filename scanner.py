import cv2
import argparse
from pyzbar import pyzbar
from centroidtracker import CentroidTracker
import numpy as np
import dbr
from dbr import *
from utils import *
import time

class BarcodeScanner:

    def __init__(self, args):
        self.args = args
        self.cap = None
        self.picam = None
        self.h = self.args['height']
        self.w = self.args['width']
        self.h_o = None
        self.w_o = None
        self.show_raw = self.args['show_raw']
        self.show_res = self.args['show_res']
        self.src = self.args['src']
        self.write_vid = self.args['write_vid']
        self.resize = False
        self.show_res = True
        self.bardet = cv2.barcode_BarcodeDetector()
        self.cntr = 0
        self.tracker = None

        if self.src in ['vid', 'usb', 'picam']:
            self.create_pipeline()
            self.tracker = CentroidTracker()
            if self.write_vid:
                self.vid_out = None
            

        BarcodeReader.init_license("t0072oQAAAGEYPhwVg5Z9zUtzetkQV/6JpaWPwH4urM/bfFHRt2J58F6KlxO5A5HWDBbEB5mBvpg4Y5bfdtsWBEC3lI7Py7jLFSI+")
        self.reader = BarcodeReader()


    def create_pipeline(self):
        if self.src == 'picam':
            self.cap = cv2.VideoCapture(0)
            self.cap.set(3,self.w)
            self.cap.set(4,self.h)
            self.cap.set(5,30)
            time.sleep(2)
        elif self.src == 'usb':
            self.cap = cv2.VideoCapture(1)
            self.cap.set(3,self.w)
            self.cap.set(4,self.h)
            self.cap.set(5,30)
            time.sleep(2)
        else:
            self.cap = cv2.VideoCapture(self.args['path'])

    def grab_frame(self):   	
        if self.picam is not None:
            self.picam.capture(self.rawCapture, format="bgr")
            frame = self.rawCapture.array
        elif self.src in ['vid', 'usb'] and self.cap.isOpened():
            r, frame = self.cap.read()
            if not r:
                return None
        else:
            frame = cv2.imread(self.args['path'])

        if self.show_raw:
            cv2.imshow("input", frame)

        return frame

    def destroy(self):
        if self.src in ['vid', 'usb']:
            self.cap.release()
        elif self.src == 'picam':
            self.picam.close()
        if self.write_vid and self.vid_out is not None:
            self.vid_out.release()
        cv2.destroyAllWindows() 

    def decode_dbr(self, frame):
        results = self.reader.decode_buffer(frame)
        # if results != None:
                # for result in results:
                #     points = result.localization_result.localization_points
                #     cv2.line(frame, points[0], points[1], (0,255,0), 2)
                #     cv2.line(frame, points[1], points[2], (0,255,0), 2)
                #     cv2.line(frame, points[2], points[3], (0,255,0), 2)
                #     cv2.line(frame, points[3], points[0], (0,255,0), 2)
                #     cv2.putText(frame, result.barcode_text, points[0], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 5)
                #     print(result.barcode_text)
                # cv2,imshow("dbr", frame)
        return results



    def process_method(self, raw_frame):
        #resize image
        self.cntr += 1
        if self.resize and (raw_frame.shape[1]*2 < self.w  or raw_frame.shape[0]*2 < self.h):
          frame = resize(raw_frame, 200)
        else:
            frame = raw_frame
        
        if self.write_vid and self.vid_out is None:
            self.vid_out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (frame.shape[1], frame.shape[0]))
        
        res_frame = frame.copy()
       
        retval, decoded_info, decoded_type, points = self.bardet.detectAndDecode(frame)
        
        rects = []
        if points is not None:
            points = points.astype(np.int)
            for i, point in enumerate(points):
                res_frame = cv2.drawContours(res_frame,[point],0,(0, 255, 0),2)
                if self.tracker is not None:

                    min_x = np.amin(points[:,0])
                    max_x = np.amax(points[:,0])
                    min_y = np.amin(points[:,1])
                    max_y = np.amax(points[:,1])
                    box = np.array([min_x, max_x, min_y, max_y])
                    rects.append(box.astype("int"))

                res = crop_bb1(np.float32([point]), cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

                # find QR and barcodes within the image
                barcodes = pyzbar.decode(resize(res, 500))
                dbr_res = self.decode_dbr(resize(res))
                if dbr_res is not None:
                    for dbr_r in dbr_res:
                        print("dbr", dbr_r.barcode_text)
                

                for barcode in barcodes:
                    print(barcode.data, barcodes)   
                    (x, y, w, h) = barcode.rect
                    #cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 5)
                    barcodeData = barcode.data.decode("utf-8")
                    barcodeType = barcode.type
                    # draw the barcode data and barcode type on the image
                    text = "{} ({})".format(barcodeData, barcodeType)
                    cv2.putText(res_frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 255), 2)

        if self.tracker is not None:
                self.objects = self.tracker.update(rects)
                # loop over the tracked objects
        for (objectID, centroid) in self.objects.items():
            # draw both the ID of the object and the centroid of the
            # object on the output frame
            text = "ID {}".format(objectID)
            cv2.putText(res_frame, text, (centroid[0] - 10, centroid[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
            cv2.circle(res_frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)    
        
        if self.show_res:
            cv2.imshow("result", res_frame)
        
        if self.write_vid and self.vid_out is not None:
            self.vid_out.write(frame)
    
    def process_method2(self, raw_frame):
        gray = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY)

        # compute the Scharr gradient magnitude representation of the images
        # in both the x and y direction
        gradX = cv2.Sobel(gray, ddepth = cv2.CV_32F, dx = 1, dy = 0, ksize = -1)
        gradY = cv2.Sobel(gray, ddepth = cv2.CV_32F, dx = 0, dy = 1, ksize = -1)

        # subtract the y-gradient from the x-gradient
        gradient = cv2.subtract(gradX, gradY)
        gradient = cv2.convertScaleAbs(gradient)

        # blur and threshold the image
        blurred = cv2.blur(gradient, (3, 3))
        (_, thresh) = cv2.threshold(blurred, 225, 255, cv2.THRESH_BINARY)

        # construct a closing kernel and apply it to the thresholded image
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
        closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # perform a series of erosions and dilations
        closed = cv2.erode(closed, None, iterations = 4)
        closed = cv2.dilate(closed, None, iterations = 4)

        # find the contours in the image, and sort the contours by their area
        (cnts, _) = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # handle the biggest 3 entries
        for c in sorted(cnts, key = cv2.contourArea, reverse = True)[:3]:

            # get the rotated bounding box
            rect = cv2.minAreaRect(c)
            box = np.int0(cv2.boxPoints(rect))

            # draw a bounding box around the detected barcode
            #cv2.drawContours(image, [box], -1, (0, 255, 0), 3)

            # create a rotation matrix to level the box
            rotMatrix = cv2.getRotationMatrix2D(rect[0], rect[2], 1.0)
            rotRect = tuple([rotatePoint(i, rotMatrix) for i in cv2.boxPoints(rect)])
            rotBox = np.int0(rotRect)

            boxWidth, boxHeight = np.int0(rect[1])
            boxOrigX, boxOrigY = np.int0((rect[0][0] - rect[1][0] / 2.0, rect[0][1] - rect[1][1] / 2.0))

            rotWidth, rotHeight = newBouncingBox((raw_frame.shape[1], raw_frame.shape[0]), rect[2])

            # rotate the image
            rotImage = cv2.warpAffine(raw_frame, rotMatrix, (rotWidth, rotHeight), flags=cv2.INTER_LINEAR)
            cv2.drawContours(rotImage, [rotBox], -1, (0, 0, 255), 2)

            rotImage = rotImage[boxOrigY:boxOrigY+boxHeight, boxOrigX:boxOrigX+boxWidth]

            if(np.array(rotImage.shape).all() > 0):
                print(pyzbar.decode(rotImage))
    

    def process_method1(self, raw_frame):
        gray_o = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY)

        # equalize lighting
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray_o)

        # edge enhancement
        edge_enh = cv2.Laplacian(gray, ddepth = cv2.CV_8U, 
                                ksize = 3, scale = 1, delta = 0)


        # bilateral blur, which keeps edges
        blurred = cv2.bilateralFilter(edge_enh, 13, 50, 50)

        # use simple thresholding. adaptive thresholding might be more robust
        (_, thresh) = cv2.threshold(blurred, 55, 255, cv2.THRESH_BINARY)


        # do some morphology to isolate just the barcode blob
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        closed = cv2.erode(closed, None, iterations = 4)
        closed = cv2.dilate(closed, None, iterations = 4)
        cv2.imshow("After morphology", closed)


        # find contours left in the image
        (cnts, _) = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cs = sorted(cnts, key = cv2.contourArea, reverse = True)
        for c in cs:
            rect = cv2.minAreaRect(c)
            box = np.int0(cv2.boxPoints(rect))
            cv2.drawContours(raw_frame, [box], -1, (0, 255, 0), 3)

    
    def threshold_image(self, frame):
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



# parse arguments
ap = argparse.ArgumentParser()
ap.add_argument('-s', '--src',
                default='img',
                const='img',
                nargs='?',
                choices=['usb', 'picam', 'vid', 'img'],
                help='select input source (default: %(default)s)')
ap.add_argument("-p", "--path", required = False, help = "video/image path")
ap.add_argument("-d", "--dstDir", required = False, default = "dstDir", help = "path to the destination")
ap.add_argument("--width", required = False, default = 1920, const = 1920, nargs='?',help = "camera frame width")
ap.add_argument("--height", required = False, default = 1080,const = 1080, nargs='?',help = "camera frame height")
ap.add_argument( "--show_raw", required = False, default = False, help = "show raw frame")
ap.add_argument( "--show_res", required = False, default = True, help = "show processed frame")
ap.add_argument( "--write_vid", required = False, default = False, help = "write_out video")

args = vars(ap.parse_args())

if __name__ == "__main__":
    print('x', args)
    sc = BarcodeScanner(args)
    if sc.src == 'img':
        raw = sc.grab_frame()

        if raw is not None:
            sc.process_method(raw)
            cv2.waitKey(0)
    else:     
        while True:
            raw = sc.grab_frame()
            if raw is not None:
                sc.process_method(raw)
            else:
                break
            # 'ESC' for quit
            key = cv2.waitKey(20)
            if key == 27:
                break
        
    sc.destroy()
