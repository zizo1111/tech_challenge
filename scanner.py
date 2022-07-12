import cv2
import argparse
from pyzbar.pyzbar import decode
from centroidtracker import CentroidTracker
from sortTracker import *
import numpy as np
import dbr
from dbr import *
from utils import *
import time


class BarcodeScanner:
    """Barcode Scanner class that detects, decodes track barcodes. Associates data with database and calculate collected value"""

    def __init__(self, args):
        """
        Initiate BarcodeScanner instance

        Args:
            args (args): input arguments
        """
        self.args = args
        self.cap = None
        self.picam = None
        self.h = self.args["height"]
        self.w = self.args["width"]
        self.h_o = None
        self.w_o = None
        self.show_raw = self.args["show_raw"]
        self.show_res = self.args["show_res"]
        self.src = self.args["src"]
        self.write_vid = self.args["write_vid"]
        self.resize = False
        self.show_res = True
        self.bardet = cv2.barcode_BarcodeDetector()
        self.cntr = 0
        self.tracker = None
        self.objects_dict = {}
        self.database = load_json_database(self.args["database_path"])
        self.total_collected_value = 0.0

        if self.src in ["vid", "usb", "picam"]:
            self.create_pipeline()
            self.tracker_type = self.args["tracker"]

            if self.tracker_type == "sort":
                self.tracker = Sort(
                    max_age=100, min_hits=5, iou_threshold=0.3
                )  # create instance of the SORT tracker
            elif self.tracker_type == "centroid":
                self.tracker = CentroidTracker()
            if self.write_vid:
                self.vid_out = None

        BarcodeReader.init_license(
            "t0072oQAAAGEYPhwVg5Z9zUtzetkQV/6JpaWPwH4urM/bfFHRt2J58F6KlxO5A5HWDBbEB5mBvpg4Y5bfdtsWBEC3lI7Py7jLFSI+"
        )
        self.reader = BarcodeReader()

    def create_pipeline(self):
        """
        Create video pipe line and set it up depending on chosen input source
        """
        if self.src == "picam":
            self.cap = cv2.VideoCapture(0)
            self.cap.set(3, self.w)
            self.cap.set(4, self.h)
            self.cap.set(5, 30)
            time.sleep(2)
        elif self.src == "usb":
            self.cap = cv2.VideoCapture(1)
            self.cap.set(3, self.w)
            self.cap.set(4, self.h)
            self.cap.set(5, 30)
            time.sleep(2)
        else:
            self.cap = cv2.VideoCapture(self.args["path"])

    def grab_frame(self):
        """
        Grab frame from input source to processed
        """
        if self.picam is not None:
            self.picam.capture(self.rawCapture, format="bgr")
            frame = self.rawCapture.array
        elif self.src in ["vid", "usb"] and self.cap.isOpened():
            r, frame = self.cap.read()
            if not r:
                return None
        else:
            frame = cv2.imread(self.args["path"])

        if self.show_raw:
            cv2.imshow("input", frame)

        return frame

    def destroy(self):
        """
        Destroy open windows and release caps and sources
        """
        if self.src in ["vid", "usb"]:
            self.cap.release()
        elif self.src == "picam":
            self.picam.close()
        if self.write_vid and self.vid_out is not None:
            self.vid_out.release()

        cv2.destroyAllWindows()

    def decode_dbr(self, frame):
        """
        Decode barcode snippet usinf DBR

        Args:
            frame (np.array): Input frame

        Returns:
            int: Decoded EAN
        """
        results = self.reader.decode_buffer(frame)
        return results

    def det_decode(self, image):
        """
        Decode and Detect barcode from image snippet using pyzbar if no results, then DBR

        Args:
            frame (np.array): Input frame

        Returns:
            int: Decoded EAN
        """
        barcodes = decode(resize(image))
        if len(barcodes) == 0:
            dbr_res = self.decode_dbr(resize(image))
            if dbr_res is not None:
                return int(dbr_res[0].barcode_text)
            else:
                return None
        else:
            return int(barcodes[0].data)

    def process_method(self, raw_frame):

        """
        Main processing method to detect and decode barcodes in image frame and drawing
        bounding boxes around found barcodes

        Args:
            raw_frame (np.array): input frame
        """
        # resize image if raw image is too small to increase the probability of barcode
        # detection
        self.cntr += 1
        if self.resize and (
            raw_frame.shape[1] * 2 < self.w or raw_frame.shape[0] * 2 < self.h
        ):
            frame = resize(raw_frame, 200)
        else:
            frame = raw_frame

        res_frame = frame.copy()

        # Find barcodes in frame using opencv's detectAndDecode
        retval, decoded_info, decoded_type, points = self.bardet.detectAndDecode(frame)

        rects = []
        if points is not None:
            points = points.astype(np.int)
            for i, point in enumerate(points):
                # draw bounding boxes around detected barcodes
                res_frame = cv2.drawContours(res_frame, [point], 0, (0, 255, 0), 2)

                # Create object and feed it to trackers
                if self.tracker is not None:
                    min_x = np.amin(point[:, 0])
                    max_x = np.amax(point[:, 0])
                    min_y = np.amin(point[:, 1])
                    max_y = np.amax(point[:, 1])
                    box = None
                    if self.tracker_type == "centroid":
                        box = np.array([min_x, max_x, min_y, max_y])
                    elif self.tracker_type == "sort":
                        box = np.array([min_x, min_y, max_x, max_y])
                    rects.append(box.astype("int"))

        # Update tracker using current detection
        if self.tracker is not None and len(rects) > 0:
            self.objects = self.tracker.update(np.array(rects))

            if self.tracker_type == "centroid":
                # loop over the tracked objects
                for (objectID, centroid) in self.objects.items():
                    # draw both the ID of the object and the centroid of the
                    # object on the output frame
                    text = "ID {}".format(objectID)
                    cv2.putText(
                        res_frame,
                        text,
                        (centroid[0] - 10, centroid[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        3,
                    )
                    cv2.circle(
                        res_frame, (centroid[0], centroid[1]), 4, (0, 255, 0), -1
                    )

            elif self.tracker_type == "sort":
                for obj in self.objects:
                    if not obj[4] in self.objects_dict.keys():
                        # decode barcode associated to object if not already decoded
                        res = crop_bb2(obj, frame)
                        ean = self.det_decode(res)
                        if ean is not None:
                            # associate data from database to tracked/detected object
                            self.objects_dict[int(obj[4])] = {"ean": ean, "data": None}

                            # find object in databse using ean
                            ret, data = find_in_database(self.database, ean)
                            if ret:
                                self.objects_dict[int(obj[4])]["data"] = data
                                self.total_collected_value += float(
                                    data["packaging"]["selling_price_per_unit"]
                                )

        else:
            # Barcode decoding every frame in absence of trackers
            if points is not None:
                for i, point in enumerate(points):
                    res = crop_bb1(
                        np.float32([point]), cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    )
                    ean = self.det_decode(res)
                    if ean is not None:
                        ret, data = find_in_database(ean)

        n_frame = None
        # draw results on frame
        if self.show_res:
            # enlarge frame to make room for text
            res_frame = cv2.copyMakeBorder(
                res_frame, 0, 0, 900, 0, cv2.BORDER_CONSTANT, value=[255, 255, 255]
            )
            # anotate frame
            res_frame = anotate_frame(
                self.objects_dict, self.total_collected_value, res_frame
            )
            n_frame = resize(res_frame, 50)
            cv2.imshow("result", n_frame)
            # Initiate video writer
            if self.write_vid and self.vid_out is None:
                self.vid_out = cv2.VideoWriter(
                    "outpy1.avi",
                    cv2.VideoWriter_fourcc("M", "J", "P", "G"),
                    30,
                    (n_frame.shape[1], n_frame.shape[0]),
                )

        # write frame to video file
        if self.write_vid and self.vid_out is not None:
            self.vid_out.write(n_frame)

    def process_method2(self, raw_frame):
        """
        Processing method to detect barcodes in image frame and drawing
        bounding boxes around found contours

        Args:
            raw_frame (np.array): input frame
        """
        gray = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY)

        # compute the Scharr gradient magnitude representation of the images
        # in both the x and y direction
        gradX = cv2.Sobel(gray, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=-1)
        gradY = cv2.Sobel(gray, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=-1)

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
        closed = cv2.erode(closed, None, iterations=4)
        closed = cv2.dilate(closed, None, iterations=4)

        # find the contours in the image, and sort the contours by their area
        (cnts, _) = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # handle the biggest 3 entries
        for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:3]:

            # get the rotated bounding box
            rect = cv2.minAreaRect(c)
            box = np.int0(cv2.boxPoints(rect))

            # draw a bounding box around the detected barcode
            # cv2.drawContours(image, [box], -1, (0, 255, 0), 3)

            # create a rotation matrix to level the box
            rotMatrix = cv2.getRotationMatrix2D(rect[0], rect[2], 1.0)
            rotRect = tuple([rotatePoint(i, rotMatrix) for i in cv2.boxPoints(rect)])
            rotBox = np.int0(rotRect)

            boxWidth, boxHeight = np.int0(rect[1])
            boxOrigX, boxOrigY = np.int0(
                (rect[0][0] - rect[1][0] / 2.0, rect[0][1] - rect[1][1] / 2.0)
            )

            rotWidth, rotHeight = newBouncingBox(
                (raw_frame.shape[1], raw_frame.shape[0]), rect[2]
            )

            # rotate the image
            rotImage = cv2.warpAffine(
                raw_frame, rotMatrix, (rotWidth, rotHeight), flags=cv2.INTER_LINEAR
            )
            cv2.drawContours(rotImage, [rotBox], -1, (0, 0, 255), 2)

            rotImage = rotImage[
                boxOrigY : boxOrigY + boxHeight, boxOrigX : boxOrigX + boxWidth
            ]

            if np.array(rotImage.shape).all() > 0:
                print(decode(rotImage))

    def process_method1(self, raw_frame):
        """
        Processing method to detect barcodes in image frame and drawing
        bounding boxes around found contours

        Args:
            raw_frame (np.array): input frame
        """
        gray_o = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY)

        # equalize lighting
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray_o)

        # edge enhancement
        edge_enh = cv2.Laplacian(gray, ddepth=cv2.CV_8U, ksize=3, scale=1, delta=0)

        # bilateral blur, which keeps edges
        blurred = cv2.bilateralFilter(edge_enh, 13, 50, 50)

        # use simple thresholding. adaptive thresholding might be more robust
        (_, thresh) = cv2.threshold(blurred, 55, 255, cv2.THRESH_BINARY)

        # do some morphology to isolate just the barcode blob
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        closed = cv2.erode(closed, None, iterations=4)
        closed = cv2.dilate(closed, None, iterations=4)
        cv2.imshow("After morphology", closed)

        # find contours left in the image
        (cnts, _) = cv2.findContours(
            closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cs = sorted(cnts, key=cv2.contourArea, reverse=True)
        for c in cs:
            rect = cv2.minAreaRect(c)
            box = np.int0(cv2.boxPoints(rect))
            cv2.drawContours(raw_frame, [box], -1, (0, 255, 0), 3)


def parse_args():
    """
    Parse command line arguments

    Returns:
        args: args
    """
    # parse arguments
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-s",
        "--src",
        default="img",
        const="img",
        nargs="?",
        choices=["usb", "picam", "vid", "img"],
        help="select input source (default: %(default)s)",
    )
    ap.add_argument("-p", "--path", required=False, help="video/image path")
    ap.add_argument(
        "--database_path",
        required=False,
        default="database.json",
        help="video/image path",
    )
    ap.add_argument(
        "-d",
        "--dstDir",
        required=False,
        default="dstDir",
        help="path to the destination",
    )
    ap.add_argument(
        "--width",
        required=False,
        default=1920,
        const=1920,
        nargs="?",
        help="camera frame width",
    )
    ap.add_argument(
        "--height",
        required=False,
        default=1080,
        const=1080,
        nargs="?",
        help="camera frame height",
    )
    ap.add_argument("--show_raw", required=False, default=False, help="show raw frame")
    ap.add_argument(
        "--show_res", required=False, default=True, help="show processed frame"
    )
    ap.add_argument(
        "--write_vid", required=False, default=False, help="write_out video"
    )
    ap.add_argument(
        "--tracker",
        default="sort",
        const="sort",
        nargs="?",
        choices=["sort", "centroid", "none"],
        help="tracker (default: %(default)s)",
    )

    return vars(ap.parse_args())


if __name__ == "__main__":
    # parse args
    args = parse_args()
    #  Create scanner instance
    sc = BarcodeScanner(args)
    if sc.src == "img":

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
