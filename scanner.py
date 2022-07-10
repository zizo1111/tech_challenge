import cv2
import argparse
from pyzbar import pyzbar
# from picamera.array import PiRGBArray
# from picamera import PiCamera
import time

class BarcodeScanner:

    def __init__(self, args):
        self.args = args
        self.cap = None
        self.picam = None
        self.h = self.args['height']
        self.w = self.args['width']
        self.show_raw = self.args['show_raw']
        self.show_res = self.args['show_res']
        self.src = self.args['src']
        if self.src == 'picam':
            self.create_picam_pipeline()
        elif self.src in ['vid', 'usb']:
            self.create_pipeline()

    def create_picam_pipeline(self):
        pass
        # self.picam = PiCamera()
        #self.picam.resolution = (self.w , self.h)
        #self.picam.framerate = 30
        #self.rawCapture = PiRGBArray(self.picam, size=(self.w , self.h))
        # allow the camera to warm up
        time.sleep(0.1)

    def create_pipeline(self):
        if self.src == 'usb':
            self.cap = cv2.VideoCapture(0)
            self.cap.set(3,self.w)
            self.cap.set(4,self.h)
            self.cap.set(5,30)
            time.sleep(2)
        else:
            self.cap = cv2.VideoCapture(self.args['path'])
        

    def process_frame(self, frame):
        pass

    def grab_frame(self):
        if self.picam is not None:
            self.picam.capture(self.rawCapture, format="bgr")
            frame = self.rawCapture.array
        elif self.src in ['vid', 'usb'] and self.cap.isOpened():
            r, frame = self.cap.read()
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

        cv2.destroyAllWindows() 


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
ap.add_argument( "--show_raw", required = False, default = True, help = "show raw frame")
ap.add_argument( "--show_res", required = False, default = False, help = "show processed frame")
args = vars(ap.parse_args())

if __name__ == "__main__":
    print('x', args)
    sc = BarcodeScanner(args)
    while True:
        sc.grab_frame()
        # 'ESC' for quit
        key = cv2.waitKey(20)
        if key == 27:
            break
    
    sc.destroy()
