#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from picamera2 import Picamera2
from cv_bridge import CvBridge
import cv2
import numpy as np
import libcamera
from threading import Thread

def slider_for_debug():
    cv2.namedWindow('image',cv2.WINDOW_NORMAL)
    cv2.createTrackbar('HMin', 'image', 0, 179, ImagePublisher.callback)
    cv2.createTrackbar('SMin', 'image', 0, 255, ImagePublisher.callback)    
    cv2.createTrackbar('VMin', 'image', 0, 255, ImagePublisher.callback)    
    cv2.createTrackbar('HMax', 'image', 0, 179, ImagePublisher.callback)
    cv2.createTrackbar('SMax', 'image', 0, 255, ImagePublisher.callback)
    cv2.createTrackbar('VMax', 'image', 0, 255, ImagePublisher.callback)
    cv2.setTrackbarPos('HMax', 'image', 179)
    cv2.setTrackbarPos('SMax', 'image', 255)
    cv2.setTrackbarPos('VMax', 'image', 255)
    cv2.setTrackbarPos('HMin', 'image', 148)
    cv2.setTrackbarPos('SMin', 'image', 108)
    cv2.setTrackbarPos('VMin', 'image', 70)
    hMin = sMin = vMin = hMax = sMax = vMax = 0
    phMin = psMin = pvMin = phMax = psMax = pvMax = 0
    ch = None
    while ch != 27:
        ch = cv2.waitKey(0)
        # if(ch == 96):
            # Send_msg_motor().publish_message('E1')
    pass
    # Send_msg_motor().publish_message('E1')

class KalmanFilter:
    kf = cv2.KalmanFilter(4,2)
    kf.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
    kf.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)

    def predict(self,coordX,coordY):
        # estimates future position of the object
        measured = np.array([[np.float32(coordX)],[np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        x,y = int(predicted[0]), int(predicted[1])
        return x,y


class ImagePublisher(Node):
    x_from_slider = 0
    hMin = sMin = vMin = hMax = sMax = vMax = 0
    phMin = psMin = pvMin = phMax = psMax = pvMax = 0
    def callback(x):
        pass

    def __init__(self):
        super().__init__("image_publisher")
        self.pub = self.create_publisher(Image, "/image", 10)
        timer_period = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = Picamera2()
        video_config = self.cap.create_video_configuration(main={"size": (426, 240), "format": "RGB888"}) # change as needed
        video_config["transform"] = libcamera.Transform(hflip=1,vflip=1)
        video_config["controls"] ["FrameDurationLimits"]= (11111, 11111) #around 90 fps ( 1000000 / 90  = 11111 ms/frame)
        self.cap.configure(video_config)
        self.cap.start()
        x1, y1 = 200, 100
        x2, y2 = 600, 100
        x3, y3 = 600, 350
        x4, y4 = 200, 350
        self.kf = KalmanFilter()


# Create ROI vertices array
        self.roi_vertices = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]], np.int32)

    def timer_callback(self):
        img=self.cap.capture_array()
        
        # adding selection area for blurring the outside of the main ball 
# ???
        hMin = cv2.getTrackbarPos('HMin', 'image')
        sMin = cv2.getTrackbarPos('SMin', 'image')
        vMin = cv2.getTrackbarPos('VMin', 'image')
        hMax = cv2.getTrackbarPos('HMax', 'image')
        sMax = cv2.getTrackbarPos('SMax', 'image')
        vMax = cv2.getTrackbarPos('VMax', 'image')

        # Set minimum and maximum HSV values to display
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])

        # Convert to HSV format and color threshold
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(img, img, mask=mask)

# //////
        cv2.imshow('image', result)
        # self.pub.publish(self.bridge.cv2_to_imgmsg(result, "rgb8"))
        # self.get_logger().info('Publishing...')

def main(args=None):
    thread = Thread(target= slider_for_debug)
    thread.start()
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()