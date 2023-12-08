#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from picamera2 import Picamera2
from cv_bridge import CvBridge
import cv2
import numpy as np
import libcamera

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

        # img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # mask = np.zeros_like(img_gray)
        # cv2.fillPoly(mask, [self.roi_vertices], 255)
        # masked_frame = cv2.bitwise_and(img, img, mask=mask)
        # cv2.imshow("Masked Frame", masked_frame)

        img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        lower_blue= np.array([30,40,40]) #0,0,150
        upper_blue= np.array([85,255,255]) #179,50,255 commented ones, for white and shades of white

        mask_blue= cv2.inRange(img_hsv, lower_blue, upper_blue)
        res= cv2.bitwise_and(img,img,mask=mask_blue)
        contours, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = None
        largest_area = 0
        i=0
        for contour in contours:
            if i==0:
                i=1
                continue

            approx = cv2.approxPolyDP(contour,0.01 * cv2.arcLength(contour,True), True)
            area= cv2.contourArea(contour)

            if area > largest_area:
                largest_area = area
                largest_contour = contour

        if largest_contour is not None:

            M= cv2.moments(largest_contour)
            if M['m00'] != 0.0:
                x= int(M['m10']/M['m00'])
                y= int(M['m01']/M['m00'])
            cv2.drawContours(img, [largest_contour], 0, (255, 0, 0), 5)
            cv2.circle(img, (x, y), 5, (0, 0, 255), thickness=cv2.FILLED)
            cv2.putText(img,"obj center",(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,180,67),2)
            
            predicted = self.kf.predict(x, y)
            center_points = []

            for i in range (2):
                # further position prediction with Kalman Filter
                predicted = self.kf.predict(predicted[0],predicted[1])
                center2 = (int(predicted[0]),int(predicted[1]))
                cv2.circle(img, (predicted),8,(255,255,0),4)
                center_points.append(center2)

            for i in range(len(center_points) -1 ):
                cv2.line(img,center_points[i],center_points[i+1],(100,0,100),2 )
                # print(center_points[i],center_points[i+1])
                # print(i)
                x1,y1 = center_points[i]
                x2,y2 = center_points[i+1]
                A = y2-y1
                B = x1-x2
                C = (y1 * (x2 - x1)) - ((y2-y1) * x1)
                if B == 0 : B = 1

            for x_line in range(x,801) :
                y = int((-A * x_line - C ) / B)
                cv2.circle(img,(x_line,y), 5 ,(255,0,255), cv2.FILLED)

                # Y values from 150 & 350 and X == 600
                if y > 150 and y <350 and x_line == 600:
                    print("inside :",y)

        # cv2.imshow("Android_cam", res)
        # cv2.imshow("grey img",img_hsv)
        
        # scale_percent = 50 # scale image to 50% of original size
        # width = int(img.shape[1] * scale_percent / 100)
        # height = int(img.shape[0] * scale_percent / 100)
        # dim = (width, height)
        # img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        cv2.imshow("rezultat",img)
        self.pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        self.get_logger().info('Publishing...')
        
        

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()