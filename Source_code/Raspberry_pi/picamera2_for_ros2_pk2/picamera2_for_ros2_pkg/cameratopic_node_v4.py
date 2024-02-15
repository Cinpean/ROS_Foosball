#!/usr/bin/python3
# Latest updates, modified exposure time, aritificially higherd brightness level from HSV spectrum, tewaked color masking & cropped image to see only the greend space
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from picamera2 import Picamera2
from cv_bridge import CvBridge
import cv2
import numpy as np
import libcamera
from std_msgs.msg import String

from threading import Thread
# from time import sleep

def slider_for_debug():
    cv2.namedWindow('Manual_motor',cv2.WINDOW_NORMAL)
    cv2.createTrackbar('thrs1', 'Manual_motor', 100, 200, ImagePublisher.callback)   
    cv2.createTrackbar('thrs2', 'Manual_motor', 100, 200, ImagePublisher.callback_4)   
    # cv2.createTrackbar('HMin', 'image', 0, 179, ImagePublisher.callback)
    # cv2.createTrackbar('SMin', 'image', 0, 255, ImagePublisher.callback)
    # cv2.createTrackbar('VMin', 'image', 0, 255, ImagePublisher.callback)
    # cv2.createTrackbar('HMax', 'image', 0, 179, ImagePublisher.callback)
    # cv2.createTrackbar('SMax', 'image', 0, 255, ImagePublisher.callback)
    # cv2.createTrackbar('VMax', 'image', 0, 255, ImagePublisher.callback)
    ch = None
    while ch != 27:
        ch = cv2.waitKey(0)
        if(ch == 96):
            Send_msg_motor().publish_message('E1')
    pass
    Send_msg_motor().publish_message('E1')



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

class Send_msg_motor(Node):
    def __init__(self):
        super().__init__("position_publisher")
        self.publisher = self.create_publisher(String, 'motor_topic',10 )
    
    def get_keyboard_input(self):
        return input("enter command:")

    def publish_message(self,message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)


class ImagePublisher(Node):
    lovitura = 0
    y_from_slider = 0
    future_object_positions = []  #used only for display
    object_positions = [] #used only for display
    interpolated_steps = 0
    def increase_brightness(self,hsv, value=30):
        # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        lim = 255 - value
        v[v > lim] = 255
        v[v <= lim] += value

        final_hsv = cv2.merge((h, s, v))
        # img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return hsv
    
    def callback(x):
        ImagePublisher.y_from_slider = x
        value = (int(ImagePublisher.interpolate_stepper_steps_16( ImagePublisher.from_px_to_mm_16(ImagePublisher.y_from_slider))))
        print (value)
        result_string = f"A{value}"
        Send_msg_motor().publish_message(result_string)

    def callback_4(x):
        ImagePublisher.y_from_slider = x
        value = (int(ImagePublisher.interpolate_stepper_steps_4( ImagePublisher.from_px_to_mm_16(ImagePublisher.y_from_slider))))
        print (value)
        result_string = f"B{value}"
        Send_msg_motor().publish_message(result_string)

    def __init__(self):
        super().__init__("image_publisher")
        self.pub = self.create_publisher(Image, "/image", 10)
        timer_period = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = Picamera2()
        # self.cap.shutter_speed = 24000
        video_config = self.cap.create_video_configuration(main={"size": (426 , 240), "format": "RGB888"}) # change as needed 426 240
        video_config["transform"] = libcamera.Transform(hflip=0,vflip=1)
        video_config["controls"] ["FrameDurationLimits"]= (100000, 100000) #around 90 fps ( 1000000 / 90  = 11111 ms/frame)
        video_config["controls"]["ExposureTime"] = 40000 # max exposure time is ~37000
        self.cap.configure(video_config)
        self.cap.start()
        x1, y1 = 23, 7
        x2, y2 = 420, 7
        x3, y3 = 420, 238
        x4, y4 = 23, 238
        self.kf = KalmanFilter()
        
        if not ImagePublisher.lovitura:
            ImagePublisher.lovitura = 0

# Create ROI vertices array
        self.roi_vertices = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]], np.int32)

    def interpolate_stepper_steps_16(distance):
        table = [(0, 0), (18, 16), (50, 32), (68, 48), (85, 64), (98, 80), (111, 96), (123, 112), (134, 128), (143, 144), (152, 160), (160, 176), (168, 192)]
        for i in range(len(table) - 1):
            if table[i][0] <= distance <= table[i + 1][0]:
                d1, s1 = table[i]
                d2, s2 = table[i + 1]
                break
            elif distance > table[len(table) - 1][0]:
                d1, s1 = table[len(table) - 2]
                d2, s2 = table[len(table) - 1]
                break
            else :
                d1, s1 = table[0]
                d2, s2 = table[0]                  
            # if distance < table[0][0] :

        # Apply linear interpolation
        if(d2-d1 != 0):
            ImagePublisher.interpolated_steps = s1 + ((distance - d1) * (s2 - s1) / (d2 - d1))
            
        if(d2-d1 == 0):
            return ImagePublisher.interpolated_steps 
        return ImagePublisher.interpolated_steps
    
    def interpolate_stepper_steps_4(distance):
        table = [(0, 0), (18, 40), (50, 80), (68, 120), (85, 160), (98, 200), (111, 240), (123, 280), (134, 320), (143, 360), (152, 400), (160, 440), (168, 480)]
        for i in range(len(table) - 1):
            if table[i][0] <= distance <= table[i + 1][0]:
                d1, s1 = table[i]
                d2, s2 = table[i + 1]
                break
            elif distance > table[len(table) - 1][0]:
                d1, s1 = table[len(table) - 2]
                d2, s2 = table[len(table) - 1]
                break
            else :
                d1, s1 = table[0]
                d2, s2 = table[0]                  
            # if distance < table[0][0] :

        # Apply linear interpolation
        if(d2-d1 != 0):
            ImagePublisher.interpolated_steps = s1 + ((distance - d1) * (s2 - s1) / (d2 - d1))
            
        if(d2-d1 == 0):
            return ImagePublisher.interpolated_steps 
        return ImagePublisher.interpolated_steps

    def from_px_to_mm_16(px_val):
        y = ((px_val-90)*168)/69
        return y
    
    def from_px_to_mm_4(px_val):
        y = ((px_val-90)*480)/69
        return y

    def timer_callback(self):
        img=self.cap.capture_array()
        
        # adding selection area for blurring the outside of the main ball 

        img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        mask = np.zeros_like(img_gray)
        cv2.fillPoly(mask, [self.roi_vertices], 255)
        masked_frame = cv2.bitwise_and(img, img, mask=mask)
        # cv2.imshow("Masked Frame for sected game area", masked_frame)

        img_hsv = cv2.cvtColor(masked_frame,cv2.COLOR_BGR2HSV)
        img_hsv = self.increase_brightness(img_hsv,value=60) #trying to imporve brightness
        
        lower_blue= np.array([164,88,75]) #0,0,150   #155,110,70                                               #old vals 165,118,100
        upper_blue= np.array([179,255,255]) #179,50,255 commented ones, for white and shades of white   #old vals 188,255,255

        # lower_blue2= np.array([164,160,50]) #0,0,150
        # upper_blue2= np.array([177,255,200]) #179,50,255 commented ones, for white and shades of white

        mask_blue= cv2.inRange(img_hsv, lower_blue, upper_blue)
        # mask_blue2= cv2.inRange(img_hsv, lower_blue2, upper_blue2) 
        # mask_blue = cv2.bitwise_or(mask_blue1,mask_blue2)
        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)


        res= cv2.bitwise_and(img_hsv,img,mask=mask_blue)
        contours, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = None
        largest_area = 0
        i=0
        for contour in contours:

            # approx = cv2.approxPolyDP(contour,0.01 * cv2.arcLength(contour,True), True)
            area= cv2.contourArea(contour)
            if area < 38:
                continue
            # Get the bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)

        # Draw a rectangle around the object
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

            center = (int(x + w / 2), int(y + h / 2))
            ImagePublisher.object_positions.append(center) #used only for display

            cv2.circle(img, center, 5, (0, 0, 255), thickness=cv2.FILLED)
            cv2.putText(img, "obj center", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 180, 67), 2)
            predicted = self.kf.predict(center[0], center[1])
            ImagePublisher.future_object_positions.append(predicted) #used only for display
            cv2.circle(img, (int(predicted[0]),int(predicted[1])), 3, (200, 30, 90), 2)
            cv2.line(img,(center[0], center[1]), (int(predicted[0]),int(predicted[1])),(23,10,255), 2)
            cv2.line(img, (center[0], center[1]), (center[0]+400, center[1]), (64, 80, 255), 4)
            
            for x_line in range(x,440) :
                # y = int((-A * x_line - C ) / B)
                # cv2.circle(img,(x_line,y), 1 ,(255,0,255), cv2.FILLED) #/// draw direction line for movement
                y = center[1]
                # Y values from 80 & 150 and X == 
                if y > 75 and y < 165 and x_line == 404 and center > (364,y) and center<(420,y) and ImagePublisher.lovitura == 0:
                    self.get_logger().info('inside : "%d"' % y)
                    Send_msg_motor().publish_message('S60')
                    ImagePublisher.lovitura = 1


                if ImagePublisher.lovitura == 1 and center <(354,y) :
                    ImagePublisher.lovitura = 0
                    self.get_logger().info('inside : "%d"' % center[0])
                    Send_msg_motor().publish_message('S0')
                

                # move around following the ball anywhere in the field, position greater than the half of the field
                if y > 83 and y < 165 and x_line == 404 and center > (200,y) and center < (410,y):
                    self.get_logger().info('px_ball : "%d"' % y)
                    self.get_logger().info('px_to_mm : "%d"' % ImagePublisher.from_px_to_mm_16(y))
                    value = (int(ImagePublisher.interpolate_stepper_steps_4( ImagePublisher.from_px_to_mm_16(y))))
                    self.get_logger().info('Linear : "%d"' % value) 
                    result_string = f"B{value}"
                    Send_msg_motor().publish_message(result_string)


        for i in range(1, len(ImagePublisher.object_positions)):
            cv2.line(img, ImagePublisher.object_positions[i - 1], ImagePublisher.object_positions[i], (0, 255, 0), 2)

        

        if largest_contour is not None:

            M= cv2.moments(largest_contour)
            if M['m00'] != 0.0:
                x= int(M['m10']/M['m00'])
                y= int(M['m01']/M['m00'])
            center = (x,y)
            # cv2.drawContours(img, [largest_contour], 0, (255, 0, 0), 2)
            cv2.circle(img, (x, y), 5, (0, 0, 255), thickness=1)
            cv2.putText(img,"Ball",(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,180,67),2)
            
            predicted = self.kf.predict(x, y)
            center_points = []

            for i in range (4):
                # further position prediction with Kalman Filter
                predicted = self.kf.predict(predicted[0],predicted[1])
                center2 = (int(predicted[0]),int(predicted[1]))
                cv2.circle(img, (predicted),1,(255,255,0),2) 
                center_points.append(center2)

            for i in range(len(center_points) -1 ):
                # cv2.line(img,center_points[i],center_points[i+1],(100,0,100),2 )  #///
                # print(center_points[i],center_points[i+1])
                # print(i)
                x1,y1 = center_points[i]
                x2,y2 = center_points[i+1]
                A = y2-y1
                B = x1-x2
                C = (y1 * (x2 - x1)) - ((y2-y1) * x1)
                if B == 0 : B = 1

            # for x_line in range(x,440) :
            #     # y = int((-A * x_line - C ) / B)
            #     # cv2.circle(img,(x_line,y), 1 ,(255,0,255), cv2.FILLED) #/// draw direction line for movement

            #     # Y values from 80 & 150 and X == 
            #     if y > 75 and y < 165 and x_line == 404 and center > (364,y) and center<(420,y) and ImagePublisher.lovitura == 0:
            #         self.get_logger().info('inside : "%d"' % y)
            #         Send_msg_motor().publish_message('S60')
            #         ImagePublisher.lovitura = 1


            #     if ImagePublisher.lovitura == 1 and center <(354,y) :
            #         ImagePublisher.lovitura = 0
            #         self.get_logger().info('inside : "%d"' % center[0])
            #         Send_msg_motor().publish_message('S0')
                

            #     # move around following the ball anywhere in the field, position greater than the half of the field
            #     if y > 83 and y < 165 and x_line == 404 and center > (200,y) and center < (410,y):
            #         self.get_logger().info('px_ball : "%d"' % y)
            #         self.get_logger().info('px_to_mm : "%d"' % ImagePublisher.from_px_to_mm_16(y))
            #         value = (int(ImagePublisher.interpolate_stepper_steps_16( ImagePublisher.from_px_to_mm_16(y))))
            #         self.get_logger().info('Linear : "%d"' % value) 
            #         result_string = f"A{value}"
            #         Send_msg_motor().publish_message(result_string)
        

                
        # cv2.imshow("Android_cam", res)
        # cv2.imshow("grey img",img_hsv)
        
        # scale_percent = 50 # scale image to 50% of original size
        # width = int(img.shape[1] * scale_percent / 100)
        # height = int(img.shape[0] * scale_percent / 100)
        # dim = (width, height)
        # img = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
        # cv2.imshow("rezultat",img)
    # draw centerline & football quads of thr Goal Keeper 
    # & publish img to topic
        cv2.line(img, (405, 0), (405, 240), (198, 125, 46), 2)
        cv2.line(img,(0,120),(430,120),(200,80,176),2)
        cv2.line(img,(350,110),(400,110),(10,10,190))
        cv2.rectangle(img, (376, 85), (424, 159), (98, 0, 255), 3)
        cv2.line(img, (0, ImagePublisher.y_from_slider), (800, ImagePublisher.y_from_slider), (64, 80, 255), 1)
        # self.pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        # self.get_logger().info('Publishing...')
        cv2.imshow("Manual_motor",img)
        cv2.imshow("res",res)
        
        

def main(args=None):
    thread = Thread(target= slider_for_debug)
    thread.start()
    rclpy.init(args=args)
    my_publisher = Send_msg_motor()
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    # my_publisher.destroy_node()
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()