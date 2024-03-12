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

# Function to calculate extended line coordinates
def extend_line(x1, y1, x2, y2, length):
    # Calculate vector components
    dx = x2 - x1
    dy = y2 - y1
    
    # Calculate unit vector
    magnitude = np.sqrt(dx ** 2 + dy ** 2)
    if magnitude > 0:
        ux = dx / magnitude
        uy = dy / magnitude
    else:
        ux, uy = 0, 0
    
    # Extend line
    x3 = x2 + ux * length
    y3 = y2 + uy * length
    
    return int(x3), int (y3)

# definision of sliders used for moving manually the GK in order to test movement
def slider_for_debug():
    cv2.namedWindow('Manual_motor',cv2.WINDOW_NORMAL)
    cv2.createTrackbar('thrs1', 'Manual_motor', 100, 200, ImagePublisher.callback)   
    cv2.createTrackbar('thrs2', 'Manual_motor', 100, 200, ImagePublisher.callback_2)   
    ch = None
    while ch != 27:
        ch = cv2.waitKey(0)
        if(ch == 96):
            Send_msg_motor().publish_message('E1')
    pass
    Send_msg_motor().publish_message('E1')


# class used for defining the Kalman Filter
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

# class for sending messages to topic used for communication through usb with the motor controller
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
    def increase_brightness(self,hsv, value):
        # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        lim = 255 - value
        v[v > lim] = 255
        v[v <= lim] += value

        final_hsv = cv2.merge((h, s, v))
        # img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return final_hsv
    
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

    def callback_2(x):
        ImagePublisher.y_from_slider = x
        value = (int(ImagePublisher.interpolate_stepper_steps_2( ImagePublisher.from_px_to_mm_16(ImagePublisher.y_from_slider))))
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
        video_config = self.cap.create_video_configuration(main={"size": (426 , 240), "format": "RGB888"})
        video_config["transform"] = libcamera.Transform(hflip=0,vflip=1)
        video_config["controls"] ["FrameDurationLimits"]= (100000, 100000) #around 90 fps ( 1000000 / 90  = 11111 ms/frame)
        video_config["controls"]["ExposureTime"] = 40000 # max exposure time is ~37000
        self.cap.configure(video_config)
        self.cap.start()
        x1, y1 = 30, 7  #23,7
        x2, y2 = 420, 7
        x3, y3 = 420, 238
        x4, y4 = 30, 238 # 23,238
        self.kf = KalmanFilter()
        
        if not ImagePublisher.lovitura:
            ImagePublisher.lovitura = 0

        # Create ROI vertices array
        self.roi_vertices = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]], np.int32)

    # used for mapping read values in regards to stepper driver microstepping _16 stands for 16/1 microstepping
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

        # Apply linear interpolation
        if(d2-d1 != 0):
            ImagePublisher.interpolated_steps = s1 + ((distance - d1) * (s2 - s1) / (d2 - d1))
            
        if(d2-d1 == 0):
            return ImagePublisher.interpolated_steps 
        return ImagePublisher.interpolated_steps

    # used for mapping read values in regards to stepper driver microstepping _4 stands for 4/1 microstepping
    def interpolate_stepper_steps_4(distance):
        table = [(0, 0), (18, 40), (54, 80), (74, 120), (90, 160), (104, 200), (116, 240), (128, 280), (139, 320), (148, 360), (157, 400), (165, 440), (170, 450)]
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

        # Apply linear interpolation
        if(d2-d1 != 0):
            ImagePublisher.interpolated_steps = s1 + ((distance - d1) * (s2 - s1) / (d2 - d1))
            
        if(d2-d1 == 0):
            return ImagePublisher.interpolated_steps 
        return ImagePublisher.interpolated_steps

    # used for mapping read values in regards to stepper driver microstepping _2 stands for 2/1 microstepping
    def interpolate_stepper_steps_2(distance):
        table = [(0, 0), (20, 20), (50, 40), (67, 60), (84, 80), (97, 100), (111, 120), (124, 140), (133, 160), (143, 180), (152, 200), (160, 220), (170, 240)]
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

        # Apply linear interpolation
        if(d2-d1 != 0):
            ImagePublisher.interpolated_steps = s1 + ((distance - d1) * (s2 - s1) / (d2 - d1))
            
        if(d2-d1 == 0):
            return ImagePublisher.interpolated_steps 
        return ImagePublisher.interpolated_steps

    def from_px_to_mm_16(px_val):
        y = ((px_val-90)*159)/69
        return y

    def timer_callback(self):
        img=self.cap.capture_array()

        img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        mask = np.zeros_like(img_gray)
        cv2.fillPoly(mask, [self.roi_vertices], 255)
        masked_frame = cv2.bitwise_and(img, img, mask=mask)

        img_hsv = cv2.cvtColor(masked_frame,cv2.COLOR_BGR2HSV)
        img_hsv = self.increase_brightness(img_hsv,value=80) #trying to imporve brightness
        
        lower_blue= np.array([164,140,65]) #164,88,75  #155,140,70  #155,81,49 for high light enviromnet                                  #old vals 165,118,100
        upper_blue= np.array([179,255,255]) #179,50,255 commented ones, for white and shades of white   #old vals 188,255,255

        mask_blue= cv2.inRange(img_hsv, lower_blue, upper_blue)

        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)


        res= cv2.bitwise_and(img_hsv,img,mask=mask_blue)
        contours, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
            cv2.line(img, (center[0], center[1]), (center[0]+400, center[1]), (64, 80, 10), 4)
            
            x3, y3 = extend_line(center[0], center[1], int(predicted[0]),int(predicted[1]), 80)
            cv2.line(img, (int(predicted[0]),int(predicted[1])), (x3, y3), (250, 250, 0), 2)

            
            for x_line in range(x,430) :
                y = center[1]
                # Y values from 90 & 165 are the extermities from the GoalKeeper penlaty area 
                # used for moving the servomotor in order to hit the ball
                if y > 90 and y < 165 and x_line == 404 and center > (344,y) and center<(420,y) and ImagePublisher.lovitura == 0:
                    self.get_logger().info('inside : "%d"' % y)
                    Send_msg_motor().publish_message('S60')
                    ImagePublisher.lovitura = 1

                if ImagePublisher.lovitura == 1 and center <(354,y) :
                    ImagePublisher.lovitura = 0
                    self.get_logger().info('inside : "%d"' % center[0])
                    Send_msg_motor().publish_message('S0')

                # move around following the ball anywhere in the field, position greater than the half of the field
                # x,y coordonates of the center of the ball 
                if y > 83 and y < 160 and x_line == 404 and center > (170,y) and center < (410,y):
                    
                    # script used for determining the intersection of the prediction line with the GK line
                    if x3 != center[0] and x3 > 390 and y3 > 89 and y3 < 160 :
                        intersection_point_y = int((((y3-center[1])/(x3-center[0]))*(404-center[0]))+center[1])
                        if intersection_point_y > 0: 
                            self.get_logger().info('Intersected w 404 : "%d"' % intersection_point_y)
                            value = (int(ImagePublisher.interpolate_stepper_steps_2( ImagePublisher.from_px_to_mm_16(y3))))
                            result_string = f"B{value}"
                            Send_msg_motor().publish_message(result_string)

                    # if x3 >= 400 and x3<425 and y3 > 89 and y3 < 160 :
                    #     self.get_logger().info('px_ball_prediction : "%d"' % y3)
                    #     self.get_logger().info('px_to_mm_prediction : "%d"' % ImagePublisher.from_px_to_mm_16(y3))
                    #     value = (int(ImagePublisher.interpolate_stepper_steps_2( ImagePublisher.from_px_to_mm_16(y3))))
                    #     self.get_logger().info('Linear_prediction : "%d"' % value)
                        
                    #     result_string = f"B{value}"
                    #     Send_msg_motor().publish_message(result_string)
                    else :
                        self.get_logger().info('px_ball : "%d"' % y)
                        self.get_logger().info('px_to_mm : "%d"' % ImagePublisher.from_px_to_mm_16(y))
                        value = (int(ImagePublisher.interpolate_stepper_steps_2( ImagePublisher.from_px_to_mm_16(y))))
                        self.get_logger().info('Linear : "%d"' % value) 
                        result_string = f"B{value}"
                        Send_msg_motor().publish_message(result_string)

        # draws the path the ball has taken
        # for i in range(1, len(ImagePublisher.object_positions)):
            # cv2.line(img, ImagePublisher.object_positions[i - 1], ImagePublisher.object_positions[i], (0, 255, 0), 2)

        # draw centerline & football quads of the Goal Keeper 
        cv2.rectangle(img, (376, 85), (424, 159), (98, 0, 255), 1)
        cv2.line(img, (0, ImagePublisher.y_from_slider), (800, ImagePublisher.y_from_slider), (64, 80, 255), 1)


        # publish img to topic
        # self.pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
        # self.get_logger().info('Publishing...')

        # print to window the image that the program is seeing
        cv2.imshow("Manual_motor",img)
        cv2.imshow("res",res)
        # cv2.imshow("masked",masked_frame)
        
        

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