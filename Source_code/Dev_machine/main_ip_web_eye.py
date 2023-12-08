import requests
import cv2
import numpy as np
import imutils
import os.path
# from kalmanfilter import KalmanFilter
from matplotlib import pyplot as plt
from collections import deque

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

# # Replace the below URL with your own. Make sure to add "/shot.jpg" at last.
url = "http://192.168.2.160:8080/shot.jpg"
cap = cv2.VideoCapture('D:/Documents/Cursuri/Fussball/Proj1/test_tracking/mij-loc.mp4')
#
# # While loop to continuously fetching data from the Url
x=0
y=0
prev_position = None
estimated_velocity = None
time_interval = 1.0
x1, y1 = 200, 100
x2, y2 = 600, 100
x3, y3 = 600, 350
x4, y4 = 200, 350

# Create ROI vertices array
roi_vertices = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]], np.int32)
pts = deque (maxlen=64)

kf = KalmanFilter()
xList = [item for item in range (0,1000)]

while True:
    img_resp = requests.get(url)
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    img = cv2.imdecode(img_arr, -1)
    img = imutils.resize(img, width=800, height=1000)
    # ///////////////////////////////////////
    # cv2.circle(img, (400, 200), 35, (0, 200, 0), thickness=-5)

    # ///load from video file
    # _ , img = cap.read()

    img_gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    mask = np.zeros_like(img_gray)
    cv2.fillPoly(mask, [roi_vertices], 255)
    masked_frame = cv2.bitwise_and(img, img, mask=mask)
    cv2.imshow("Masked Frame for selecting game area", masked_frame)

    # Press Esc key to exit
    if cv2.waitKey(1) == 27:
        break


    # img_rgb= cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    hsv= cv2.cvtColor(img,cv2.COLOR_BGR2HSV)    #replace img with masked_frame for selecting game area
    # cv2.imshow("grey img",hsv)
    x, y, rect_width, rect_height = 100, 100, 200, 150
    # mask = np.zeros_like(hsv[:, :, 0])
    # cv2.rectangle(mask, (x, y), (x + rect_width, y + rect_height), 255, -1)
    # cv2.imshow("un kkt", mask)


    lower_blue= np.array([30,40,40]) #0,0,150
    upper_blue= np.array([85,255,255]) #179,50,255 commented ones, for white and shades of white

    mask_blue= cv2.inRange(hsv, lower_blue, upper_blue)
    res= cv2.bitwise_and(img,img,mask=mask_blue)

    # cv2.imshow("Android_cam", res)  # cu negru

# contour
#     _, threshold= cv2. threshold(img_gray,80,255,cv2.THRESH_BINARY)

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
        center = (x,y)
        # print(center)
        cv2.drawContours(img, [largest_contour], 0, (255, 0, 0), 5)
        cv2.circle(img, (x, y), 5, (0, 0, 255), thickness=cv2.FILLED)
        cv2.putText(img,"obj center",(x,y),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,180,67),2)

        predicted = kf.predict(x, y)
        cv2.circle(img, predicted, 8, (0, 255, 0), 4)
        center_points = []

        for i in range (2):
        # further position prediction with Kalman Filter
            predicted = kf.predict(predicted[0],predicted[1])

            center2 = (int(predicted[0]),int(predicted[1]))
            cv2.circle(img, center2,8,(255,255,0),4)

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
            # print ("X:",A,B,C )
            # if i == len(center_points)-1 and abs(x1-x2) > 3

        for x_line in range(x,801) :
            y = int((-A * x_line - C ) / B)
            cv2.circle(img,(x_line,y), 5 ,(255,0,255), cv2.FILLED)
            # print("y=", y,x)


            # Y values from 150 & 350 and X == 600
            if y > 150 and y <350 and x_line == 600:
                print("inside :",y)

        # draw trail line ///////////////////////////////
        # pts.appendleft(center)
        #
        # for i in range (1,len(pts)):
        #     if pts[i-1] is None or pts[i] is None:
        #         continue
        #
        #     thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
        #     cv2.line(img, pts[i - 1], pts[i], (0, 0, 255), thickness)
        # end draw trail line ///////////////////////////////

        # Predict the next position based on estimated velocity
        # if prev_position is not None and estimated_velocity is not None:
        #     time_interval = 1.0  # You may need to adjust this based on frame rate
        #     next_x = prev_position[0] + (estimated_velocity[0] * time_interval)
        #     next_y = prev_position[1] + (estimated_velocity[1] * time_interval)
        #
        #     # Draw a line indicating the predicted trajectory
        #     cv2.line(img, (x, y), (int(next_x), int(next_y)), (0, 0, 255), 2)

        # Update estimated velocity
        # if prev_position is not None:
        #     position_change = np.array([x, y]) - np.array(prev_position)
        #     estimated_velocity = position_change / time_interval
        #
        # prev_position = (x, y)

    cv2.line(img, (600, 0), (600, 450), (198, 125, 46), 5)
    cv2.line(img,(0,225),(800,225),(200,80,176),4)
    cv2.rectangle(img, (550, 100), (650, 350), (98, 0, 255), 3)

    cv2.imshow("shapes",img)

cv2.destroyAllWindows()
# //////////////////////////////////////////////////////////////////////////
