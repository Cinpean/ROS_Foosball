# import cv2
# import cv2 as cv
# import sys
# import numpy as np
#
# img = cv.imread(cv.samples.findFile("starry_night.jpg"))
# if img is None:
#  sys.exit("Could not read the image.")
# cv.imshow("Display window", img)
# k = cv.waitKey(0)
# if k == ord("s"):
#  cv.imwrite("starry_night.png", img)
#
# alpha_img= cv2.imread('starry_night.jpg',cv2.IMREAD_COLOR)
# print('RGB shape', alpha_img.shape)
#
# grey_img= cv2.imread('starry_night.jpg',cv2.IMREAD_GRAYSCALE)
# print('Grey', grey_img.shape)
#
# gray= cv2.cvtColor(img,cv2.COLOR_RGBA2GRAY)
# cv2.imshow('original', img)
# cv2.imshow('grayscale',gray)
# resize= cv2.resize(img,(800,800))
# cv2.imshow('resize',resize)
#
# color_change= cv2.cvtColor(img,cv2.COLOR_RGB2LAB)
# cv2.imshow('color_change',color_change)
#
#
#
# cv2.waitKey(0)

# /////////////////////////////////////////////////////////////////////////////////////////
# video from ip webcam and object recognition

import requests
import cv2
import numpy as np
import imutils
from matplotlib import pyplot as plt

# # Replace the below URL with your own. Make sure to add "/shot.jpg" at last.
url = "http://192.168.2.160:8080/shot.jpg"
#
# # While loop to continuously fetching data from the Url
while True:
    img_resp = requests.get(url)
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    img = cv2.imdecode(img_arr, -1)
    img = imutils.resize(img, width=1000, height=1800)


    # Press Esc key to exit
    if cv2.waitKey(1) == 27:
        break

    img_gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    img_rgb= cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    cv2.imshow("grey img",img_rgb)

    face_data= cv2.CascadeClassifier('haarcascade_eye_tree_eyeglasses.xml')
    found= face_data.detectMultiScale(img_gray,minSize=(20,20))
    amount_found= len(found)

    if amount_found !=0:
        for(x,y,width,height) in found:
            cv2.rectangle(img,(x,y), (x+height, y+width), (0,255,0),5)
            cv2.imshow("Android_cam", img)

cv2.destroyAllWindows()
# //////////////////////////////////////////////////////////////////////////
# get images from WEB
# from icrawler.builtin import BingImageCrawler
#
# # //we are building cats image detection that's why we put cat here
# # //if you want some other images then put that name in classes list
# storage_dir= r'C:\Users\sebyc\Documents\Cursuri\Fussball\Proj1\training pictures\p'
# classes=['cats images']
# number=100
# # //here root directory is find your root directory there u will find
# # //new file name data in which all images are saved.
# for c in classes:
#     bing_crawler=BingImageCrawler(storage={'root_dir':f'p/{c.replace(" ",".")}'})
#     bing_crawler.crawl(keyword=c,filters=None,max_num=number,offset=0)