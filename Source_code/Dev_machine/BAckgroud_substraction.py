import cv2
import requests
import numpy as np
import imutils


url = "http://192.168.2.160:8080/shot.jpg"

# Create a VideoCapture object to capture video from a webcam or a video file
 # Use 0 for the default webcam

# Create a background subtractor object
background_subtractor = cv2.createBackgroundSubtractorMOG2()

# img_resp = requests.get(url)
# img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
# frame = cv2.imdecode(img_arr, -1)
# frame = imutils.resize(frame, width=800, height=1000)
# fg_mask2 = background_subtractor.apply(frame)

while True:
    # Read a frame from the video source
    img_resp = requests.get(url)
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    frame = cv2.imdecode(img_arr, -1)
    frame = imutils.resize(frame, width=800, height=1000)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue= np.array([30,40,40]) #0,0,150
    upper_blue= np.array([85,255,255])
    mask_blue= cv2.inRange(hsv, lower_blue, upper_blue)

    # Apply background subtraction
    fg_mask = background_subtractor.apply(frame)

    combined_mask = cv2.bitwise_and(mask_blue, fg_mask)

    # Optionally, you can further process the result (e.g., apply thresholding)
    # _, binary_mask = cv2.threshold(fg_mask, 128, 255, cv2.THRESH_BINARY)

    # Display the original frame and the result
    cv2.imshow('Original Frame', frame)
    cv2.imshow('Foreground Mask', combined_mask)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture and close all windows
cap.release()
cv2.destroyAllWindows()