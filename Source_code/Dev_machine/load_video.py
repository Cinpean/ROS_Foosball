import os.path

import cv2


# Path to your MP4 video file
video_path =('mijloc.mp4')

# Create a VideoCapture object
cap = cv2.VideoCapture(video_path)

# Check if the video capture object is successfully opened
if not cap.isOpened():
    print("Error: Could not open video file.")
    exit()

# Loop to read frames from the video
while True:
    # Read a frame from the video
    ret, frame = cap.read()

    # Check if the frame is successfully read
    if not ret:
        print("End of video.")
        break

    # Display the frame or perform any processing as needed
    cv2.imshow('Video', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break

# Release the VideoCapture and close all windows
cap.release()
cv2.destroyAllWindows()
