import cv2
import numpy as np
from mouse_em import MouseController
from threading import Thread

mc = MouseController()
mouse_thread = Thread(target=mc.run)
mouse_thread.start()  # This will start listening and handling mouse events in the background

# current_position = mc.get_position()  # retrieve current mouse position
# mc.move_cursor(100, 50)  # example to move cursor
def open_camera(device, width, height, fps):
    gst_str = (
        f"v4l2src device={device} ! "
        f"image/jpeg, width=(int){width}, height=(int){height}, framerate=(fraction){fps}/1 ! "
        f"jpegdec ! videoconvert ! "
        f"appsink"
    )
    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
    return cap

cap = open_camera('/dev/video0', 1920, 1080, 60)

# offset_x = int(0.01 * 1920)
roi_width, roi_height = 200, 200  # width and height of the ROI
roi_x = (1920 - roi_width) // 2  # X coordinate of the top-left corner of the ROI
roi_y = (1080 - roi_height) // 2 # Y coordinate of the top-left corner of the ROI

if not cap.isOpened():
    print("Error: Could not open video capture device")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break

    roi = frame[roi_y:roi_y + roi_height, roi_x:roi_x + roi_width]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Define your custom HSV color bounds
    HSVCustomLower = np.array([30, 125, 150])
    HSVCustomUpper = np.array([30, 255, 255])

    # Threshold the HSV image to get only colors within your custom bounds
    mask = cv2.inRange(hsv, HSVCustomLower, HSVCustomUpper)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Determine the extreme points by considering all contours
    min_x, min_y = float('inf'), float('inf')
    max_x, max_y = -float('inf'), -float('inf')

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)

        min_x, min_y = min(min_x, x), min(min_y, y)
        max_x, max_y = max(max_x, x + w), max(max_y, y + h)
        shifted_contour = contour + [roi_x, roi_y]
        cv2.drawContours(frame, [shifted_contour], -1, (0, 0, 255), 2)
        print("her1")
        
    if min_x < float('inf') and max_x > -float('inf'):
        # Draw the overall bounding box
        cv2.rectangle(frame, (roi_x + min_x, roi_y + min_y), (roi_x + max_x, roi_y + max_y), (255, 0, 0), 2)  # Bounding box in blue
        
        # Calculate the center of the overall bounding box
        cX = roi_x + min_x + (max_x - min_x) // 2
        cY = roi_y + min_y + (max_y - min_y) // 2

        # Draw a circle at the center of the bounding box
        cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)  # Center marked in white
        mc.move_cursor(cX, cY, (1920,1080))  # example to move cursor 

    print("here")
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
mouse_thread.join()  # Wait for the mouse handling thread to finish if necessary
