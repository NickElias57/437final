import cv2
import numpy as np
import queue
from threading import Thread
import time
from mouse_em import MouseController
from bezier import Bezier
class Detection():
    def __init__(self, device, width, height, fps):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.roi_width = 150
        self.roi_height = 500
        self.mouseForward = MouseController()
        self.mouse_thread = Thread(target=self.mouseForward.run)
        self.mouse_thread.start()
        self.mouseForward.resume()
        self.mouseHack = MouseController() 
        
        self.roi_x = (width - self.roi_width) // 2
        self.roi_y = (height - self.roi_height) // 2
    def open_camera(self):
        
        gst_str = (
            f"v4l2src device={self.device} ! "
            f"image/jpeg, width=(int){self.width}, height=(int){self.height}, framerate=(fraction){self.fps}/1 ! "
            f"jpegdec ! videoconvert ! "
            f"appsink"
        )
        self.cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        self.q = queue.Queue()
        t = Thread(target=self._reader)
        t.daemon = True
        t.start()

        
    def _reader(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()   # discard previous (unprocessed) frame
                except queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
       
        return self.q.get()
    
    
    def run_detection(self):
        time.sleep(10)
        self.open_camera()
        
        try:
            
            while True:
                
                frame = self.read()
                
                roi = frame[self.roi_y:self.roi_y + self.roi_height, self.roi_x:self.roi_x + self.roi_width]
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                HSVCustomLower = np.array([30, 125, 150])   
                HSVCustomUpper = np.array([30, 255, 255])
                mask = cv2.inRange(hsv, HSVCustomLower, HSVCustomUpper)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                startingX= self.width // 2
                startingY= self.height // 2 - 5
                # print("here")
                min_x, min_y = float('inf'), float('inf')
                max_x, max_y = -float('inf'), -float('inf')

                for contour in contours:
                    # print("her2")
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    min_x, min_y = min(min_x, x), min(min_y, y)
                    max_x, max_y = max(max_x, x + w), max(max_y, y + h)
                    shifted_contour = contour + [startingX, startingY]
                    # cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)  # Center marked in white
                # print("hi")
                if min_x < float('inf') and max_x > -float('inf'):
                    
                    cX = self.roi_x + min_x + (max_x - min_x) // 2
                    cY = self.roi_y + min_y + (max_y - min_y) // 6
                    
                    print(cX,cY)
                    if abs(cX - startingX) > 5 or abs(cY - startingY) > 20:
                        print("found bounding box, pausing mouse")
                        self.mouseForward.pause()
                        self.mouseHack.open_devices()
                        time.sleep(.4)
                        frame = self.read()
                        roi = frame[self.roi_y:self.roi_y + self.roi_height, self.roi_x:self.roi_x + self.roi_width]
                        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                        HSVCustomLower = np.array([30, 125, 150])
                        HSVCustomUpper = np.array([30, 255, 255])
                        mask = cv2.inRange(hsv, HSVCustomLower, HSVCustomUpper)

                        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        startingX= self.width // 2
                        startingY= self.height // 2 - 5
                        # print("here")
                        min_x, min_y = float('inf'), float('inf')
                        max_x, max_y = -float('inf'), -float('inf')

                        for contour in contours:
                            # print("her2")
                            x, y, w, h = cv2.boundingRect(contour)
                            
                            min_x, min_y = min(min_x, x), min(min_y, y)
                            max_x, max_y = max(max_x, x + w), max(max_y, y + h)
                            shifted_contour = contour + [startingX, startingY]
                        
                        if min_x < float('inf') and max_x > -float('inf'):
                    
                            cX = self.roi_x + min_x + (max_x - min_x) // 2
                            cY = self.roi_y + min_y + (max_y - min_y) // 6   
                            
                            if abs(cX - startingX) > 20 or abs(cY - startingY) > 20:
                                bezier = Bezier()
                                ctrl1, ctrl2 = Bezier.calculate_control_points((startingX,startingY),(cX,cY),.3)
                                
                                
                                # time.sleep(2)
                                movementX,movementY = Bezier.bezier_curve([(startingX,startingY),ctrl1,ctrl2,(cX,cY)])
                                movementX = movementX[::-1]
                                movementY =movementY[::-1]
                                
                                last = (startingX,startingY)
                                
                                for i in range(len(movementX)):
                                    
                                    
                                    dx = (movementX[i]  - last[0])  
                                    dy = (movementY[i]  - last[1])
                                    # time.sleep(2)
                                    self.mouseHack.move_cursor(dx,dy)
                                    last = (movementX[i],movementY[i])
                                    time.sleep(.004)
                                
                        self.mouseHack.close_devices()
                        self.mouseForward.resume()
                        
                    
        except KeyboardInterrupt:
            
            # Handle Ctrl+C here if needed
            # For example, you might want to stop threads or close resources
            import sys
            self.mouseForward.stop()
            self.cap.release()
            cv2.destroyAllWindows()
            sys.exit(0)
            
            
            # print("hi2")
                

            
            # cv2.imshow("Frame", frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
        
        

det = Detection('/dev/video0', 1280, 720, 20)
det.run_detection() 