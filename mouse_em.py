import os
import struct
from evdev import InputDevice, ecodes
import threading
import math
import time
class MouseController(threading.Thread):
    def __init__(self, mouse_device_path='/dev/input/event6', hidg_path='/dev/hidg0'):
        self.mouse_device_path = mouse_device_path
        self.hidg_path = hidg_path
        self.mouse = None
        self.hidg = None
        self.btn_state = 0  # Keep track of the button state
        self.position = [0, 0]  # Start the mouse position at [0, 0]
        threading.Thread.__init__(self)
        self.pause_event = threading.Event()
        self.pause_event.set()  # Thread starts in paused state
        self.exit_flag = threading.Event()
        self.running = True  # Control the run loop
        self.overflowX = 0
        self.sum = 0
        self.overflowY = 0

    def get_position(self):
        with self.lock:
            return self.position.copy()
    
    def open_devices(self):
        
        """Open the mouse device and HID gadget file in binary write mode."""
        self.mouse = InputDevice(self.mouse_device_path)
        self.hidg = open(self.hidg_path, 'wb')

    def close_devices(self):
        """Close the HID gadget file and mouse device."""
        if self.hidg:
            self.hidg.close()
        if self.mouse:
            self.mouse.close()

    def send_hid(self, dx, dy, buttons):
        
        # print(f"{dy} {dx}")
        """Send a HID report for a relative mouse movement."""
        # print("sending hid")
        dx = max(-128, min(127, dx))
        dy = max(-128, min(127, dy))
        # self.position[0] += dx
        # self.position[1] += dy
        report = struct.pack('Bbb', buttons & 0x07, dx, dy)
        self.hidg.write(report)
        self.hidg.flush()
        
    def move_cursor(self, dx, dy):
        
        dx = dx*2.5
        dy = dy*2.5
        self.overflowX +=  (dx % 1)
        self.overflowY +=  (dy % 1)
        
        if self.overflowX > 1:
            self.overflowX -=1
            dx+=1
        if self.overflowY < -1:
            self.overflowY +=1
            dy+=1
        # Compute the relative movement needed
        
        dx = int(math.floor(dx))
        dy = int(math.floor(dy))
        self.sum+=dx
        # print(self.sum)

        # Perform the movement in steps if the distance is too large for a single HID report
        while dx != 0 or dy != 0:
            step_dx = max(-127, min(127, dx))
            step_dy = max(-127, min(127, dy))
            self.send_hid(step_dx, step_dy, self.btn_state)
            dx -= step_dx
            dy -= step_dy
        
                
    def pause(self):
        
        
        self.pause_event.clear()  # Clears the event, pauses the thread

    def resume(self):
        self.pause_event.set()  # Sets the event, resumes the thread
        
       

    def stop(self):
        self.close_devices()
        self.exit_flag.set()

    def run(self):
        """Start the event loop for reading mouse input."""
        
        self.open_devices()
        for event in self.mouse.read_loop():
            
            
            self.pause_event.wait()
            if event.type == ecodes.EV_KEY:
                # Process key/button press events
                if event.code == ecodes.BTN_LEFT:
                    
                    if event.value == 1:
                        self.btn_state |= 0x01  # Button pressed
                    else:
                        self.btn_state &= ~0x01  # Button released
                elif event.code == ecodes.BTN_RIGHT:
                    
                    if event.value == 1:
                        self.btn_state |= 0x02
                    else:
                        self.btn_state &= ~0x02
                elif event.code == ecodes.BTN_MIDDLE:
                    if event.value == 1:
                        self.btn_state |= 0x04
                    else:
                        self.btn_state &= ~0x04
                self.send_hid(0, 0, self.btn_state)  # Send HID report with updated button state

            elif event.type == ecodes.EV_REL:
                # Process relative movement events
                if event.code == ecodes.REL_X:
                    
                    self.send_hid(event.value, 0, self.btn_state)  # X movement
                elif event.code == ecodes.REL_Y:
                    self.send_hid(0, event.value, self.btn_state)  # Y movement
                



# Usage in another file:
# from mouse_controller import MouseController
# mc = MouseController()
# # mc.run()  # to start listening and handling mouse events
# # current_position = mc.get_position()  # retrieve current mouse position
# # mc.move_cursor(100, 50)  # example to move cursor
# mc.run()