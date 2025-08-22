# -*- coding: utf-8 -*-
# Gerekli kütüphaneleri içe aktar
import cv2
import numpy as np
import time
from pymavlink import mavutil
import os
from gpiozero import DigitalInputDevice

# --- GPIO and Telemetry Settings ---
# Set the Raspberry Pi GPIO pin number for the button.
# For example, use 17 for GPIO 17.
TRIGGER_PIN = 17

# Set the serial port for MAVLink connection.
TELEMETRY_PORT = "/dev/ttyUSB0"
TELEMETRY_BAUD = 57600

# Create MAVLink connection. Does not stop the code from running in case of an error.
master = None
try:
    master = mavutil.mavlink_connection(TELEMETRY_PORT, baud=TELEMETRY_BAUD)
    print("Waiting for MAVLink connection...")
    master.wait_heartbeat(timeout=5)
    print("MAVLink connection established successfully.")
except Exception as e:
    print(f"MAVLink connection failed: {e}")
    master = None

# --- Camera and Image Settings ---
# Note: Used only for color detection, the image is not displayed.
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_AUTO_WB, 1)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Adjust this variable to True/False according to the environment
USE_OUTDOOR = False

# Using your preferred color ranges
if USE_OUTDOOR:
    lower_red1, upper_red1 = (0, 140, 110), (10, 255, 255)
    lower_red2, upper_red2 = (170, 140, 110), (179, 255, 255)
    lower_green, upper_green = (40, 120, 110), (85, 255, 255)
    lower_black, upper_black = (0, 0, 0), (179, 70, 45)
else:
    lower_red1, upper_red1 = (0, 120, 80), (10, 255, 255)
    lower_red2, upper_red2 = (170, 120, 80), (179, 255, 255)
    lower_green, upper_green = (36, 80, 80), (85, 255, 255)
    lower_black, upper_black = (0, 0, 0), (179, 80, 55)

kernel = np.ones((5,5), np.uint8)

def clean(mask):
    """Performs morphological operations (opening and closing) to clean noise."""
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    return mask

def detect_color(roi):
    """Detects the most dominant color (Black, Red, Green or Undefined) in the ROI."""
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_black = cv2.inRange(hsv, lower_black, upper_black)

    mask_red = clean(mask_red)
    mask_green = clean(mask_green)
    mask_black = clean(mask_black)
    
    min_area = int(roi.shape[0]*roi.shape[1]*0.005)

    def max_contour_area(mask):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return 0
        return max((cv2.contourArea(c) for c in cnts))

    red_area = max_contour_area(mask_red)
    green_area = max_contour_area(mask_green)
    black_area = max_contour_area(mask_black)
    
    areas = {"KIRMIZI": red_area, "YESIL": green_area, "SIYAH": black_area}
    max_area_label = max(areas, key=areas.get)
    max_area_value = areas[max_area_label]
    
    if max_area_value > min_area:
        return max_area_label, max_area_value / (roi.shape[0] * roi.shape[1])
    else:
        return "BELIRSIZ", 0.0

def send_mavlink_message(label, conf):
    """Sends a MAVLink STATUSTEXT message with the detected color and confidence."""
    if master is None:
        return
    message = f"Detected: {label} (Confidence: {conf:.2f})"
    master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, message.encode())

# --- Main Loop ---
# Global variables to handle pin events
global pulse_width_us, pulse_start_time
pulse_width_us = 0.0
pulse_start_time = 0.0

# Timer variables for state management
ACTIVATION_THRESHOLD = 1500
DEACTIVATION_THRESHOLD = 1500
TIMER_THRESHOLD = 2.0  # 2 seconds
active_start_time = None
idle_start_time = time.time()
current_status = "Boşta"
last_status = "Boşta"

# Pin event handlers
def pin_activated():
    global pulse_start_time
    pulse_start_time = time.time()

def pin_deactivated():
    global pulse_width_us
    if pulse_start_time > 0:
        pulse_width_us = (time.time() - pulse_start_time) * 1000000

pin = DigitalInputDevice(TRIGGER_PIN)
pin.when_activated = pin_activated
pin.when_deactivated = pin_deactivated

last_detected_color = "BELIRSIZ"
last_detected_conf = 0.0

# Clear the terminal and print the header
def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

clear_screen()
print("-------------------------------------")
print(" Live Color Detection and Status Screen ")
print("-------------------------------------")
print(f"RC Trigger Pin: GPIO {TRIGGER_PIN}")

try:
    while True:
        # Check for state changes with a timer
        if pulse_width_us > ACTIVATION_THRESHOLD:
            if active_start_time is None:
                active_start_time = time.time()
            # If the signal has been consistently high for the threshold time
            if time.time() - active_start_time >= TIMER_THRESHOLD:
                current_status = "AKTİF"
                idle_start_time = None
        else:
            active_start_time = None
            if idle_start_time is None:
                idle_start_time = time.time()
            # If the signal has been consistently low for the threshold time
            if time.time() - idle_start_time >= TIMER_THRESHOLD:
                current_status = "Boşta"
                
        if current_status != last_status:
            last_status = current_status
        
        # Continuously process camera image
        ok, frame = cap.read()
        if not ok:
            print("\nCamera could not be read. Terminating program.")
            break
            
        small_frame = cv2.resize(frame, (320, 240))
        h, w = small_frame.shape[:2]
        roi_ratio = 0.7
        x0 = int((1-roi_ratio)/2 * w)
        x1 = int((1+roi_ratio)/2 * w)
        y0 = int((1-roi_ratio)/2 * h)
        y1 = int((1+roi_ratio)/2 * h)
        roi = small_frame[y0:y1, x0:x1]

        # Color detection is performed continuously.
        current_color, current_conf = detect_color(roi)
        
        # Update display only if the detection changes
        if current_color != last_detected_color:
            last_detected_color = current_color
            last_detected_conf = current_conf

        if current_status == "AKTİF":
            send_mavlink_message(last_detected_color, last_detected_conf)

        # Clear the terminal and print the new status
        print("\033[H\033[J")
        print("-------------------------------------")
        print(" Live Color Detection and Status Screen ")
        print("-------------------------------------")
        print(f"Last Detected Color: {last_detected_color}")
        print(f"Confidence: {last_detected_conf:.3f}")
        print(f"RC Trigger Pin: GPIO {TRIGGER_PIN}")
        print(f"Pulse Width (µs): {pulse_width_us:.2f}")
        
        if master:
            print(f"MAVLink Connection: OK ({TELEMETRY_PORT})")
        else:
            print(f"MAVLink Connection: ERROR")

        print(f"\nRC Trigger Status: {current_status}")

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nProgram terminated by user.")
finally:
    cap.release()
