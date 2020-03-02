#libraries imported
import cv2
import numpy as np
import math
import sys
import RPi.GPIO as GPIO

#enabling video capture through USB camera
video = cv2.VideoCapture(0)
#GPIO pins as digital outputs
r = 18
l = 16
GPIO.setmode(GPIO.BOARD)
GPIO.setup(r, GPIO.OUT,initial=0)
GPIO.setup(l, GPIO.OUT,initial=0)

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]


def average_slope_intercept(frame, lines):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if lines is None:
        print("No line_segment segments detected")
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/10
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

    for line_segment in lines:
        for x1, y1, x2, y2 in line_segment:
            #if x1 == x2:
                #logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                #continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))
    if lane_lines is not None:
        for line in lane_lines:
            x1, y1, x2, y2 = line[0]
            
            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
   
    print(lane_lines)
    



#giving gpio output based on the slope and intercepts    
    
    if len(lane_lines) == 0:
        print("No lane lines detected, do nothing")
        GPIO.output(l, GPIO.HIGH)
        GPIO.output(r, GPIO.HIGH)

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        print("Only detected one lane line, just follow it")
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel
    
    if steering_angle <= 89:
        GPIO.output(l, GPIO.LOW)
        GPIO.output(r, GPIO.HIGH)
    elif steering_angle == 90:
        GPIO.output(l, GPIO.HIGH)
        GPIO.output(r, GPIO.HIGH)
    else:
        GPIO.output(l, GPIO.HIGH)
        GPIO.output(r, GPIO.LOW)
        
        
    #return steering_angle





while True:
    ret, orig_frame = video.read()
    if not ret:
        video = cv2.VideoCapture(0)
        continue

    frame = cv2.GaussianBlur(orig_frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low_yellow = np.array([161, 155, 84])
    up_yellow = np.array([179, 255, 255])
    mask = cv2.inRange(hsv, low_yellow, up_yellow)
    edges = cv2.Canny(mask, 75, 150)

    lane_lines = cv2.HoughLinesP(edges, 1, np.pi/180, 35, maxLineGap=50)
    
    lessline = average_slope_intercept(frame, lane_lines)
    
    
    
    

    
   

    cv2.imshow("frame", frame)
    

    key = cv2.waitKey(1)
    if key == 27:
        break

GPIO.cleanup()  
print("Exiting...")
video.release()
cv2.destroyAllWindows()