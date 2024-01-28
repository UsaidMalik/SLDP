import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
import RPi.GPIO as GPIO
import time
"""These are all the libraries this project uses, going from top to bottom the libraries function to do the following
The open CV library stands for camera vision and serves to add tracking for the robot, the numpy library serves as 
a complementary library to the open cv library, the picamera library functions to add software for a camera connected
to the raspberry pi, the picamera RGB library functions to detect colors on the RGB scale, the RPI.GPIO library 
functions to initializes and set the I/O of the pins on the raspberry pi, the time library functions to add pauses 
in the code"""

GPIO.setmode(GPIO.BCM)
# sets the pin mode of the raspberry pi to GPIO
GPIO.setup(17, GPIO.OUT)
# sets pin 17 to output,
GPIO.setup(27, GPIO.OUT)
# sets pin 27 to output


def ultrasonic_sensor(input_pin, output_pin):
    """this is a function that uses the ultrasonic sensors to detect the distance
    :param input_pin: the input pin on the ultrasonic sensor
    :param output_pin: the ouput pin on the ultrasonic sensor
    :returns the distance detected by the ultrasonic sensors"""
    
    GPIO.setup(input_pin, GPIO.IN)
    # sets the input pin on the ultrasonic sensor to input
    GPIO.setup(output_pin, GPIO.OUT)
    # sets the output pin on the ultraosnic sensor to output
    
    GPIO.output(output_pin, True)
    # sets the output pin to on
    time.sleep(0.00001)
    # adds a pause to not overload
    GPIO.output(output_pin, False)
    # sets the output pin to false
    
    StartTime = time.time()
    # starts the time for when the ultrasonic sensor sends a pulse
    StopTime = time.time()
    # the time the ultrasonic sensor stops
    
    while GPIO.input(input_pin) == 0:
        StartTime = time.time()
    # if the input pin is off the time starts

    while GPIO.input(input_pin) == 1:
        StopTime = time.time()
    # if the output pin is on the time stops

    TimeElapsed = StopTime - StartTime
    # calculates the time that is elapsed

    distance = (TimeElapsed * 34300) / 2
    # stores the distance by using the basic velocity = distance/ time formula in centimeters
    
    return distance
    # finds the distance in centimeters


servo_two = GPIO.PWM(17, 100)
# sets the servo that rotates forward to pin 17 with a 100 Hz cycle
servo_one = GPIO.PWM(27, 100)
# sets the servo that rotates forward to pin 27 with a 100 Hz cycle
GPIO.setwarnings(False)
# sets the warnings to off

while True:
    # starts the loop for the camera
    camera = PiCamera()
    # sets the camera to the camera on the raspberry Pi
    camera.resolution = (640, 480)
    # determines the frame size of the camera
    camera.framerate = 30
    # determines the frame rate of the camera
    rawCapture = PiRGBArray(camera, size=(640, 480))
    # determines what is to be captured
    pixels = 640*480
    # determines the amount of pixels in the camera
    servo_two.start(15)
    # starts the second servo and sets it to the middle of its rotation
    
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # loop to determine the quantities within the frame in a camera
        time.sleep(0.1)
        # stops for a time to not overload the pi
        ultra_sonic_sensor_left = ultrasonic_sensor(2, 3)
        # sets the ultrasonic sensor on the left to pins 2, 3
        ultra_sonic_sensor_right = ultrasonic_sensor(23, 24)
        # sets the ultrasonic sensor on the right to pins 23, 24
        frame = frame.array
        # sets the frame
        
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # sets the color scale

        low_red = np.array([161, 155, 84])
        # sets the lower range of the color red
        high_red = np.array([179, 255, 255])
        # sets the higher range of the color red

        left_frame_red = cv2.cvtColor(frame[0:480, 0:320], cv2.COLOR_BGR2HSV)
        # the left half of the camera
        right_frame_red = cv2.cvtColor(frame[0:480, 320:640], cv2.COLOR_BGR2HSV)
        # the right half of the camera
        left_red_mask = cv2.inRange(left_frame_red, low_red, high_red)
        right_red_mask = cv2.inRange(right_frame_red, low_red, high_red)
        total_red_mask = cv2.inRange(hsv_frame, low_red, high_red)
        # masks to show the amount of red on a camera
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


        red_lower = np.array([150, 30, 50])
        red_upper = np.array([180, 255, 255])
        # different range
        
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        
        result_red = cv2.bitwise_and(frame, frame, mask=red_mask)
        # shows the result
        cv2.imshow("child monitor", result_red)
        # shows the child on the screen

        time.sleep(0.01)
        # sleeps for a time
        
        if(cv2.countNonZero(red_mask)>(0.3*pixels)):
            # checks if 30% of camera is covered
            servo_one.start(0)
            # starts the back servo
            servo_one.ChangeDutyCycle(55)
            # moves robot forward
            time.sleep(0.01)
            # sleeps for a time
            if ultra_sonic_sensor_left < 10 or cv2.countNonZero(left_red_mask) > 1.2*cv2.countNonZero(right_red_mask):
                # above checks if there is too little distance on the left or too many red pixels on the left
                angle_rotate_ultra_sonic = ultra_sonic_sensor_left + 5
                # determines the angle to rotate
                angle_rotate_red_pixels = (10/.45)(cv2.countNonZero(right_red_mask)/cv2.countNonZero(total_red_mask))
                # determines the angle to rotate
                if angle_rotate_ultra_sonic >= angle_rotate_red_pixels:
                    # determines which rotation angle is greater and moves there
                    servo_two.ChangeDutyCycle(angle_rotate_ultra_sonic)
                    # rotates the dog to the right
                else:
                    servo_two.ChangeDutyCycle(angle_rotate_red_pixels)
                    # rotates the dog to the right
            elif ultra_sonic_sensor_right < 10 or \
                    cv2.countNonZero(left_red_mask)*1.2 < cv2.countNonZero(right_red_mask):
                # above checks if there is too little distance on the right or too many red pixels on the right
                angle_rotate_ultra_sonic = ultra_sonic_sensor_right + 15
                # determines angle of rotation
                angle_rotate_red_pixels = (10/.40)(cv2.countNonZero(left_red_mask)/cv2.countNonZero(total_red_mask))
                # determines angle of rotation
                if angle_rotate_ultra_sonic >= angle_rotate_red_pixels:
                    # determines which angle to rotate is larger
                    servo_two.ChangeDutyCycle(angle_rotate_ultra_sonic)
                    # rotates the dog to the right
                else:
                    servo_two.ChangeDutyCycle(angle_rotate_red_pixels)
                    # rotates dog to the right
        else:
            servo_one.stop()
            # stops the dog if there is no red
            servo_one = GPIO.PWM(27, 100)
            # initializes the motor

        key = cv2.waitKey(1)
        # sets the key
        rawCapture.truncate(0)
        # resets the stream
                 
        if key == 27:
            # if the key is pressed the loop is broken
            cv2.destroyAllWindows()
            break
    break
    # the loop breaks if the camera stops
    
