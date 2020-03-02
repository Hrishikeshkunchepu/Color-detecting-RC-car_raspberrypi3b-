# Color-detecting-RC-car_raspberrypi3b-
control of robotic car using openCV, Raspberrypi3b+ and USB camera
Initial idea: Simulating traffic signal control for autonomous vehicles. Controlling 2 DC motors using raspberrypi3 GPIO. Enabling Computer Vision by employing Color Detection algorithm.
using BGR to HSV conversion to differentiate color.


Tivac LaunchPad has been employed, because the GPIO of Raspberrypi3b+ has 3.3 volts. which do not support the motor driver IC.  serial communication (UART) between TM4c123 microcontroller and Raspberrypi3b+


Alternatevely Optocoupler can also be used instead of Microcontroller.
