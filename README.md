# Mip
UCSD Mae 144 Embedded Control and Robotics

Professor Thomas Bewley 

https://www.ucsdrobotics.org/edumip


Description

This is the main balance code designed to balance a mobile inverted pendulum, mainatin its position over a set point, and correct for turning.

The controller was designed and analyzed using MATLAB and implemented in C using Robot Control Library developed by James Strawson

Main 3 components include a fast, high priority interrupt function that keeps the robot upright, a slower low priority outer loop that prevents robot from drifting away from initial point, and a separate tracker that corrects for changes in turning.

Additional features include check for start conditions, wheel saturation timeout, steering input max, battery check, tip angle check, and controller engagement.

State estimation is achieved using the onboard IMU and encoders. A complementary filter applied to both gyroscope and accelerometer estimates the body angle. The wheel position is calculated using optical encoder values and geometry to obtain distance from initial set point.



