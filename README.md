# djh_ins
Written by: David Hanley

This repository contains a C++ class for inertial navigation systems. 

This code takes in IMU measurements and (based on a IMU and gravity model) integrates those IMU measurements into estimated state.

A detailed description of the code can be found [here](TBD).

Instructions for installing (in plain CMAKE) and testing the code in a ROS package can be found [here](https://github.com/hanley6/djh_ins/wiki).

Our code is designed to run with multiple gravity models and Euler, Heun's, and Runge-Kutta integration. It can be run as a batch of IMU measurements or it can be called each time an IMU measurement is made.

In the future, we intend to include code for INS as described by Paul Savage and Preintegration. We have created space for that in our code, but as of October 17, 2017 those features have not been implemented.
