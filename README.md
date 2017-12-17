# Unscented Kalman Filter

## About
This project implements a PID controller as required by the Udacity's [Self Driving Car Nano-Degree program.](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)

The input data to the PID controller is the cross-track error (CTE), current vehicle velocity, and current steering angle. The PID controller aims to keep the vehicle at the center of the track at all times by manipulating the steering angle.
Additionally, the throttle (i.e., brake and acceleration) input the car has been manipulated based on the requested steering angle. The underlying aim is to recognize that high steering angles indicate a curve, and therefore, the vehicle must slow down in order to remain on the track. 
Of course, a second PID controller to modulate the throttle inputs would be possible, but has not been implemented yet.

The PID controller was tuned using the [coordinate ascent or the "Twiddle"](https://www.udacity.com/file?file_key=agpzfnVkYWNpdHl1ckcLEgZDb3Vyc2UiBWNzMzczDAsSCUNvdXJzZVJldiIHZmViMjAxMgwLEgRVbml0GLG0EwwLEgxBdHRhY2hlZEZpbGUY4ZgXDA) algorithm. [Video Lecture] (https://www.youtube.com/watch?v=2uQ2BSzDvXs)


### How it Looks
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). Select "PID Controller" option.

A sample frame from one of the videos as the vehicle negotiates the circuit inside the simulator is ![shown](https://github.com/RomanoViolet/Udacity-PID-Controller/blob/master/Results/screenshot.png)


### Prerequisites
_Ad-verbatim from Udacity's Instructions:_

uWebSocketIO Starter Guide

All of the projects in Term 2 and some in Term 3 involve using an open source package called [uWebSocketIO](https://github.com/uNetworking/uWebSockets). This package facilitates the same connection between the simulator and code that was used in the Term 1 Behavioral Cloning Project, but now with C++. The package does this by setting up a web socket server connection from the C++ program to the simulator, which acts as the host. In the project repository there are two scripts for installing uWebSocketIO - one for Linux and the other for macOS.

Note: Only uWebSocketIO branch e94b6e1, which the scripts reference, is compatible with the package installation.
Linux Installation:

From the project repository directory run the script: install-ubuntu.sh


### Structure of the Project
The project is structured as follows:
- Folder "src": Contains the core logic required to build the PID Controller application.
- Folder "Results": Contains a sample video and a screenshot when testing the PID controller using the simulator supplied by Udacity.
- Folder "Python": The implementation of the Twiddle algorithm. The python script included therein requires the PID controller binary to be available (but not running), alongwith the simulator with .
- `build.sh`: A shell script to build the PID controller project.
- `install-ubuntu.sh`: A script required for installing dependencies for running the simulator. See the section above on "Prerequisites"
- `run.sh`: A shell script to run the binary built using `build.sh`, and connects to the Udacity's Term 2 simulator. See the section above "How it Looks".

### Implementation of the Twiddle Algorithm
The python script _searchPIDValues.py_ contained in Python/ folder runs the PID controller binary and analyzes the returned metric to be optimized (the Cross Track Error, CTE) in order to move the twiddle algorithm forward. The script _searchPIDValues.py_ is able automatically reset the simulator if the vehicle has diverted too much from the center of the lane as part of the exploration process. Typically, the script can be left to execute on a computer over a period of time as it logs acceptable values values of P, I, and D as per the criteria set (at the moment: successfully completing 5000 frames, where 2500 frames correspond to ~1.1 lap of the circult).


### Running the PID Controller
- In the Twiddle Mode (that is: searching for P,I, and D values):
-- Build the PID application
-- Start the Udacity Term 2 simulator, and select the option "PID Controller"
-- Run _searchPIDValues.py_.

- In the production mode (i.e., PID controller values are known):
-- Set the appropriate values of P,I, and D in main.cpp (lines: 53-55), and build the application,
-- Start the Udacity Term 2 simulator, and select the option "PID Controller"


### Pending Improvements
- Increasing the speed of the vehicle is possible by further tuning of the PID controller, and
- Replacing the current throttle modulation implementation with a PID controller that works alongwith steering-angle PID controller.

### Credits
- Udacity: Lecturers, and mentors;
- Internet: for examples and samples.

### Disclaimer
Some of the ideas are borrowed and adapted from other people's work.
