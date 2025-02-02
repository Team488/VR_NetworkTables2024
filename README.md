# VR_NetworkTables2024
This project provides for integration of virtual reality hardware and RoboRIO network tables. The objective of this integration is to provide a way of using VR hardware tracker technology to deliver an independent source of pose ground truth in the pursuit of better robot localization. 

## Hardware
This has been tested using a VR tracker from Tundra Labs (https://tundra-labs.com/products/additional-tracker). This is a 6 DOF tracker that works with SteamVR lighthouse technology. 
## Configuration
- This was tested using the FRC WPILib version of vscode.
- Install standard FRC software to configure a windows PC as an FRC software development and drive station system

- Install AdvantageScope (if not already available)
- Install HTC Vive SteamVR https://www.vive.com/us/support/vive/category_howto/setting-up-for-the-first-time.html
- Install Python (if not already available)
- Install Python packages

- Follow this to install Python packages for FRC: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html

- pip install openvr OR download the installer at https://github.com/cmbruns/pyopenvr/releases
  
```
Here is a pip list from a known working environment
Package                  Version
------------------------ -----------
appdirs                  1.4.4
bcrypt                   4.2.0
cffi                     1.17.1
colorama                 0.4.6
contourpy                1.3.0
cryptography             43.0.1
cycler                   0.12.1
flexcache                0.3
flexparser               0.3.1
fonttools                4.53.1
iniconfig                2.0.0
kiwisolver               1.4.7
matplotlib               3.9.2
numpy                    2.1.1
openvr                   2.5.101
packaging                23.2
paramiko                 3.5.0
pillow                   10.4.0
Pint                     0.24.3
pip                      24.2
pluggy                   1.5.0
pycparser                2.22
pyfrc                    2024.0.1
PyNaCl                   1.5.0
pynetconsole             2.0.4
pyntcore                 2024.3.2.1
pyparsing                3.1.4
pytest                   8.3.3
pytest-reraise           2.1.2
python-dateutil          2.9.0.post0
robotpy                  2024.3.2.2
robotpy-cli              2024.0.0
robotpy-hal              2024.3.2.1
robotpy-halsim-gui       2024.3.2.1
robotpy-installer        2024.2.2
robotpy-wpilib-utilities 2024.1.0
robotpy-wpimath          2024.3.2.1
robotpy-wpinet           2024.3.2.1
robotpy-wpiutil          2024.3.2.1
setuptools               72.1.0
six                      1.16.0
tomli                    2.0.2
tomlkit                  0.13.2
typing_extensions        4.12.2
wheel                    0.43.0
wpilib                   2024.3.2.1
```

## Setup Hardware Environment

- Run the Steam environment; launch Library->SteamVR application
- Plug-in lighthouse base stations and position them looking down on the tracker (optionally use 12V batteries to power them; a 5V->12V DC-DC converter is needed if using a 5V USB battery)
- Place the trackers in an area with line-of-sight to the base stations
- plug the tracker dongle(s) into the PC running the test
- press/hold the button on each tracker to power-up. Light should go from blue (searching) to green (connected)
- At this point, you should have green lights on the lighthouses and trackers and they should be green in the SteamVR window on your PC

## Running the tracker_test program
- Open tracker_test.py in VSCode
### Calibrate trackers to the FRC field orientation
- Affix tracker_1 to the calibration device (1 meter long metal arm that rotates on a plastic base)
- Stick the plastic base to the carpet (via velcro) and line the arm up so that it is pointing up precisely in the +y-axis of the FRC field; it can be anywhere in the field, but tracker_1 must have line of site to the lighthouses.
- run tracker_test.py -v
- This will initiate a calibration mode. First, you will see if tracker_1 and tracker_2 have been discovered. If not, end the program and make sure that you have green lights on all of the lighthouses and trackers (you must have tracker_1; tracker_2 is optional).
- Once you have verified that tracker_1 was found, hit ENTER to initiate gathering calibration data points
- tracker_test automatically starts reading points from the tracker (you can see them in the terminal). Move the calibration arm clockwise, in a steady circle, two times around the center. It should only take a few seconds. You can stop momentarily, but try to be real smooth.
- tracker_test will automatically stop when it has enough circle points to calibrate; You will see 3 plots showing you the quality of the results. you should see offset circles, then a plot of the rotation angles, then a final overlay of circles. close each plot window to see the next one.
- If the calibration is successful, you should see a new transform.txt file written to the folder. This calibration the calibration parameters for rotation, scaling, and translation. This supplies the transformations for mapping tracker coordinates to FRC field coordinates.
- this calibration must be repeated if the lighthouse base stations are moved or power-cycled.
- Once calibration is complete, exit tracker_test (Ctrl-C in the terminal window or end-program using VSCode GUI)
### Live Tracking
In this step, we will restart tracker_test using some options that will:
1. Read in the calibration parameters from transform.txt (-f transform.txt)
2. Zero the tracker coordinates to a know location (-z)
3. Stream the tracker positions to a NetworkTables server at a specified IP Address (-a <IP Address>); typically 127.0.0.1 for running locally with a simulator, or an FRC address such as 10.4.88.2 when running on a live robot.

To send to a NetworkTables server on your local machine, you must start the robot simulator
- run the python robot simulator (py -3 -m robotpy sim). See https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html
- Place tracker_1 in the predefined location for the -z option (e.g., a know location for an AprilTag; you will need to verify this location by looking at the code)
- run tracker_test.py -v -z -f transform.txt -a 127.0.0.1
- Hit ENTER as prompted in the terminal window

run AdvantageScope
  - Set IP Address (Help->Show Preferences->roboRIO Address: 127.0.0.1)
  - File->Connect to Simulator
  - "+" Odometry view
  - drag tracker_1 and or tracker_2 pose to view

To send to a NetworkTables server on your robot, you must 
- Boot the robot; this starts a NetworkTables server that will be visible on the robot's IP addess
- Connect the PC that will be running tracker_test to the robot's WiFi (e.g., robot SSID or IP such as "10.4.88.2")
- Place tracker_1 in the predefined location for the -z option (e.g., a know location for an AprilTag; you will need to verify this location by looking at the code)
- run tracker_test.py -v -z -f transform.txt -a 10.4.88.2
- Hit ENTER as prompted in the terminal window

run AdvantageScope
  - Set IP Address (Help->Show Preferences->roboRIO Address: 10.4.88.2)
  - File->Connect to Robot
  - "+" Odometry view
  - drag tracker_1 and or tracker_2 pose to view

- You can place additional poses from the robot (e.g., wheel odometry) and compare with the tracking pose
- Mount tracker_1 or tracker_2 to the center of the robot to track its pose in real time
- Use tracker_1 or tracker_2 to check the location of landmarks on the field, such as AprilTags

 
## References
- https://github.com/TriadSemi/triad_openvr
- https://github.com/cmbruns/pyopenvr
