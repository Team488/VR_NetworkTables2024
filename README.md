# VR_NetworkTables2024
This project provides for integration of virtual reality hardware and RoboRIO network tables. The objective of this integration is to provide a way of using VR hardware tracker technology to deliver an independent source of pose ground truth in the pursuit of better robot localization.

## Hardware
This has been tested using a VR tracker from Tundra Labs (https://tundra-labs.com/products/additional-tracker). This is a 6 DOF tracker that works with SteamVR lighthouse technology.
## Configuration
- This was tested using the FRC WPILib version of vscode.
- Install standard FRC software to configure a windows PC as an FRC software development and drive station system

- Install AdvantageScope (if not already available)
- Install Steam from the official website https://store.steampowered.com/
- Launch Steam, Step up an account, search for SteamVR in the Steam Store and Install SteamVR
- Once the install for SteamVR is finished, exit out of the Steam Application, and also make sure it's not running in the background
- next find the steamvr.vrsettings file by navigating to the C:\Program Files(x86)\Steam\config\ directory
- replace the vrsettings file in the config directory with the steamvr.vrsettings file that comes along with this repo.
- Install Python (if not already available)
- Install Python packages

- Follow this to install Python packages for FRC: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html

- uv add openvr OR download the installer at https://github.com/cmbruns/pyopenvr/releases

```
Here is a uv package list from a known working environment
Package                  Version
------------------------ -----------
appdirs                  1.4.4
circle-fit               0.2.1
colorama                 0.4.6
icecream                 2.1.10
matplotlib               3.10.8
openvr                   2.12.1401
pyopenxr                 1.1.5301
robotpy                  2026.2.1
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
2. the -z parameter sets the starting position of tracker 1 to april tag 18. will need to be adjusted for future FRC seasons
3. Stream the tracker positions to a NetworkTables server at a specified IP Address (-a <IP Address>); typically 127.0.0.1 for running locally with a simulator, or an FRC address such as 10.4.88.2 when running on a live robot.

To send to a NetworkTables server on your local machine, you must start the robot simulator
- run the python robot simulator (see https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html)
   ## py -3 -m robotpy sim
- Place tracker_1 in the predefined location, currently april tag 18, for the -z option (e.g., a know location for an AprilTag; you will need to verify this location by looking at the code)
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
- Place tracker_1 in the predefined location, currently april tag 18, for the -z option (e.g., a know location for an AprilTag; you will need to verify this location by looking at the code)
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
