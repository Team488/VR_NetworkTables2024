# VR_NetworkTables2024
This project provides for integration of virtual reality hardware and RoboRIO network tables. The objective of this integration is to provide a way of using VR hardware tracker technology to deliver an independent source of pose ground truth in the pursuit of better robot localization. 

## Hardware
This has been tested using a VR tracker from Tundra Labs (https://tundra-labs.com/products/additional-tracker). This is a 6 DOF tracker that works with SteamVR lighthouse technology. 
## Configuration
This was tested using the FRC WPILib version of vscode.
Install standard FRC software to configure a windows PC as an FRC software development and drive station system

Install AdvantageScope (if not already available)
Install HTC Vive SteamVR https://www.vive.com/us/support/vive/category_howto/setting-up-for-the-first-time.html
Install Python (if not already available)
Install Python packages

Follow this to install Python packages for FRC: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html

pip install openvr OR download the installer at https://github.com/cmbruns/pyopenvr/releases

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

## Execution

Plug-in lighthouse base stations and position them looking down on the tracker
Place the tracker in an area with line-of-sight to the base stations
plug the tracker dongle into the PC running the test (the code is currently hard-coded for the tracker/dongle combination #without sticker# labels)

run tracker_test.py
run AdvantageScope
  File->Connect to simulator
  "+" Odometry view
  drag pose to view
  
## References
https://github.com/TriadSemi/triad_openvr
https://github.com/cmbruns/pyopenvr
