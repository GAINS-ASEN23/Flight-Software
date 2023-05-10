# GAINS Flight Software

### University of Colorado Boulder

### Ann and H.J. Smead Dept. of Aerospace Engineering

### Fall 2022 / Spring 2023 - Senior Design Project - ASEN 4018 / 4028

Bennett Grow, Addison Woodard, Jason Popich, Kaylie Rick, Derek Popich, Cannon Palmer, Tucker Peyok, Ben McHugh, Alfredo Restrepo, Alexander Pichler, Ross White, and Brian Trybus
 
 ---

Project GAINS aims to determine near real-time position
and velocity estimation of a CubeSat beyond GEO with relatively low power and size while maintaining a high level of accuracy with the help of intermittent ground station updates. Clohessy-Wiltshire (CW) dynamics predict the state of the satellite/inertial navigation system (INS) by determining the deviations of the INS from a nominal circular orbit. During periods of ground contact aquisition, the external ground software sends state updates treated as truth data into the INS. During satellite thrusting in periods of loss of contact, an accelerometer measures thrust. Accelerations, ground software state updates, and CW dynamics are incorperated into a Kalman Filter (KF) to produce a final estimate. 

The code in this repository falls into three main categories:

1. Matlab code to develop a Kalman Filter utilizing the Clohessy-Wiltshire equations of motion to determine the position and velocity of a satellites in a lunar orbit.
   
2. C/C++ code to implement the CW KF on a Teensy 4.1 microcontroller.

3. Analysis of physical testing of the Teensy implementation. Analysis completed in Matlab.

### Folder Descriptions

1. **_Archive** - Largely depricated early files from experimentation
2. **Figures** - Relevant images depicting the results from the KF development and testing analysis.
3. **GAINS_FSW** - Main flight software. C/C++ CW KF implementation for a Teensy 4.1 microcontroller. 
4. **Integration Methods** - Exporation of numerical integration methods using continous differentiatable sample data.
5. **Kalman Filter** - Matlab development of the Kalman Filter
6. **Testing - CNC** - Analysis of testing of the KF on the Teensy using data from moving the INS in a circle on a CNC mill to simulate forces from a thrusting satellite. 
7. **Testing - Temperature and Bias Calibration** - Analysis of static heating and cooling the INS to determine calibration parameters to remove bias and scale factor from the accelerometer signals in the FSW.

--- 

## Flight Software

### Prerequisites
- Microsoft VSCode
- PlatformIO extension for VSCode
- GAINS PCB with Teensy 4.1
- Ethernet cable
- Micro-USB cable
- Internet Router
- Ability to set static IP adresses on the router

### Flight Software Initialization
1. Click on the *PlatformIO* button on the left hand side of VSCode
2. Under the *Quick Access* menu, select *PIO Home* > *Open*
3. Select *Open Project*
4. Open the **GAINS_FSW** directory 
5. The PlatformIO extension should initialize
6. Connect the Micro-USB port on the Teensy 4.1 to the computer running VSCode/PIO
7. Connect the both the computer and the ethernet port on the GAINS PCB to the same router/LAN
8. Ensure the Teensy is recognized by the computer
   1. Navigate to *PIO Home*
   2. Select the *Devices* tab on the left
   3. Under *Serial*, click the *Refresh* button and ensure the Teensy is listed (likely "COM#")
9. Click the *Upload* button (symbol: rightwards arrow) on the bottom left of VSCode
10. Once program compilation and upload is complete, click the *Serial Monitor* button (symbol: electrical plug) on the bottom left of VSCode
11. The Serial Monitor should open. After a few seconds the MAC address and other information should be displayed. 
12. Assign the Teensy a static IP address on the network using the MAC address 
13. Configure the correct IP addresses and ports in the *main.cpp* file under the *src*. 
14. Complete steps 9-11 again. The Ground Software should now be recieving packets.

### Library Information

- The main C++ file is under **GAINS_FSW/src/main.cpp**
- *NativeEthernet* is used for external communications: **GAINS_FSW/lib/NativeEthernet**
- *CControl* is used for linear algebra: **GAINS_FSW/lib/CControl**
- *SDFat* is used for writing data to a microSD card using the reader on the Teensy 4.1: **GAINS_FSW/lib/SDFat**
- GAINS original files are under **GAINS_FSW/lib/GAINS**
  - **ccsd** implements the CCSD Space Packet Protocol to encode/decode packets to/from the ground software
  - **GAINSEthernet** wraps the NativeEthernet library and the CCSD packet encoding to provide one interface for GAINS to communicate over ethernet
  - **kf** handles all of the Kalman Filtering operations. See the Matlab scripts under **Kalman Filter** for understanding. Boolean *CW_or_K* configures usage of Clohessy-Wiltshire or kinematic dynamics
  - **operations** contains some helper functions for linear algebra and KF execution
  - **SDRW** handles SD card memory operations
  - **sensor** handles reading and processing data from the accelerometer









