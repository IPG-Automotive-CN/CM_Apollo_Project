

# Overview
This project creates a communication bridge between CarMaker and Apollo 7.0 (LG variant).
After compiling this code, you can connect to the resulting executable from CarMaker to connect to the Apollo "bridge".
Please follow the instructions below to use it. 

The source code for this has been written by CavPoint to a demonstrator level. It has not been written to a level for 
production code which would have required a higher level of rigour. It has been provided part-way through the 
development process and is therefore in no way in a state that could be considered 'complete' or 'ready to release'. 
It is provided as-is with no warranties, express or implied.


# Usage Instructions
1. Install Apollo 7.0 (LG version) and build and run the Apollo containers, run the Apollo bridge and open the Apollo Dreamview page
2. Download and install the last version of Protobuf https://github.com/protocolbuffers/protobuf/releases
3. Run the script `generate_protobufs.sh`. This should generate in the src folder : CarMaker.linux64
4. Open CarMaker and make the carmaker-bridge repository the CarMaker Project folder.
  -> In CarMaker go to File -> Project folder -> Select the folder containing this project
5. In CarMaker go to File -> Open -> in the browser select "SF" and click OK.
6. In CarMaker go to Application -> Configuration/Status -> In the field command, open src/CarMaker.linux64 and click on the Close button.
7. Press the start button and open IPG Movie,
8. In the Apollo Dreamview page, the car should be now localized in a San Francisco street and the traffic in front of the car should be detected by the LIDAR sensor.


# Known issues in this software
*Localization* 
- Jerky location within Apollo. This issue would be fixed by updating the IMU accelerometer values and making the transform from Long/Lat 
  coordinates to Eastings/Northings adaptative around the map.

*Lidar*
- Field of view is too narrow with the field of view used within the CarMaker TestRun

*Controls*
- *Update*: Apollo now communicates control commands (accelerator, brakes, steering). This, however, if it has worked in a previous version of the software which used a different architecture, we notice that Apollo often fails to provide a reference trajectory. We suspect that could be due to IMU data/GPS data inconsitency (see above). 

6 February 2020


(c) 2020 CavPoint
