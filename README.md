

# Overview
This project creates a communication bridge between CarMaker and Apollo 7.0 (CarMaker variant).
After compiling this code, you can connect to the resulting executable from CarMaker to connect to the Apollo "bridge".
Please follow the instructions below to use it. 

The source code for this has been written by CavPoint to a demonstrator level. It has not been written to a level for 
production code which would have required a higher level of rigour. It has been provided part-way through the 
development process and is therefore in no way in a state that could be considered 'complete' or 'ready to release'. 
It is provided as-is with no warranties, express or implied.


# Usage Instructions
1. Install Apollo 7.0 (CarMaker version) and build and run the Apollo containers, run the Apollo bridge and open the Apollo Dreamview page https://github.com/IPG-Automotive-CN/Apollo7.0-CarMaker
2. Download and install the version 3.11.4 of Protobuf https://github.com/protocolbuffers/protobuf/releases
3. Run the script `generate_protobufs.sh`. This should generate in the src folder : CarMaker.linux64
4. Open CarMaker and make the carmaker-bridge repository the CarMaker Project folder.
  -> In CarMaker go to File -> Project folder -> Select the folder containing this project
5. In CarMaker go to File -> Open -> in the browser select a testrun and click OK.
6. In CarMaker go to Application -> Configuration/Status -> In the field command, open src/CarMaker.linux64 and click on the Close button.
7. Press the start button and open IPG Movie,
8. In the Apollo Dreamview page, the car should be now localized in a San Francisco street and the traffic in front of the car should be detected by the LIDAR sensor.

9. Running CarMaker on a different host from apollo is recommanded if you are using lidarRSI or CameraRSI, because it is not stable to run Apollo and CarMaker physical sensor model at the same time with limited GPU resources. You can set the server ip by editing the CarMaker infofile, which named "SimParameter" and located in "./Data/Config", usage:  "Apollo_Server_Ip = <ip address>"



4 July 2023

