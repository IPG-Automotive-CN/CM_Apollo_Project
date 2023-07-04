# Usage Instructions
1. Install Apollo 3.5 (LG version) and build and run the Apollo containers, run the Apollo bridge and open the Apollo Dreamview page
2. Download and install the last version of Protobuf https://github.com/protocolbuffers/protobuf/releases
3. Run the script `generate_protobufs.sh`. This should generate in the src folder : CarMaker.linux64
4. Open CarMaker and make this folder the CarMaker Project folder.
  -> In CarMaker go to File -> Project folder -> Select the folder containing this project
5. In CarMaker go to File -> Open -> in the browser select "SF_OpenDRIVE_3TrfObj_v1" and click OK.
6. In CarMaker go to Application -> Configuration/Status -> In the field command, open src/CarMaker.linux64 and click on the Close button.
7. Press the start button and open IPG Movie,
8. In the Apollo Dreamview page, the car should be now localized in a San Francisco street.


29 May 2020


(c) 2020 IPG-Automotive
