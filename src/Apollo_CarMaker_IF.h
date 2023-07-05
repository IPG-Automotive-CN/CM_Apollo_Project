/*
 * Apollo_CarMaker_IF.h
 *
 *  Created on: 09.04.2020
 *      Author: def
 */

#ifndef APOLLO_CARMAKER_IF_H_
#define APOLLO_CARMAKER_IF_H_

#include <Vehicle/Sensor_FSpace.h>

typedef struct FSSplus_SensorUnit {
	char sensor_name[100];
	int sensor_index;
	tFSpaceSensor *CM_FSSplus;
	int sensor_cycleTime;
}tFSSplus_SensorUnit;


/* Integration functions for User.c */
void Apollo_User_Init			(void);
int  Apollo_TestRun_Start_atEnd	(void);
void Apollo_User_In				(void);
int  Apollo_User_VehicleControl_Calc	(void);
int  Apollo_User_Out				(void);
void Apollo_User_ShutDown		(void);
void Apollo_Cleanup				(void);

/* Used for ploting in IPGContol to check the result online*/
int  Apollo_User_DeclQuants		(void);


int FSSplus_Initialization(void);
int LidarRSI_Initialization(void);
int RadarSensor_Initialization(void);



#endif /* APOLLO_CARMAKER_IF_H_ */
