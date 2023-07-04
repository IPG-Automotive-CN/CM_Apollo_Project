/*
 * Apollo_CarMaker_IF.c
 *
 *  Created on: 09.04.2020
 *      Author: def
 */


#include "Apollo_CarMaker_IF.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdio.h>

#include <Car/Vehicle_Car.h>
#include <VehicleControl.h>
#include "SimCore.h"
#include "InfoUtils.h"
#include "DataDict.h"
#include "Log.h"
#include "DirectVarAccess.h"

#include <Vehicle/Sensor_FSpace.h>
#include <Vehicle/Sensor_LidarRSI.h>
#include <Vehicle/Sensor_Inertial.h>
#include <Vehicle/Sensor_Radar.h>


#include "bridge_tcp_client_c_interface.h"
#include "conversion_C_interface.h"
#include "cyberbridge_C_interface.h"
#include "rsds-client-camera.h"


/*Modification for Localization*/
#if defined (WITH_CMLOCAL)
	#include "Vehicle.h"
	#include "MathUtils.h"
	#include "UTM_PositionData.h"
#endif

#define INFO(stringMessage) printf("%s:%i:%s:%s\n", __FILE__, __LINE__, __func__, (stringMessage))

#define PI 3.14159265359

extern int gasCount;
/*
 * Parameters for Cloud Points
 * The same as vehicle info , use to calc elevation， used for lidar
*/
#define LIDAR_TILT_ANGLE 14.0
#define NUM_CLOUD_POINTS 32768

/* Integrate the c variables as cm UAQs */
static struct {
	double timeCMT;                          /* Sys. WC(Wall Clock) time in User_VehicleControl_Calc */
	double timelidarsend;                    /* Sys. WC time used in send_lidar msgs to Apollo */

	double throttle;                         /* Apollo gas signal in percent */
	double steering;                         /* Apollo steering signal in percent */
	double brake;                            /* Apollo brake signal in percent */
	double lateralErr;
	double headErr;                          /* Apollo ego car heading angel error between actual and expected */
	unsigned int seqNum;                     /* The seqNum of ctrl msgs from apollo */
	double timeApolloCtrl;                   /* The time stamp in apollo ctrl msgs header */
	double timelidar;                        /* The time stamp of lidar topic used for ctr */
	double timetotal;                        /* The used time for generating ctrl msgs in apollo*/
	double timeexcee;

	double timelastCtrl;                     /* Sys. WC time in cm at receiving last ctrl msgs */

	double diff_time;                        /* The difference of the time stamp in Apollo's ctrl msgs and cm sys. WC */
	double diff_lasttime;                    /* The difference of timelastCtrl and timeCMT */
} DelQuant;


int sensor_type = 1; //0:FSpace_Sensor 1:LidarRSI_Sensor 2:Radar_Sensor 3:lidarRSI&&Radar 4:cameraRSI

tFSSplus_SensorUnit *FSSplus_SensUnit;
static float x_array[NUM_CLOUD_POINTS];
static float y_array[NUM_CLOUD_POINTS];
static float z_array[NUM_CLOUD_POINTS];


#if defined (WITH_CMSIMTIME)
/* The sys. time stamp in cm at simulation start */
static struct timespec simstart_WCtime;
static u_int64_t simstart_WCtime_ns;
double get_cm_sim_WCtime (void);


/* function to the sim time with a start time in sys. WC */
double get_cm_sim_WCtime () {
	double tmp;
	tmp = simstart_WCtime_ns + SimCore.Time*1000*1000*1000;   //Support SimCore.Time with nano sec.?
    return tmp;
}
#endif


#if 0
/* struct using for build pid controller to modify the sim spd*/
typedef struct tPIDCtrl {
    int    State;     /* state of PID controller */
    double Kp;        /* proportional gain */
    double Ki;        /* integral gain */
    double Kd;        /* derivative gain */

    /* Variables internally used for Calculation
     * - Do NOT manipulate! Only for Debugging!
     */
    struct {
    double in;    /* input value of signal to be controlled */
    double out;   /* output value of signal used to control the input signal */
    double trgt;  /* target value of signal to be controlled */
    double delta; /* difference between target and input. delta = trgt - in" */
    double sum;   /* integral of delta */
    double deriv; /* derivative of delta */
    double dt;    /* time stepsize */
    } Calc;
} tPIDCtrl;


int SimSpdCtrl_Init(tPIDCtrl *SimSpdCtrl, double Kp, double Ki, double Kd);
int SimSpdCtrl_Calc(tPIDCtrl *SimSpdCtrl, double In, double Trgt, double dt);
int SimSpdCtrl_DelQuant (void );


static double input;
tPIDCtrl SimSpdCtrl;


int SimSpdCtrl_Init(tPIDCtrl *SimSpdCtrl, double Kp, double Ki, double Kd) {

	memset (SimSpdCtrl, 0, sizeof(tPIDCtrl));

	SimSpdCtrl-> Kp = Kp;
	SimSpdCtrl-> Ki = Ki;
	SimSpdCtrl-> Kd = Kd;
	SimSpdCtrl-> State = 1;

	return 0;
}


int SimSpdCtrl_DelQuant ( ){

	tDDefault *df = DDefaultCreate("SimSpdCtrl.");


    DDefDouble4 (df, "Kd",            "-",     &SimSpdCtrl.Kd,         DVA_DM);
    DDefDouble4 (df, "Ki",            "-",     &SimSpdCtrl.Ki,         DVA_DM);
    DDefDouble4 (df, "Kp",            "-",     &SimSpdCtrl.Kp,         DVA_DM);
    DDefInt     (df, "State",         "-",     &SimSpdCtrl.State,      DVA_DM);

    DDefDouble4 (df, "Output",        "-",     &SimSpdCtrl.Calc.out,   DVA_None);
    DDefDouble4 (df, "Input",         "-",     &SimSpdCtrl.Calc.in,   DVA_None);

	return 0;
}



int SimSpdCtrl_Calc(tPIDCtrl *SimSpdCtrl, double In, double Trgt, double dt) {

	double delta     = Trgt - In;

	SimSpdCtrl->Calc.deriv  = (delta - SimSpdCtrl->Calc.delta)/dt;
	SimSpdCtrl->Calc.sum   += delta*dt;
	SimSpdCtrl->Calc.delta  = delta; /* must be after calculation of derivative! */
	SimSpdCtrl->Calc.in     = In;
	SimSpdCtrl->Calc.trgt   = Trgt;
	SimSpdCtrl->Calc.dt     = dt;

	SimSpdCtrl->Calc.out    =
			SimSpdCtrl->Kp*SimSpdCtrl->Calc.delta + SimSpdCtrl->Ki*SimSpdCtrl->Calc.sum + SimSpdCtrl->Kd*SimSpdCtrl->Calc.deriv;

	return 0;
}
#endif

double last_liadarRSI_calcu_time = 0;
int lidar_index = -1;

int LidarRSI_Initialization(void){
	char* lidar_name = iGetStrOpt(SimCore.Vhcl.Inf, "LidarName",NULL);
	lidar_index = LidarRSI_FindIndexForName(lidar_name);
	printf("lidar_name = %s  lidar_index = %d\n", lidar_name, lidar_index);
	return 0;
}


int radar_index = -1;
double last_radar_sensor_calcu_time = 0;

int RadarSensor_Initialization(void){
	 if(sensor_type < 0)
        return -1;
	char* radar_name = "RA00";
	radar_index = RadarSensor_FindIndexForName(radar_name);
    printf("radar_index = %d radar count = %d\n", radar_index, RadarCount);
	return 0;
}

/* Init FSpace sensor with help of VehicleInfoFile parameters or set static values */
int
FSSplus_Initialization(void)
{
    char pre[64];
    char buf[64];

    int ret = 0;

    FSSplus_SensUnit = malloc(sizeof(tFSSplus_SensorUnit));

    /* Prefix */
    sprintf(pre, "Sensor.1");

	/* Read in name of FSSplus sensor from VehicleInfoFile */
    sprintf(buf, "%s.name", pre);
	sprintf(FSSplus_SensUnit->sensor_name, "%s", iGetStr(SimCore.Vhcl.Inf, buf));
	Log("Name: %s\n", FSSplus_SensUnit->sensor_name);


	/* Get the FSSplus index from FSpaceSensor name */
	FSSplus_SensUnit->sensor_index = FSpaceSensor_FindIndexForName (FSSplus_SensUnit->sensor_name);

	/* If there is a FSpaceSensor. But this information will show when the simulation starts */
	if ((FSSplus_SensUnit->sensor_index >= 0) && (FSSplus_SensUnit->sensor_index <= FSpaceSensorCount)) {

		/* Allocate memory for the FSSplus_User. If there is only one FSpaceSensor, the variable tFSpaceSensor could be used.
		 * If there are more than one FSpaceSensor, we can save all the data by using this method */
		FSSplus_SensUnit->CM_FSSplus = malloc (sizeof(tFSpaceSensor));
		FSSplus_SensUnit->CM_FSSplus = &FSpaceSensor[FSSplus_SensUnit->sensor_index];
	}

	/* Read in CycleTime of LidarRSI sensor */
	sprintf(buf, "%s.UpdRate", pre);
	double cyleTime_Hz = iGetDbl(SimCore.Vhcl.Inf, buf); // Hz
	FSSplus_SensUnit->sensor_cycleTime = (1.0/cyleTime_Hz) * 1000;
	Log("SensorCycleTime: %d ms\n", FSSplus_SensUnit->sensor_cycleTime);

    return ret;
}


/* Save cloud as a CSV file only once after starting the simulation */
static inline void save_cloud(size_t lidar_points_number)
{
    // Adding points to CSV file
    static char buffer_file_name[32] = {};
    static u_int64_t previous_save_time_sec = 0;//ms
    struct timespec current_time;
    timespec_get(&current_time, TIME_UTC); // get time in UTC
    u_int64_t current_time_ms = (current_time.tv_sec) * 1000 +
                                (current_time.tv_nsec) / (1000000);

    if (current_time_ms >= previous_save_time_sec + 1)
    {
        //printf("\n Saving CSV file \n");
        snprintf(buffer_file_name, 32, "./lidar_%i.csv", (int)current_time_ms);
        FILE *file = fopen(buffer_file_name, "w");
        fprintf(file, "x,y,z\n");

        size_t i;
        for (i = 0; i < lidar_points_number; i++)
        {
            double x = x_array[i];
            double y = y_array[i];
            double z = z_array[i];
            fprintf(file, "%f,%f,%f\n", x, y, z);
        }

        fclose(file);
        previous_save_time_sec = current_time_ms;
    }
}

static inline int get_cm_time_stamp(){
#if defined (WITH_CMSIMTIME)
	double cm_time_stamp;
	cm_time_stamp = get_cm_sim_WCtime();
#else
	return -1;             // -1 means, no using cm SimTime related time stamp in sending msgs
#endif
}


static inline int update_radar_sensor(){
	if(radar_index >=0 &&radar_index<RadarCount){
		set_radar_sensor(&RadarSensor[radar_index], get_cm_time_stamp());
	}
}


static inline int update_lidarRSI_cloud(){
	static double lidar_timestamp = 0.0;
    size_t lidar_num_points = 0; // Current number of points to be sent to bridge
    size_t i;

	

    // Clear array (defensive: should be unecessary thanks to lidar_num_points)
    for (i = 0; i < NUM_CLOUD_POINTS; i++)
    {
        x_array[i] = 0.0f;
        y_array[i] = 0.0f;
        z_array[i] = 0.0f;
    }

	if(lidar_index>=0&&lidar_index<=LidarRSICount){
			tLidarRSI* lidarRSI = &LidarRSI[lidar_index];
			int nScanPoints = lidarRSI->nScanPoints;
			double ScanTime = lidarRSI->ScanTime;

			if(ScanTime-last_liadarRSI_calcu_time > 1){
				last_liadarRSI_calcu_time = ScanTime;
				tScanPoint* ScanPoint = lidarRSI->ScanPoint;
				//printf("ScanNumber = %d\tScanTime = %lf\tnScanPoints = %d\n", lidarRSI->ScanNumber, ScanTime, nScanPoints);
				for(int i = 0;i<nScanPoints;i++){
					tScanPoint point_data = *(ScanPoint+i);

					int row = point_data.BeamID % 120;
					int col = point_data.BeamID / 120;
					double h_deg = (-59.5+row*1)/180.0 *PI;
					double v_deg = (-7.75+col*0.5)/180.0 *PI;
					// printf("beamId = %d,  row = %d, col = %d\n", point_data.BeamID, row, col);
					// printf("h_deg = %lf   v_deg = %lf len = %f\n", h_deg, v_deg, point_data.LengthOF);
					x_array[i] = point_data.LengthOF*sin(PI/2-v_deg)*cos(h_deg);
					y_array[i] = point_data.LengthOF*sin(PI/2-v_deg)*sin(h_deg);
					z_array[i] = point_data.LengthOF*cos(PI/2-v_deg);

					
				}

				printf("update_lidarRSI_cloud nScanPoints = %d\n",nScanPoints);

				set_cloud_buffer(x_array, y_array, z_array, nScanPoints, ScanTime);

			}
		}else{
			Log("index not valid");
			//return -1;
		}

}

static inline size_t update_cloud()
{
    static double lidar_timestamp = 0.0;
    size_t lidar_num_points = 0; // Current number of points to be sent to bridge
    size_t i;

    // Clear array (defensive: should be unecessary thanks to lidar_num_points)
    for (i = 0; i < NUM_CLOUD_POINTS; i++)
    {
        x_array[i] = 0.0f;
        y_array[i] = 0.0f;
        z_array[i] = 0.0f;
    }

    if(FSSplus_SensUnit->CM_FSSplus == 0){
        INFO("ERROR: It looks like the LIDAR sensors has not been defined\n");
        return lidar_num_points;  			// zez: should be return a negtive value
    }
    /* Interesting part */
/*    printf("FSSplus Timestamp: %f, lidar_timestamp: %f\n",FSSplus_SensUnit->CM_FSSplus->TimeStamp, lidar_timestamp);
    printf("FSSplus TotSegm: %d\n",FSSplus_SensUnit->CM_FSSplus->nTotSegm);*/
    //TODO: Bugfix: release the if-condition, because at 2nd start, the static lidar_timestamp will not be reset, which lead to non sent of the Cloud Points
    //if ( FSSplus_SensUnit->CM_FSSplus->TimeStamp > lidar_timestamp)
    //{
        // Create floor at specific altitude
        //const float road_altitude =  -1.91; // -2.33; //

        lidar_timestamp = FSSplus_SensUnit->CM_FSSplus->TimeStamp;
        //printf("FSSplus Timestamp: %f\n",lidar_timestamp);
        size_t next_index = 0;
        size_t Nb_S_count = FSSplus_SensUnit->CM_FSSplus->nTotSegm;

        size_t Nb_S;

        for (Nb_S = 0; Nb_S < Nb_S_count; Nb_S++)
        {
            double ds_p = FSSplus_SensUnit->CM_FSSplus->Segm[Nb_S].ds_p;
            double azimuth = FSSplus_SensUnit->CM_FSSplus->Segm[Nb_S].alpha_p;
            double elevation = FSSplus_SensUnit->CM_FSSplus->Segm[Nb_S].theta_p;

            elevation = elevation - LIDAR_TILT_ANGLE * PI/180.0;
            double x = ds_p*sin(PI/2-elevation)*cos(azimuth);
            double y = ds_p*sin(PI/2-elevation)*sin(azimuth);
            double z = ds_p*cos(PI/2-elevation);

            // Only add points if they are not at zero
            if (fabs(x) > 1e-3 || fabs(y) > 1e-3 || fabs(z) > 1e-3)
            {
                // Only add points if they are not further away than 75 meters
                if(x < 75.0){
                    x_array[next_index] = x;
                    y_array[next_index] = y;
                    z_array[next_index] = z;
                    next_index++;
                }
            }
        }

        lidar_num_points = next_index; // Update number of lidar points
        if (lidar_num_points > 0) {
        	set_cloud_buffer(x_array, y_array, z_array, lidar_num_points, lidar_timestamp);
        } else {
        	 printf("update_cloud() no lidar_points is calculated (%zu)!!!\n",lidar_num_points);
        }
        // Adding points to CSV file
        //save_cloud();
    //}
    return lidar_num_points;
}

/* Update the ego car position, speed and chassis status */
static inline void update_position_speed()
{
	 /*Modification for Localization*/
#if defined (WITH_CMLOCAL)
	/* Coordinate transformation:
	 * Get x,y(Fr0) of ego car and convert to x_frloc, y_frloc
	 */
	double x_frloc, y_frloc, x_ref, y_ref;
	double rzyx_off_deg;
	//Rotation angle for the coordinate transformation per hand ,-45,3, 45 or -44,7, this is because of the correct imu.
	rzyx_off_deg = - 45;

	//Reference point for the GCS coordinate from the ROAD5 file
	x_ref = -293.33;
	y_ref = 0.17;
	//Calc the coordinate with the coordinate transformation, inertial sensor is mounted on the middle of rear axe
	x_frloc= (InertialSensor[0].Pos_0[0]-x_ref)*cos(rzyx_off_deg*deg2rad) + (InertialSensor[0].Pos_0[1]-y_ref)*sin(rzyx_off_deg*deg2rad);
	y_frloc= -(InertialSensor[0].Pos_0[0]-x_ref)*sin(rzyx_off_deg*deg2rad) + (InertialSensor[0].Pos_0[1]-y_ref)*cos(rzyx_off_deg*deg2rad);

	//with testrun, the offset can be set up for a exactly localization between CarMaker and Apollo
	double x_off, y_off;
	x_off =0;
    y_off =0;
	x_frloc = x_frloc + x_off;
	y_frloc = y_frloc + y_off;

	/*from frlocal to GCS*/
	tGPS_PosData GPScoords;
	UTM_FromLocal2GPS_Conv(&GPScoords, x_frloc, y_frloc, Vehicle.PoI_Pos[2]);
//	Log("Ref_Veh_Frloc: %f\t %f\t %f\n", GPScoords.latitude_deg, GPScoords.longitude_deg, GPScoords.elevation);

	double longitude_deg = GPScoords.longitude_deg;
    double latitude_deg = GPScoords.latitude_deg;

	//printf("WITH_CMLOCAL\n store latitude = %f longitude = %f", latitude_deg, longitude_deg);

    set_gps_coord(longitude_deg, latitude_deg);

    float heading_deg = Car.Fr1.r_zyx[2] * 180 / PI - rzyx_off_deg;
    set_vehicle_heading(heading_deg);
#endif
    struct CarMakerChassis carmaker_chassis;
    carmaker_chassis.vehicle_speed_mps = Vehicle.v; // Already in m/s
    // TODO: IPG: use the Vehicle.Engine_rotv instead
//    carmaker_chassis.vehicle_engine_rpm = PowerTrain.IF.Engine_rotv * 60 / (2*PI);
    carmaker_chassis.vehicle_engine_rpm = Vehicle.Engine_rotv * 60 / (2*PI);   // From rad/s to RPM
    carmaker_chassis.vehicle_throttle_sensor_pct = PowerTrain.ControlIF.Gas * 100.0f;
    //printf("%s:%i:%s vehicle_throttle_sensor_pct %f\n", __FILE__, __LINE__, __func__, carmaker_chassis.vehicle_throttle_sensor_pct);
    carmaker_chassis.vehicle_brake_sensor_pct = Brake.IF.Pedal * 100;

    //7.33 rad came from 420 degree, which is max. Steering for Demo_Jaguar, for apollo need to convert from rad into percent
    if (Vehicle.Steering.Ang > (7.33)) {
    	carmaker_chassis.vehicle_steering_sensor_pct = 100.0;
    } else if (Vehicle.Steering.Ang < (-7.33)) {
    	carmaker_chassis.vehicle_steering_sensor_pct = -100.0;
    } else {
    	// Steering 40% for apollo is PI.
        carmaker_chassis.vehicle_steering_sensor_pct = 100.0 * Vehicle.Steering.Ang/(7.33);
    }

    //TODO: ipg: use the InertialSensor, because apollo need the Fr_Vehicle (RFU) und Fr_Local (ENU)
    carmaker_chassis.global_linear_velocity = InertialSensor[0].Vel_B;
    carmaker_chassis.global_linear_acceleration = InertialSensor[0].Acc_B;
    carmaker_chassis.vehicle_angular_velocity = InertialSensor[0].Omega_B;

    set_vehicle_chassis_values(&carmaker_chassis);
}



/* Integration functions for User.c */

void
Apollo_User_Init()
{
	INFO("\nPreparing CarMaker bridge\n");
	fflush(stdout);

//	SimSpdCtrl_Init (&SimSpdCtrl, 0.03, 0.0, 0.0);

	INFO("\n[User_Init] Testing some code ........ \n");




}

/* import the c variables to UAQs */
int
Apollo_User_DeclQuants()
{

	tDDefault *df = DDefaultCreate("Apollo.");

	DDefDouble (df, "timeCMT",     "", &DelQuant.timeCMT, DVA_None);
	DDefDouble (df, "lidarSend",     "", &DelQuant.timelidarsend, DVA_None);

	DDefDouble (df, "Brake",       "", &DelQuant.brake, DVA_None);
	DDefDouble (df, "Throttle",    "", &DelQuant.throttle, DVA_None);
	DDefDouble (df, "Steering",    "", &DelQuant.steering, DVA_None);

	DDefDouble (df, "timeApollo",  "", &DelQuant.timeApolloCtrl, DVA_None);
	DDefDouble (df, "timelidarApollo",  "", &DelQuant.timelidar , DVA_None);


	DDefDouble (df, "DiffTime",    "", &DelQuant.diff_time, DVA_None);

	DDefDouble (df, "timeCMLast",  "", &DelQuant.timelastCtrl, DVA_None);
	DDefDouble (df, "DiffLasttime",    "", &DelQuant.diff_lasttime, DVA_None);

	DDefDouble (df, "timelatenTotal",  "", &DelQuant.timetotal, DVA_None);
	DDefDouble (df, "timelatenExcee",    "", &DelQuant.timeexcee, DVA_None);


	DDefDouble (df, "LateralErr",  "", &DelQuant.lateralErr, DVA_None);
	DDefDouble (df, "HeadErr",    "", &DelQuant.headErr, DVA_None);

	DDefUInt (df, "CtrlSeqNum",    "", &DelQuant.seqNum, DVA_None);
	DDefUInt (df, "GasCount",    "", &gasCount, DVA_None);
//	SimSpdCtrl_DelQuant ();

	return 0;
}


int
Apollo_TestRun_Start_atEnd()
{
	if(SimCore.State <= SCState_Start) {
		goto ErrorHandling;
	}

	sensor_type = iGetIntOpt(SimCore.Vhcl.Inf, "SensorType",NULL);

	printf("read parameter from Vehicle infofile, key = %s, value = %d\n", "SensorType", sensor_type);
	/* Initialization of FSpaceSensor needed */
	int ret = 0;
	if(sensor_type == 0){
		ret = FSSplus_Initialization();
	}
	if(sensor_type == 1 || sensor_type == 3){
		ret = LidarRSI_Initialization();
	} 
	if(sensor_type == 2 || sensor_type == 3){
		ret =  RadarSensor_Initialization();
	}
	/* Initialization: Read in coordinates (GCS/CM local coordinate system) of road origin point */
#if defined (WITH_CMLOCAL)
	UTM_FromLocal_Conv_Init();
#endif

	INFO("\n[Apollo_TestRun_Start_atEnd] Initialization ........ \n");

	if(sensor_type == 4){
		RSDS_Init();
		RSDS_Start();
	}

	return ret;

	ErrorHandling:
	return -1;
}


void
Apollo_User_In	()
{
	static u_int64_t previous_time_85ms;
	static u_int64_t previous_time_10ms;

#if defined (WITH_CMSIMTIME)
	static int record_simstart_WCtime;
#endif

	if (SimCore.State == SCState_Start)
	{
		INFO("\n[SimCore.State == SCState_Start] starting CarMaker bridge subscriber and publisher");
		fflush(stdout);
		destroy_bridge();
		construct_bridge();

#if defined (WITH_CMSIMTIME)
		record_simstart_WCtime = 1;
#endif
		if(!is_bridge_connected()){
			INFO("[SimCore.State == SCState_Start] : Bridge is not connected\n");
			return; // Exit function if Bridge is not connected !
		}

	}
	else if (SimCore.State == SCState_Simulate)
	{

		// Only send data according to a certain frequency
		struct timespec current_time;
		timespec_get(&current_time, TIME_UTC);
		u_int64_t current_time_ms = (current_time.tv_sec) * 1000 +
									(current_time.tv_nsec) / (1000000);
		// printf("SimCore.State == SCState_Simulate : Running\n");
		// fflush(stdout);
		if(!is_bridge_connected()){
			// Print every now and then if the bridge is not connected
			static u_int64_t previous_time_ms = 0;
			if(current_time_ms > previous_time_ms + 2000){
				previous_time_ms = current_time_ms;
				//INFO("[SimCore.State == SCState_Simulate] : Bridge is not connected attempt to connect\n");
				fflush(stdout);
				destroy_bridge();
				construct_bridge();
			}
			return; // Exit function if Bridge is not connected !
		}

#if defined (WITH_CMSIMTIME)
		double cm_time_stamp;
		if (record_simstart_WCtime ==1) {
			timespec_get(&simstart_WCtime, TIME_UTC);
			simstart_WCtime_ns = (simstart_WCtime.tv_sec) * 1000*1000*1000 + simstart_WCtime.tv_nsec;
			record_simstart_WCtime = 0;
		}
#endif

		/*
		 * Send the msgs to apollo
		 * use low frame rate to save the performance of cpu
         * The imu, correct imu, control are sent with ~100Hz FrameRate to keep the ego car driving well
         * the cycle time 10ms should be used according the /apollo-3.5/modules/control/conf/adapter.conf and control_conf.pb.txt
         */
		if(current_time_ms > 10 + previous_time_10ms){
			previous_time_10ms = current_time_ms;
			update_position_speed();

#if defined (WITH_CMSIMTIME)
			cm_time_stamp = get_cm_sim_WCtime();
#else
			double cm_time_stamp = -1;              // -1 means, no using cm SimTime related time stamp in sending msgs
#endif
			send_high_framerate(cm_time_stamp);
		}

		if(current_time_ms > 0 + previous_time_85ms){
			previous_time_85ms = current_time_ms;
			update_position_speed();
#if defined (WITH_CMSIMTIME)
			cm_time_stamp = get_cm_sim_WCtime();
#else
			double cm_time_stamp = -1;              // -1 means, no using cm SimTime related time stamp in sending msgs
#endif
			send_low_framerate(cm_time_stamp);
		}

	}
	else if (SimCore.State == SCState_End)
	{
		INFO("[SimCore.State == SCState_End] End of simulation called\n\n");
		fflush(stdout);
		destroy_bridge();

		INFO("[SimCore.State == SCState_End] Simulation Ended \n\n");
	}
}


int
Apollo_User_VehicleControl_Calc()
{
    int ret = 0;

	static int    flag_ctrl_start;    // flag to check if the updated ctrl comes
    static double throttle=0;
	static double brake=0.7;          // set the inti. brake on 0.7 to keep the Auto stopping at beginning
	static double steering=0;
	static double previous_lastCtrltime;      // To check if the updated ctrl msgs comes


	// Only send data according to a certain frequency
	struct timespec current_time;
	timespec_get(&current_time, TIME_UTC);
	u_int64_t current_time_ms = (current_time.tv_sec) * 1000 +
								(current_time.tv_nsec) / (1000000);


    if (SimCore.State == SCState_StartLastCycle) {
    	//ipg: reset the value to initial
    	throttle=0;
    	brake=0.7;
    	steering=0;
    	flag_ctrl_start = 0;
    	printf("[VehicleControl]:reset the control commands and flags at beginning!\n");

    	current_time_ms = 0;   //before simulation start, set the timer to 0
    }


    if (SimCore.State == SCState_Simulate) {



    	if(!is_bridge_connected()){
    		// Print every now and then if the bridge is not connected
    		static u_int64_t previous_time_ms = 0;
    		if(current_time_ms > previous_time_ms + 2000){
    			previous_time_ms = current_time_ms;
    			//INFO("[SimCore.State == SCState_Simulate] : Bridge is not connected attempt to connect\n");
    			fflush(stdout);
    			destroy_bridge();
    			construct_bridge();
    		}
    		return -1; // Exit function if Bridge is not connected !
    	}


    	// check the seqNum of ctrl msgs. The seqNum ctrl msgs for every simulation should start from 0
    	int ApolloCtrl_seqNum;
    	ApolloCtrl_seqNum = get_apollo_seqNum_value();
    	if (ApolloCtrl_seqNum < 5) {
    		flag_ctrl_start = 1;
//    		INFO("Recevie the apollo ctrl msgs with seqNum smaller than 5, start using the ctrl msgs \n");
    	}

		flag_ctrl_start = 1;
		//if(0){
    	if(flag_ctrl_start==1 && is_apollo_control_data_fresh()){
			
    	    DelQuant.timeCMT         = current_time_ms;
    	    DelQuant.lateralErr      = get_apollo_lateralErr_value();
    	    DelQuant.headErr         = get_apollo_headErr_value();
    	    DelQuant.timeApolloCtrl  = get_apollo_timespec_value()*1000; //ms
    	    DelQuant.timelidar  = get_apollo_lidartimespec_value()/1000000; //ms
    	    DelQuant.timetotal = get_apollo_timetotal_value(); //ms
    	    DelQuant.timeexcee = get_apollo_timeexcee_value(); //ms
      	    DelQuant.timelastCtrl    = get_lastCtrl_time()*1000;        //ms
    	    DelQuant.diff_time       = current_time_ms - DelQuant.timeApolloCtrl;   //ms
    	    DelQuant.diff_lasttime   = current_time_ms - DelQuant.timelastCtrl;     //ms
    	    DelQuant.seqNum = get_apollo_seqNum_value();

// The test with a pid controller for simCore.TAccel
//    	    if (DelQuant.timelastCtrl - previous_lastCtrltime < 0.1) {
//    	    input = DelQuant.timeCMT  - (simstart_WCtime_ns/(1000*1000) + SimCore.Time*1000); //ms
//    	    SimSpdCtrl_Calc(&SimSpdCtrl, input, DelQuant.diff_time, DeltaT);
//    	    SimCore.TAccel =   SimSpdCtrl.Calc.out ;
//    	    printf ("the SimSpd is %lf \n", SimCore.TAccel);
//    	    } else  {
//    	    	 SimCore.TAccel = 0.55 ;
//    	    }


    	    // 	SimCore.TAccel = 0.2;
    	    // } else {
    	    // 	SimCore.TAccel = 0.55 ;             //zez: with 2 cameras the simulation spd is max. around 0.6. (only from observation)
    	    // }
			SimCore.TAccel = 1;
    	    //update the previous last ctrl time stamp
    	    previous_lastCtrltime =  DelQuant.timelastCtrl;        //ms

    	    // set the ctrl msgs in cm
    		throttle = (get_apollo_throttle_value() / 100.0f);
			if (throttle > 1.0)
			{
				throttle = 1.0;
			}

			steering = get_apollo_steering_value()/100.0f;
			brake = (get_apollo_brake_value() / 100.0f);



			//the modification of sim spd, based on the last ctrl msgs in carmaker
    	    if (DelQuant.timelastCtrl - previous_lastCtrltime < 0.5) {
				DVA_WriteRequest("VC.Gas", OWMode_Abs, 500, 0, 0, throttle, NULL);
				DVA_WriteRequest("VC.Steer.Ang", OWMode_Abs, 500, 0, 0, (steering)*7.33, NULL);
			}else{
				VehicleControl.Gas = 0;
				VehicleControl.Steering.Ang = 0;
			}

			//printf("apollo throttle value = %f\n", get_apollo_throttle_value());
			//printf("carmaker Gas value = %f\n", throttle);

			//printf("apollo brake value = %f\n", get_apollo_brake_value());
			//printf("carmaker Brake value = %f\n", brake);

			//printf("apollo steering value = %f\n", get_apollo_steering_value());
			//printf("carmaker Steering.Ang value = %f\n", (steering)*7.33);
			//VehicleControl.Gas = throttle;
			VehicleControl.Brake = brake;
			//VehicleControl.Steering.Ang = (steering)*7.33;  //+/-420 degree max/min steering angel for demo_Jaguar
    	} else {


    		// stop the car if the usable ctrl msgs are not comming in cm
        	// throttle=0;
        	// brake=0.7;
        	// steering=0;

            // DelQuant.timeCMT         = current_time_ms;
            // DelQuant.timelidar      = current_time_ms;
			// DelQuant.timeApolloCtrl = current_time_ms;
			// DelQuant.timelastCtrl =  current_time_ms;

    		// VehicleControl.Gas = throttle;
			// VehicleControl.Brake = brake;
			// VehicleControl.Steering.Ang = steering*7.33;
	    	//if (SimCore.CycleNo % 4100 ==1) {
	    		//printf("[Not fresh]Targets  steering in percent:%f \t brake:%f \t throttle:%f \n", steering, brake, throttle);
	    	//}
    	}
    } else if (SimCore.State == SCState_End) {

    		INFO("[SimCore.State == SCState_End] End of simulation called\n\n");
    		fflush(stdout);
    		destroy_bridge();
    		INFO("[SimCore.State == SCState_End] Simulation Ended \n\n");

    }


    //Mapping the ctrl signal to CM UAQs
    DelQuant.brake           = brake;
    DelQuant.steering        = steering;
    DelQuant.throttle        = throttle;

    return ret;
}



int
Apollo_User_Out()
{
	int ret = 0;

	if(!is_bridge_connected()){
		ret = -1; // Exit function if Bridge is not connected !
		return ret;
	}

	if (SimCore.State == SCState_Start)
	{
		INFO("\n[SimCore.State == SCState_Start] starting CarMaker bridge subscriber and publisher");
		fflush(stdout);
	}
	else if (SimCore.State == SCState_Simulate)
	{

		//Free Space Sensor
		// Only send data according to a certain frequency
		if(sensor_type == 0  
		&& FSSplus_SensUnit->CM_FSSplus->TimeStamp > 0.05 /*TODO: add the condition FSS calc timestamp bigger than 0.1*/
		&& FSSplus_SensUnit->sensor_cycleTime >0 
		&& (SimCore.CycleNo)%FSSplus_SensUnit->sensor_cycleTime == 0 /* SensorCycleOffset */)
		{
			size_t lidar_points_number = update_cloud();
			if(lidar_points_number > 0){

#if defined (WITH_CMSIMTIME)
			double cm_time_stamp;
			cm_time_stamp = get_cm_sim_WCtime();
#else
		double cm_time_stamp = -1;              // -1 means, no using cm SimTime related time stamp in sending msgs
#endif
			send_cloud(cm_time_stamp);

			DelQuant.timelidarsend = get_lidarsend_time();
/*				printf("[User_Out, Simulation State]"
				         "CarMaker cloud has (%i) points at CycleNo %d!\n\n",
				         (int) lidar_points_number, SimCore.CycleNo);*/
			} else {
				printf("[User_Out, SimCore.State == SCState_Simulate]"
						"Cloud is empty (%i)!\n\n",
						(int) lidar_points_number);
			}
		}

		//printf("lidar_index = %d\n", lidar_index);

		//LidarRSI Sensor
		if((sensor_type == 1||sensor_type ==3) && (SimCore.CycleNo)%60 == 0){
			update_lidarRSI_cloud();
#if defined (WITH_CMSIMTIME)
			double cm_time_stamp;
			cm_time_stamp = get_cm_sim_WCtime();
#else
			double cm_time_stamp = -1;              // -1 means, no using cm SimTime related time stamp in sending msgs
#endif
			send_cloud(cm_time_stamp);

		}
		//Radar Sensor
		if((sensor_type == 2 || sensor_type == 3) && (SimCore.CycleNo)%70 == 0){
			

			update_radar_sensor();
			send_radar_sensor(get_cm_time_stamp());
		}
		//CameraRSI Sensor
		if(sensor_type == 4 && (SimCore.CycleNo)%33 == 0){
			send_image(get_cm_time_stamp());
		}
	}
	return ret;
}

void
Apollo_User_ShutDown ()
{

	fflush(stdout);
	/* Prepare application for shutdown and return that
	   shutdown conditions are reached */
}


void Apollo_Cleanup()
{

	INFO(" Stopping the Cyberbridge TCP client");
	    tcp_client_stop();
	free(FSSplus_SensUnit);
	if(sensor_type == 4){
		RSDS_Exit();
	}
}

