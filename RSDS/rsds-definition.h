/*
******************************************************************************
**  CarMaker - Version 11.0
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
*/

#ifndef _RSDS_DEFINITION_H_
#define _RSDS_DEFINITION_H_

// -----------------------------------------------------------------------------
// -------- result messages ----------------------------------------------------
// -----------------------------------------------------------------------------

// ######## all RSIs ###########################################################
typedef enum {
    MSG_OUTDATA_FULL          = 0,
    MSG_OUTDATA_SYNC          = 1
} tRSIResMsgType;

typedef struct tRSIResMsg_Header {
    short               ID;		// message ID
    short               padding;	// unused
    tRSIResMsgType      ResMsgType;	// for RSIs (tRSDS_ClientType)
    int                 MsgSize;
    int                 MsgBodySize;
    float               TimeStamp;
    int                 nSensors;
} tRSIResMsg_Header;

typedef struct tRSIResMsg {
    tRSIResMsg_Header	Header;
    char		Data[];
} tRSIResMsg;


// ########### Radar RSI ######################################################
typedef struct tOutputPointC_R tOutputPointC_ADT;
typedef struct tOutputVRx_R tOutputVRx_ADT;

// Output with coordinates
typedef struct tOutputPointC_R {
    union tCoord {
	struct tCoordCartOut {
	    float x;
	    float y;
	    float z;
	} CoordCartOut_R;
	struct tCoordSphOut {
	    float r;
	    float phi;
	    float theta;
	} CoordSphOut_R;
    } tCoord_R;
    float PowerdB;
    float vel;
} tOutputPointC_R;

// Output with complex amplitudes
typedef struct tComplex_R {
    float real;
    float imag;
}tComplex_R;

typedef struct tOutputVRx_R {
    float range;
    float vel;
    tComplex_R AmpLinRx[0];
} tOutputVRx_R;

typedef struct tMsgRadarOutputInfo {
    int SensorID;	// global sensor ID
    int OutputType;	// 0 cart, 1 spherical, 2 VRx
    int nDetPointC;
    int nDetVRx;
    int nVRx;
    int nMaxDet;
} tRSIResMsg_Radar_Header;

typedef struct tRSIResMsg_Radar_PointC {
    tRSIResMsg_Radar_Header Header;
    tOutputPointC_R Data[];
} tRSIResMsg_Radar_PointC;

typedef struct tRSIResMsg_Radar_VRx {
    tRSIResMsg_Radar_Header Header;
    tOutputVRx_R Data[];
} tRSIResMsg_Radar_VRx;

typedef struct tRSIResMsg_Radar {
    tRSIResMsg_Radar_Header Header;
    void *Data[];
} tRSIResMsg_Radar;


// ########### Lidar RSI ######################################################
typedef struct tScanPoint_L {
    int		BeamID;		// Beam ID
    int		EchoID;		// Echo ID (per Beam)
    float	TimeOF;		// Time of flight in nanoseconds
    float	LengthOF;	// Path length in m
    float	Origin[3];	// Ray origin (x,y,z) in FrSensor in rad
    float	Intensity;	// Intensity of reflected light in nano Watt
    float	PulseWidth;	// Echoe Pulse Width in nanoseconds
    int		nRefl;		// Number of reflections
} tScanPoint_L;

typedef struct tRSIResMsg_Header_Lidar {
    int	SensorID;
    int	nScanPoints;
} tRSIResMsg_Header_Lidar;

typedef struct tRSIResMsg_Lidar{
    tRSIResMsg_Header_Lidar Header;
    tScanPoint_L SP[];			// definition in Sensor_LidarRSI.h
} tRSIResMsg_Lidar;

// ########### USonic RSI #####################################################

typedef struct tComplex_U {
    float real;
    float imag;
}tComplex_U;

typedef struct tDetPoint_U {
    float range;
    tComplex_U SPA_rec;
}tDetPoint_U;

typedef struct tRSIResMsg_USonic_Header {
    int i_Tx;
    int i_Rx;
    int nDetections;
} tRSIResMsg_USonic_Header;

typedef struct tRSIResMsg_USonic{
    tRSIResMsg_USonic_Header DataHeader;
    tDetPoint_U Detections[];
} tRSIResMsg_USonic;

#endif //_RSDS_DEFINITION_H_
