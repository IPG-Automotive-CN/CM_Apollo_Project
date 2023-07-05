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

#ifndef RSI_CLIENT_RSDS_CLIENT_H
#define RSI_CLIENT_RSDS_CLIENT_H


#include "rsds-definition.h"

// ######### RSDS configuration #########
static struct {
    char *MovieHost;  /* pc on which IPGMovie runs          */
    int   MoviePort;  /* TCP/IP port for RSDS               */
    int   sock;       /* TCP/IP Socket                      */
    char  hbuf[64];   /* Buffer for transmitted information */
    int   RecvFlags;  /* Receive Flags                      */
    int   Verbose;    /* Logging Output                     */
    int   ConnectionTries;
} RSDScfg;

struct {
    int         nFrames;
    int         nBytes;
} RSDSIF;


// ######### Radar RSI #########
typedef struct tRadarRSI_RSDS {
    int         nDetections;	// Number of detections
    tOutputPointC_R  *DetPoints;	// Detection if Output is Point Cloud
    tOutputVRx_R   **DetVRx;		// Detection pointer for VRx mode - read data here
    tOutputVRx_R    *DetVRxData;
} tRadarRSI_RSDS;

// RadarRSI output types
typedef enum {
    RADAR_OUTPUT_CART 		= 0,
    RADAR_OUTPUT_SPHERICAL	= 1,
    RADAR_OUTPUT_VRX		= 2
} OutputTypeRadar;


// ######## Functions #########

// Core functions for connection and messages
static int RSDS_RecvHdr(int sock, char *hdr);
static int RSDS_Connect(void);
static int RSDS_GetData(void);
void RSDS_Init(void);

// Sensor specific processing functions
void processLidarData(const char *dataP);
void processUltraSonicData(const char *dataP);
int processRadarData(
	const char *dataP,
	float SimTime);

// Radar RSI specific functions
static tOutputVRx_R* GetVRxPointer(int DetectionId, tRadarRSI_RSDS* radarRSI, int nVRx);
static int GetMsgBlockSize(int nDet, int nVRx, int OutputType, size_t *PacketSize);
static int GetDataSize(int nDet, int nVRx, int OutputType, size_t *PacketSize);



#endif //RSI_CLIENT_RSDS_CLIENT_H
