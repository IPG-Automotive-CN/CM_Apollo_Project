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
**
** Raw Signal Data Stream example for IPGMovie 3.4 and later versions.
**
** Compiling:
** Linux
**	gcc -Wall -Os -o rsds-client rsds-client.c
** MS Windows (MSYS/MinGW)
**	gcc -Wall -Os -o rsds-client.exe rsds-client.c -lws2_32
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#ifdef WIN32
#  include <winsock2.h>
#else
#  include <unistd.h>
#  include <sys/socket.h>
#  include <sys/types.h>
#  include <net/if.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <netdb.h>
#endif

#include "rsds-client.h"


#define FreeIfNotNULL(a) if ((a)!=NULL) do {free (a); (a)=NULL;} while(0)



tRadarRSI_RSDS *RadarRSI;
int nSensMsg_Radar;
int nSens_Radar_Last = 0;

static float SimTime_Radar_Last = 0.0;

/*** function to make return data pointer of VRx detection
 *
 * @param DetectionId detection id
 * @param radarRSI pointer to private radar
 * @param nVRx number of virtual receivers
 * @return
 */
static tOutputVRx_R*
GetVRxPointer(int DetectionId, tRadarRSI_RSDS* radarRSI, int nVRx){

    size_t DataPntSize = sizeof(tComplex_R) * (size_t) nVRx + sizeof(tOutputVRx_R);

    return (tOutputVRx_R*) ((char*) radarRSI->DetVRxData
		       + (size_t) ((size_t) DetectionId * DataPntSize));

}

/** returns MsgBlockSize for a single sensor (all data plus header of sensor msg)
 *
 * @param nDet [in] Detections
 * @param nVRx [in] Virutal Receivers
 * @param OutputType [in] cart / polar / VRx (0/1/2)
 * @param PacketSize [out] size of Msg Block (PacketSize)
 * @return
 */
static int GetMsgBlockSize(int nDet, int nVRx, int OutputType, size_t *PacketSize)
{
    if (PacketSize == NULL || OutputType < 0 || OutputType > 2 || nDet < 0) {
	return -1;
    }

    if (OutputType == 0 || OutputType == 1) {
	*PacketSize = sizeof(tRSIResMsg_Radar_PointC) + nDet * sizeof(tOutputPointC_R);
    } else if (OutputType == 2) {
	if(nVRx <= 0){
	    return -1;
	}
	*PacketSize = sizeof(tRSIResMsg_Radar_VRx) +
		      nDet * (sizeof(tOutputVRx_R) + nVRx * sizeof(tComplex_R));
    }

    return 0;
}

/** returns OutputSize for a sensor msg block (all data without header)
 *
 * @param nDet [in] Detections
 * @param nVRx [in] number of virtual Receivers
 * @param OutputType [in] cart / polar / VRx (0/1/2)
 * @param PacketSize [out] size of Output (PacketSize)
 * @return
 */
static int GetDataSize(int nDet, int nVRx, int OutputType, size_t *PacketSize) {

    if (PacketSize == NULL || OutputType < 0 || OutputType > 2 || nDet < 0) {
	return -1;
    }

    if (OutputType == 0 || OutputType == 1) {
	*PacketSize = nDet * sizeof(tOutputPointC_R);

    } else if (OutputType == 2) {
	if (nVRx <= 0){
	    return -1;
	}
	*PacketSize = nDet * (sizeof(tOutputVRx_R) + nVRx * sizeof(tComplex_R));
    }

    return 0;
}


/*
** RSDS_RecvHdr
**
** Scan TCP/IP Socket and writes to buffer
*/
static int
RSDS_RecvHdr (int sock, char *hdr)
{
    const int HdrSize = 64;
    int len = 0;
    int nSkipped = 0;
    int i;

    while (1) {
        for ( ; len < HdrSize; len += i) {
	    if ((i=recv(sock, hdr+len, HdrSize-len, RSDScfg.RecvFlags)) <= 0)
		return -1;
	}
	if (hdr[0]=='*' && hdr[1]>='A' && hdr[1]<='Z') {
	    /* remove white spaces at end of line */
	    while (len>0 && hdr[len-1]<=' ')
		len--;
	    hdr[len] = 0;
	    if (RSDScfg.Verbose && nSkipped>0)
	        printf ("RSDS: HDR resync, %d bytes skipped\n", nSkipped);
	    return 0;
	}
	for (i=1; i<len && hdr[i]!='*'; i++) ;
	len -= i;
	nSkipped += i;
	memmove (hdr, hdr+i, len);
    }
}



/*
** RSDS_Connect
**
** Connect over TCP/IP socket
*/
static int
RSDS_Connect (void)
{
#ifdef WIN32
    WSADATA WSAdata;
    if (WSAStartup(MAKEWORD(2,0), &WSAdata) != 0) {
        fprintf (stderr, "RSDS: WSAStartup ((2,0),0) => %d\n", WSAGetLastError());
        return -1;
    }
#endif

    struct sockaddr_in	DestAddr;
    struct hostent  	*he;

    int tries = RSDScfg.ConnectionTries;

    if ((he=gethostbyname(RSDScfg.MovieHost)) == NULL) {
        fprintf (stderr, "RSDS: unknown host: %s\n", RSDScfg.MovieHost);
        return -2;
    }
    DestAddr.sin_family      = AF_INET;
    DestAddr.sin_port        = htons((unsigned short)RSDScfg.MoviePort);
    DestAddr.sin_addr.s_addr = *(unsigned *)he->h_addr;
    RSDScfg.sock = socket(AF_INET, SOCK_STREAM, 0);

    printf("Hostip : %s \nHostport : %d \nSocket : %d \n", RSDScfg.MovieHost, RSDScfg.MoviePort, RSDScfg.sock);

    fflush(stdout);

    while (connect(RSDScfg.sock, (struct sockaddr *) &DestAddr, sizeof(DestAddr)) < 0 && tries > 0){
	fprintf(stderr, "RSDS: can't connect '%s:%d'\n", RSDScfg.MovieHost, RSDScfg.MoviePort);
	if (tries > 1) {
            fprintf(stderr, "\tretrying in 1 second... (%d)\n", --tries);
            fflush(stderr);
            sleep(1);
        } else {
            return -4;
        }
    }

    if (RSDS_RecvHdr(RSDScfg.sock, RSDScfg.hbuf) < 0)
        return -3;

    printf ("RSDS: Connected: %s\n", RSDScfg.hbuf+1);

    return 0;
}


/*
** RSDS_GetData
**
** data and image processing
*/
static int
RSDS_GetData(void)
{
    int len = 0;
    int res = 0;

    /* Variables for Image Processing */
    char    SensorType[64];
    float   SimTime;
    int     DataLen;

    if (sscanf(RSDScfg.hbuf, "*%s %f %d", SensorType, &SimTime, &DataLen) == 3) {
        if (RSDScfg.Verbose)
            printf ("#%6d -- %8s -> t = %6.3f -- s = %d\n",RSDSIF.nFrames, SensorType, SimTime, DataLen);
        if (DataLen > 0) {
            char *data = (char *)malloc(DataLen);
            for (len=0; len<DataLen; len+=res) {
                if ((res=recv(RSDScfg.sock, data+len, DataLen-len, RSDScfg.RecvFlags)) < 0) {
                    printf("RSDS: Socket Reading Failure\n");
                    break;
                }
            }

            if (len == DataLen) {

                // process your data here
		
		if (strcmp(SensorType, "LidarRSI") == 0) {
		    processLidarData(data);
		}
		
		if (strcmp(SensorType, "USonicRSI") == 0) {
		    processUltraSonicData(data);
		}
		
		if (strcmp(SensorType, "RadarRSI") == 0) {
		    if (processRadarData(data, SimTime) < 0)
		        return -1;
		}

            }
            FreeIfNotNULL(data);
        }
        RSDSIF.nFrames++;
        RSDSIF.nBytes += len;
    } else {
        printf ("RSDS: not handled: %s\n",RSDScfg.hbuf);
    }

    return 0;
}


void processLidarData(const char *dataP)
{
    int i_sens, i_det;

    tRSIResMsg *ResMsg = (tRSIResMsg*) dataP;
    // ATTENTION: nSensors only corresponds to the sensors running on the GPUSensor
    // which is sending the data refer to SensorID in tRSIResMsg_Header_Lidar
    // for the global sensor ID 
    int nSensors = ResMsg->Header.nSensors;

    tScanPoint_L *ScanPoints[nSensors];
    int nScanPoints[nSensors];

    tRSIResMsg_Lidar *DataBlockLidar = (tRSIResMsg_Lidar*) (ResMsg->Data);

    for (i_sens = 0; i_sens < nSensors; i_sens++) {

        // get data of this DataBlock
	nScanPoints[i_sens] = DataBlockLidar->Header.nScanPoints;
	ScanPoints[i_sens] = DataBlockLidar->SP;

	// read ScanPoints
	if (0) {
	    for (i_det = 0; i_det < nScanPoints[i_sens]; i_det++) {
		printf("           BeamID %d\n", ScanPoints[i_sens][i_det].BeamID);
	    }
	}

	if (RSDScfg.Verbose) {
	    printf("       Sensor[%d] - nSP %d, ID %d\n",
		   i_sens, nScanPoints[i_sens], DataBlockLidar->Header.SensorID);
	}
	
	// jump to next memory address after current data block to get correct new DataBlock position
	DataBlockLidar =(tRSIResMsg_Lidar*) &(DataBlockLidar->SP[DataBlockLidar->Header.nScanPoints]);

    }
}

void processUltraSonicData(const char *dataP)
{
    int i_TxRx,i_det;

    tRSIResMsg *ResMsg = (tRSIResMsg*) dataP;
    // This corresponds to the Transmitter-Receiver-Pairs
    int nTxRxPair = ResMsg->Header.nSensors;

    // save the data pointers
    tDetPoint_U *Detections[nTxRxPair];
    int Rx_Id[nTxRxPair];
    int Tx_Id[nTxRxPair];
    int nDetections[nTxRxPair];

    tRSIResMsg_USonic* DataPackUSonic = (tRSIResMsg_USonic*) ResMsg->Data;
    for (i_TxRx = 0; i_TxRx < nTxRxPair; i_TxRx++) {

        // read out data
	nDetections[i_TxRx] = DataPackUSonic->DataHeader.nDetections;
	Detections[i_TxRx] = DataPackUSonic->Detections;
	Rx_Id[i_TxRx] = DataPackUSonic->DataHeader.i_Rx;
	Tx_Id[i_TxRx] = DataPackUSonic->DataHeader.i_Tx;

	if (RSDScfg.Verbose) {
	    printf("       Sensor[%d] - nEchoes %d, Transmitter %d Receiver %d\n",
		   i_TxRx, nDetections[i_TxRx], Tx_Id[i_TxRx], Rx_Id[i_TxRx]);
	}

	// read single detection
	if (0) {
	    for (i_det = 0; i_det < nDetections[i_TxRx]; i_det++) {
		printf("           LengthOF %f\n",Detections[i_TxRx][i_det].range);
	    }
	}

	// jump to next memory address after current data block to get correct new DataBlock position
	DataPackUSonic = (tRSIResMsg_USonic*) &(Detections[i_TxRx][nDetections[i_TxRx]]);
    }
}


static int
Init_RadarRSI_RSDS(
	tRadarRSI_RSDS *RadarRSI,
	OutputTypeRadar OutputType,
	int nMaxDet,
	int nVRx)
{

    if (OutputType == RADAR_OUTPUT_CART || OutputType == RADAR_OUTPUT_SPHERICAL) {
    	// allocate memory for detections
	RadarRSI->DetPoints = calloc(nMaxDet, sizeof(tOutputPointC_R));
	if (RadarRSI->DetPoints == NULL) {
	    printf("RadarRSI: detection memory allocation failed\n");
	    return -1;
	}
    } else if (OutputType == RADAR_OUTPUT_VRX) {
	size_t OutputSize = nMaxDet * (sizeof(tOutputVRx_R) + sizeof(tComplex_R) * nVRx);
	// allocate memory for detections data
	RadarRSI->DetVRxData = (tOutputVRx_R *) malloc(OutputSize);
	if (RadarRSI->DetVRxData == NULL) {
	    printf("RadarRSI: detection memory allocation failed.\n");
	}

	// allocate memory for detections
	RadarRSI->DetVRx = (tOutputVRx_R **) calloc(nMaxDet, sizeof(tOutputVRx_R *));
	if (RadarRSI->DetVRx == NULL) {
	    printf("RadarRSI: detection memory allocation failed\n");
	}

	// initialize the detection pointers for the detection data
	int i_det = 0;
	for (i_det = 0; i_det < nMaxDet; ++i_det) {
	    RadarRSI->DetVRx[i_det] = GetVRxPointer(i_det, RadarRSI, nVRx);
	}
    }

    return 0;

}

static int
Realloc_Radar(int nSensors)
{
    int i_sens = 0;
    for (i_sens = 0; i_sens < nSens_Radar_Last; i_sens++)
    {
	FreeIfNotNULL(RadarRSI[i_sens].DetPoints);
	FreeIfNotNULL(RadarRSI[i_sens].DetVRx);
	FreeIfNotNULL(RadarRSI[i_sens].DetVRxData);
    }
    FreeIfNotNULL(RadarRSI);
    RadarRSI = calloc(nSensors, sizeof(tRadarRSI_RSDS));
    nSensMsg_Radar = nSensors;
    nSens_Radar_Last = nSensors;
    if (RadarRSI == NULL)
    {
	printf("RadarRSI: RadarRSI memory allocation failed\n");
	return -1;
    }

    return 0;
}


int
processRadarData(
	const char *dataP,
	float SimTime)
{
    int i_sens, msg_shift = 0;
    int nSensMsg_Radar;
    // marks if a reallocation of a single sensor is needed
    int FlagRealloc = 0;

    tRSIResMsg *ResMsg = (tRSIResMsg*) dataP;
    nSensMsg_Radar = ResMsg->Header.nSensors;

    // set up Data Storage
    int nDetSensor[nSensMsg_Radar];
    int nVRx[nSensMsg_Radar];
    int SensorID[nSensMsg_Radar];

    // the realloc is necessary in case you restart the simulation with a different
    // sensor setting, but leave the client running
    if (SimTime < SimTime_Radar_Last || SimTime_Radar_Last == 0.0) {
	Realloc_Radar(nSensMsg_Radar);
	FlagRealloc = 1;
    }
    SimTime_Radar_Last = SimTime;

    // ATTENTION: nSensors only corresponds to the sensors running on the GPUSensor
    for (i_sens = 0; i_sens < nSensMsg_Radar; i_sens++) {

        // get to next block of message, containing information of data
	tRSIResMsg_Radar *SensorBlock = (tRSIResMsg_Radar*) (ResMsg->Data + msg_shift);
	// Sanity check
	if (SensorBlock->Header.nMaxDet <= 0) {
	    printf("RadarRSI: result message corrupted - nMaxDet\n");
	    return -1;
	}

	// read out SensorID
	SensorID[i_sens] = SensorBlock->Header.SensorID;
	int OutputType = SensorBlock->Header.OutputType;

	// initialize specific sensor memory if reallocation is needed
	if (FlagRealloc) {
	    if(Init_RadarRSI_RSDS(&(RadarRSI[i_sens]),OutputType,SensorBlock->Header.nMaxDet,SensorBlock->Header.nVRx))
	    {
		printf("RadarRSI: Reallocation error\n");
		return -1;
	    }
	}

	// get type of sensor and sanity checks
	if (OutputType == RADAR_OUTPUT_CART || OutputType == RADAR_OUTPUT_SPHERICAL) {
	    nDetSensor[i_sens] = SensorBlock->Header.nDetPointC;
	    nVRx[i_sens] = 0;
	    // sanity checks
	    if (SensorBlock->Header.nVRx != -1 ||
	    	SensorBlock->Header.nDetPointC < 0 ) {
		printf("RadarRSI: result message corrupted\n");
		return -1;
	    }
	    // reset memory
	    memset(RadarRSI[i_sens].DetPoints, 0, SensorBlock->Header.nMaxDet * sizeof(tOutputPointC_R));

	} else if (OutputType == RADAR_OUTPUT_VRX) {
	    nDetSensor[i_sens] = SensorBlock->Header.nDetVRx;
	    nVRx[i_sens] = SensorBlock->Header.nVRx;
	    // sanity checks
	    if (SensorBlock->Header.nDetPointC != -1 ||
		SensorBlock->Header.nDetVRx < 0 ) {
		printf("RadarRSI: Result msg corrupted - nDetections\n");
		return -1;
	    }
	    // reset memory
	    size_t OutputSize = SensorBlock->Header.nMaxDet * (sizeof(tOutputVRx_R) + sizeof(tComplex_R) * nVRx[i_sens]);
	    memset(RadarRSI[i_sens].DetVRxData, 0, OutputSize);

	} else {
	    printf("RadarRSI: output transfer failed, unknown output type\n");
	    return -1;
	}

	// get size of memory block of detections
	size_t MemSizeData; //size of memory to copy
	size_t MemSizeSkip; //size of memory to skip (includes header)
	if (GetMsgBlockSize(nDetSensor[i_sens], nVRx[i_sens], OutputType, &MemSizeSkip) < 0) {
	    printf("RadarRSI: result msg corrupted - MsgBlockSize\n");
	    return -1;
	}
	if (GetDataSize(nDetSensor[i_sens], nVRx[i_sens], OutputType, &MemSizeData) < 0) {
	    printf("RadarRSI: result msg corrupted - DataBlockSize\n");
	    return -1;
	}

	// get type of sensor and copy memory
	if (OutputType == RADAR_OUTPUT_CART || OutputType == RADAR_OUTPUT_SPHERICAL) {
	    memcpy(RadarRSI[i_sens].DetPoints, SensorBlock->Data, MemSizeData);
	} else if (OutputType == RADAR_OUTPUT_VRX) {
	    memcpy(RadarRSI[i_sens].DetVRxData, SensorBlock->Data, MemSizeData);
	}

	// add MemSize to advance through ResMsg
	msg_shift += (int) MemSizeSkip;

	if (RSDScfg.Verbose==1) {
	    printf("       Sensor[%d] - nSP %d, ID %d\n",
		   i_sens,
		   nDetSensor[i_sens],
		   SensorID[i_sens]);
	}

    }

    FlagRealloc = 0;

    return 0;
}


/*
** RSDS_Init
**
** Initialize Data Struct
*/
void
RSDS_Init(void)
{
    RSDScfg.MovieHost = "localhost";
    RSDScfg.MoviePort = 2210;
    RSDScfg.Verbose   = 0;
    RSDScfg.RecvFlags = 0;
    RSDScfg.ConnectionTries = 5;

    RSDSIF.nFrames  = 0;
    RSDSIF.nBytes   = 0;

    nSensMsg_Radar = 0;
}


int
main (int argc, char *const*argv)
{
    int i;
    char *appname = argv[0];

    RSDS_Init();

    while (*++argv != NULL) {
	if (strcmp(*argv,"-v") == 0) {
	    RSDScfg.Verbose++;
	} else if (strcmp(*argv,"-p")==0 && argv[1]) {
	    RSDScfg.MoviePort = atoi(*++argv);
	} else if (**argv!='-') {
	    RSDScfg.MovieHost = *argv;
	} else if (strcmp(*argv, "-t") == 0 && argv[1]) {
            RSDScfg.ConnectionTries = atoi(*++argv);
        } else if (strcmp(*argv, "-h") == 0) {
	    printf("%s - IPG Automotive GmbH", appname);
            printf(" %s ?<option>? ?<host>?\n", appname);
            printf(" * options *\n");
            printf("    -t <n>  tries n times to connect to IPGMovie (default = 5)\n");
            printf("    -v		fully verbose output\n");
            printf("    -p <port>	set RSDS port to listen to (default = 2210)\n");
	    printf(" * host *\n");
            printf("    default = localhost\n");
	    fflush(stdout);
            goto end;
	}
    }

    /* Connect to RSDS Server */
    if ((i = RSDS_Connect ()) != 0) {
	fprintf(stderr, "Can't initialise RSDS client (returns %d, %s)\n",
		i, i == -4 ? "No server": strerror(errno));
    }

    /* Read from TCP/IP-Port and process the image */
    while (RSDS_RecvHdr(RSDScfg.sock,RSDScfg.hbuf) == 0) {
	RSDS_GetData();
    }

    // closing and deleting radar structs if necessary
    int iS;
    printf("RSDS Client: deleting Radar structs\n");
    for (iS = 0; iS < nSensMsg_Radar; iS++) {
	FreeIfNotNULL(RadarRSI[iS].DetPoints);
	FreeIfNotNULL(RadarRSI[iS].DetVRx);
	FreeIfNotNULL(RadarRSI[iS].DetVRxData);
    }
    FreeIfNotNULL(RadarRSI);

end:
    printf("Closing RSDS-Client...\n");
    printf("TOTAL: %d Frames received, for %d bytes (%dMB)\n", RSDSIF.nFrames, RSDSIF.nBytes, RSDSIF.nBytes/1024/1024);
#ifdef WIN32
    closesocket (RSDScfg.sock);
#else
    close (RSDScfg.sock);
#endif

    return 0;
}
