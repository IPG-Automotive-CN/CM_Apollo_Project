/*
 * UTM_PositionData.h
 *
 *  Created on: 01.05.2020
 *      Author: zez and def
 */

#ifndef UTM_POSITIONDATA_H_
#define UTM_POSITIONDATA_H_

#include <road.h>
#include <Environment.h>
#include <string.h>
#include <Log.h>


typedef struct GPS_PosData
{
    double latitude_deg;
    double longitude_deg;
    double elevation;
}tGPS_PosData;


typedef struct UTM_PosData {
    char zone_info_area;		// UTM Zone Info (1 ~ 60) - Area (10); San Fransisco: 10
    double east;			// Nav. UTM east in [m]
    double north;			// Nav. UTM north in [m]
} tUTM_PosData;


#ifdef __cplusplus
extern "C" {
#endif

/* Initialization: Read in coordinates (GCS/CM local coordinate system) of road origin point */
void UTM_FromLocal_Conv_Init(void);

/* Functions to calculate UTM coordinates from local coordinates in CarMaker */
void UTM_FromLocal2GPS_Conv(tGPS_PosData *GPScoords, double Car_PosFr0_x, double Car_PosFr0_y, double Car_PosFr0_z);

void UTM_FromGPS2UTM_Conv(tUTM_PosData *UTMcoords, tGPS_PosData *GPScoords);


#ifdef __cplusplus
}
#endif
#endif /* UTM_POSITIONDATA_H_ */
