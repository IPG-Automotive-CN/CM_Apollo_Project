/*
 * UTM_PositionData.cpp
 *
 *  Created on: 01.05.2020
 *      Author: zez and def
 */

#include <stdio.h>
#include <iostream>
#include <exception>
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "GeographicLib/UTMUPS.hpp"
#include "GeographicLib/MGRS.hpp"
#include "UTM_PositionData.h"

using namespace GeographicLib;
using namespace std;

LocalCartesian local_;

void UTM_FromLocal_Conv_Init()
{
    /* Reference origin coordinates (GCS/Fr0) */
    double Ref_Origin_Fr0[3];
    double Ref_Origin_GCS[3];

    /* Read in set reference coordinates from user. Error if no reference data is provided. */
    tGCSMode loc_TestGround = GCS_FlatEarth;
    if(RoadGetGCS(Env.Road, &loc_TestGround, Ref_Origin_Fr0, Ref_Origin_GCS) != 0){
	LogErrF(EC_Init, "No GCS-Coordinates reference entered! Please enter a reference value for lat, long and height.");
    } else {
	Log("Ref_Origin_GCS: %f\t %f\t %f\n", Ref_Origin_GCS[0], Ref_Origin_GCS[1], Ref_Origin_GCS[2]);
	Log("Ref_Origin_Fr0: %f\t %f\t %f\n\n", Ref_Origin_Fr0[0], Ref_Origin_Fr0[1], Ref_Origin_Fr0[2]);
    }

    /* Reset the local coordinate system to provided WGS84 origin */
    local_.Reset(Ref_Origin_GCS[0], Ref_Origin_GCS[1], Ref_Origin_GCS[2]);


    /* Initialization of the LocalCartesian system */
    local_ = GeographicLib::LocalCartesian(Ref_Origin_GCS[0], Ref_Origin_GCS[1], Ref_Origin_GCS[2], GeographicLib::Geocentric::WGS84());

}

void UTM_FromLocal2GPS_Conv(tGPS_PosData *GPScoords, double Car_PosFr0_x, double Car_PosFr0_y, double Car_PosFr0_z)
{
    double temp_geographic[3];

    /* Use of GeographicLib conversion functionality to calculate geographic coordinates of vehicle position from local coordinates */
    local_.Reverse(Car_PosFr0_x, Car_PosFr0_y, Car_PosFr0_z, temp_geographic[0], temp_geographic[1], temp_geographic[2]);

    GPScoords->latitude_deg     = temp_geographic[0] ;
    GPScoords->longitude_deg     = temp_geographic[1];
    GPScoords->elevation 	= temp_geographic[2];
}

void UTM_FromGPS2UTM_Conv(tUTM_PosData *UTMcoords, tGPS_PosData *GPScoords)
{
    double temp_geographic[3];
    double temp_UTM[2];
    int zone;
    bool northp;
    double gamma;
    double k;

    temp_geographic[0] = GPScoords->latitude_deg ;
    temp_geographic[1] = GPScoords->longitude_deg ;

    /* Conversion functionality to calculate UTM coordinates from geographic coordinates of vehicle position */
    UTMUPS::Forward(temp_geographic[0], temp_geographic[1], zone, northp, temp_UTM[0], temp_UTM[1], gamma, k, UTMUPS::STANDARD, false);

    UTMcoords->zone_info_area = (char)10; //zone
    UTMcoords->east     = temp_UTM[0] ;
    UTMcoords->north     = temp_UTM[1];
}




