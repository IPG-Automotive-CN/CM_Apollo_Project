
#ifndef CONVERSION_C_INTERFACE_H

#define PI 3.14159265359

#include "Vehicle/Sensor_Radar.h"

struct CarMakerChassis
{
    float vehicle_speed_mps;
    float vehicle_engine_rpm;
    float vehicle_throttle_sensor_pct;
    float vehicle_brake_sensor_pct;
    float vehicle_steering_sensor_pct;
    double * global_linear_velocity; // Pointer to array xyz of global velocities in m/s
    double * vehicle_angular_velocity; // Pointer to array xyz of global velocities in rad/s
    double * global_linear_acceleration; // Pointer to array xyz of global velocities in rad/s
};


double get_lidarsend_time(void);

// C wrapper for setting cloud in double buffer
void set_cloud_buffer(float * x_array, float * y_array, float * z_array,
                     int num_points, double timestamp);

void set_gps_coord(double longitude, double latitude);

void set_vehicle_heading(float vehicle_heading_simulator);

void set_vehicle_chassis_values(struct CarMakerChassis * carmaker_chassis);

void set_radar_sensor(tRadarSensor* radar_sensor, double time_stamp);

#endif /* CONVERSION_H */
