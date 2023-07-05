
#ifndef CONVERSION_H

struct NorthEast
{
    double x;
    double y;
};

struct GPScoord
{
    double latitude;
    double longitude;
};


void set_ins_stat(std::unique_ptr<apollo::drivers::gnss::InsStat> & insStat, double time_stamp);

void set_cloud_proto_msg(std::unique_ptr<apollo::drivers::PointCloud> & lidar, double time_stamp);
void set_radar_front_msg(std::unique_ptr<apollo::drivers::ContiRadar> & contiRadar, double time_stamp);
bool set_image_proto_msg(std::unique_ptr<apollo::drivers::Image> & image, double time_stamp);

void set_position_proto_msg(std::unique_ptr<apollo::localization::Gps> &gps, double time_stamp);
void set_position_proto_msg(std::unique_ptr<apollo::drivers::gnss::GnssBestPose> &bestPose, double time_stamp);
void set_position_proto_msg(std::unique_ptr<apollo::canbus::Chassis> &chassis);
void set_position_proto_msg(std::unique_ptr<apollo::drivers::gnss::Imu> &imu, double time_stamp);
void set_position_proto_msg(std::unique_ptr<apollo::localization::CorrectedImu> &imu, double time_stamp);


void set_chassis_powertrain_proto_msg(std::unique_ptr<apollo::canbus::Chassis> &chassis, double time_stamp);


void define_double_buffer_ptr(DoubleBuffer * cloud);
void delete_double_buffer_ptr(void);


#endif /* CONVERSION_H */
