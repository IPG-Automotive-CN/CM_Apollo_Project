#include <string>
#include <atomic>
#include <cmath>
#include <memory>
#include <array>
#include <iomanip>

#include "modules/common/proto/error_code.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"

/* Added for sensor image */
#include "modules/drivers/proto/sensor_image.pb.h"
//#include "modules/drivers/camera/proto/config.pb.h"

/* Added for velodyne Lidar sensor */
#include "modules/drivers/velodyne/proto/velodyne.pb.h"
#include "modules/drivers/velodyne/proto/config.pb.h"

#include "cloud_doublebuffer.h"
#include "conversion.h"

#include <google/protobuf/util/json_util.h>

extern "C" {
#include "conversion_C_interface.h"
#include "rsds-client-camera.h"
#include "stdio.h"
#include "stdlib.h"
}


/* Modification for Localization */
#if defined (WITH_CMLOCAL)
        #include "UTM_PositionData.h"
#endif

static std::atomic<float> veh_heading_deg_atomic;
static std::atomic<float> veh_speed_mps;
static std::atomic<float> veh_engine_rpm;
static std::atomic<float> veh_throttle_sensor_pct;
static std::atomic<float> veh_brake_sensor_pct;
static std::atomic<float> veh_steering_sensor_pct;
static std::atomic<float> veh_global_velocity_x;
static std::atomic<float> veh_global_velocity_y;
static std::atomic<float> veh_global_accelation_x;
static std::atomic<float> veh_global_accelation_y;
static std::atomic<float> veh_angular_velocity_x;
static std::atomic<float> veh_angular_velocity_y;
static std::atomic<float> veh_angular_velocity_z;
static std::atomic<GPScoord> gps_coord;


double get_system_time (double time_stamp) {
    auto time_now = std::chrono::system_clock::now();
    double t_epoch_WC = std::chrono::duration_cast<std::chrono::nanoseconds>(
        time_now.time_since_epoch()).count()*1e-9; // seconds

//    double diff;
//    double t_epoch = time_stamp*1e-9; // seconds
//    diff = t_epoch_WC- t_epoch;
//    printf("the lantency at sending is %lf \n", diff);
    if (time_stamp < 0 ){
    	return t_epoch_WC;
    } else {
    	time_stamp = time_stamp*1e-9; //sec
    	return time_stamp;

    }
}

#if 0
typedef std::vector <std::vector<double>> Matrix;

static std::vector<double> MatrixMultiplyVector(Matrix M, std::vector<double> vect)
{
    std::vector<double> result (3);

    for (size_t row = 0; row < M.size(); row++)
    {
        auto & line = M[row];
        result[row] = 0.0f;
        for (size_t col = 0; col <line.size() ; col++)
        {
            result[row] = result[row] + (line[col] * vect[col]);
            // std::cout<< std::setprecision(16) << "col" << ":" << col << " line[col]:" << line[col]  << ", vect[col]" << vect[col] <<"\n" ; 
        }
        // std::cout<< std::setprecision(16) << "result[" << row << "] = " << result[row] <<"\n" ; 
    }
    return result;
}

static std::vector<double> HomographyTransform(std::vector<double> src){
    const Matrix HomographyMatrix  {
        { 1.1279351492461691e+00, -1.2492478953272310e-01, -3.7398432021948458e+00},
        { 2.6993678386570169e-01,  7.5398037217027758e-01, -1.9973255670231652e-01},
        { 3.1074660131301434e-04, -2.9539707579612006e-04,  1.0000000000000000e+00}
    };
    auto destTemp = MatrixMultiplyVector(HomographyMatrix,src);
    
    std::vector<double> dest (2) ;
    dest[0] = destTemp[0]/destTemp[2];
    dest[1] = destTemp[1]/destTemp[2];
    return dest;
}
#endif


/* Because the reworking on the UTMConversion, this function is no longer used. */
#if 0
static NorthEast convertLongLat2NE(GPScoord &coord)
{
    NorthEast ne;
    ne.y = (coord.latitude-37.78711092)/(37.78890189-37.78711092)*(4182563.94-4182363.94)+4182363.94;
    ne.x = (coord.longitude+122.4009191)/(-122.3986332+122.4009191)*(552949.63-552749.63)+552749.63;
    static std::vector<double> src;
    src = {ne.y, ne.x};
    src.push_back(1.0f);
    
    const std::array<double, 2> offset = {4182000, 552000};
    src[0] -=  offset[0];
    src[1] -=  offset[1];
    std::vector<double> dest = HomographyTransform(src);
    dest[0] +=  offset[0];
    dest[1] +=  offset[1];
    
    ne.x = dest[1];
    ne.y = dest[0];
    // std::cout<< std::setprecision(16) << "x:" << ne.x << " y:" << ne.y << "\n";
    return ne;
}
#endif



static DoubleBuffer * cloud_buffer_ptr = nullptr;
//std::unique_ptr<std::vector<CloudPoint>> global_cloud_buffer;

// C wrapper for setting cloud in double buffer
void set_cloud_buffer(float *x_array, float *y_array, float *z_array,
                         int num_points, double timestamp)
{
    if (cloud_buffer_ptr != nullptr)
    {
        cloud_buffer_ptr->set(x_array, y_array, z_array, num_points, timestamp);
    }
    else
    {
        std::cout << "ERROR : cloud_buffer_ptr is not defined, you must use define_double_buffer" << std::endl;
    }
}

void define_double_buffer_ptr(DoubleBuffer *cloud)
{
    cloud_buffer_ptr = cloud;
}

void delete_double_buffer_ptr(){
    cloud_buffer_ptr = nullptr;
}



static std::map<int, int> DynPropCM2Apollo = {
    {0, 4}, //unknow
    {1, 1}, //stationary
    {2, 7}, //stopped
    {3, 0}, //moving
    {4, 2}  //oncoming
};

static std::map<int, double> ProbExistCM2Apollo = {
    {0, 0},
    {1, 0.25},
    {2, 0.5},
    {3, 0.75},
    {4, 0.9},
    {5, 0.99},
    {6, 0.999},
    {7, 1}
};

static std::map<int, int> MeasStatCM2Apollo = {
    {0, 0}, //no object
    {1, 1}, //new object
    {2, 0}, //TODO object not measured
    {3, 2}  //object measured
};


static apollo::drivers::ContiRadar* conti_radar = new apollo::drivers::ContiRadar();

apollo::drivers::ContiRadar* get_radar_sensor_proto(){
    return conti_radar;
}


void set_vehicle_chassis_values(CarMakerChassis * carmaker_chassis){
    veh_speed_mps.store(carmaker_chassis->vehicle_speed_mps);
    veh_engine_rpm.store(carmaker_chassis->vehicle_engine_rpm);
    veh_throttle_sensor_pct.store(carmaker_chassis->vehicle_throttle_sensor_pct);
    veh_brake_sensor_pct.store(carmaker_chassis->vehicle_brake_sensor_pct);
    veh_steering_sensor_pct.store(carmaker_chassis->vehicle_steering_sensor_pct);

    //TODO: ipg: convert CM Fr1 (FLU) to Apollo FrVehicle (RFU)
    veh_global_velocity_x.store(carmaker_chassis->global_linear_velocity[0]);
    veh_global_velocity_y.store(carmaker_chassis->global_linear_velocity[1]);
    float acc[3], ang_vel[3];
    acc[0] = carmaker_chassis->global_linear_acceleration[1]*(-1);
    acc[1] = carmaker_chassis->global_linear_acceleration[0];
    acc[2] = carmaker_chassis->global_linear_acceleration[2];

    ang_vel[0] = carmaker_chassis->vehicle_angular_velocity[1];
    ang_vel[1] = carmaker_chassis->vehicle_angular_velocity[0];
    ang_vel[2] = carmaker_chassis->vehicle_angular_velocity[2];

    veh_global_accelation_x.store(acc[0]);
    veh_global_accelation_y.store(acc[1]);

    veh_angular_velocity_x.store(ang_vel[0]);
    veh_angular_velocity_y.store(ang_vel[1]);
    veh_angular_velocity_z.store(ang_vel[2]);
}

void set_vehicle_heading(float vehicle_heading_deg)
{
    // Adjust offset
    vehicle_heading_deg = std::fmod(vehicle_heading_deg, 360.0);

    // the under code should not be used, because the modification by IPG.
#ifdef WITH_CMLOCAL
    vehicle_heading_deg -= 90.0f;   //from cm fr0 coordinate to gps coordinate in apollo needs to turn 90 degree.

    // Limits -314.6942 deg -> 45.3058 deg
    if(vehicle_heading_deg > 45.3058){
        vehicle_heading_deg -= 360.0;
    }
    if(vehicle_heading_deg <= -314.6942)
    {
        vehicle_heading_deg += 360.0f;
    }
#endif

    veh_heading_deg_atomic.store(vehicle_heading_deg);
}

void set_gps_coord(double longitude, double latitude)
{
    GPScoord gps = {
        .latitude = latitude,
        .longitude = longitude
        };
    gps_coord.store(gps);
}


/////////////////////////////////////////////////////////////////
//record the time stamp
static std::atomic<double> timelidarsend;



double get_lidarsend_time(){
	double tmp;
	tmp = timelidarsend;
    return tmp;
}

void logProtoMsg(google::protobuf::Message& msg){
    std::string json_str;
    google::protobuf::util::MessageToJsonString(msg, &json_str);
    std::cout<<"obs json stamp = "<<json_str<<std::endl;
}

static int is_first = true;
//////////////////////////////////////////////////////////////
bool set_image_proto_msg(std::unique_ptr<apollo::drivers::Image> & image, double time_stamp){
    if(is_first){
        is_first = false;
        //write
        // FILE* file = fopen("test.txt", "w");
        // int d[] = {4, 3, 9, 9};
        // fwrite(d, sizeof(int), 4, file);
        // fclose(file);

        //read
        FILE* a=fopen("test.txt","r");
        int p[5];
        fread(p,sizeof(int),4,a);
        for(int i = 0; i<4;i++){
            printf("i = %d value = %d\n", i, p[i]);
        }
        fclose(a);
        
    }

    if(RSDSIF.imgLen == 0 || RSDSIF.imgHeight == 0 || RSDSIF.imgWidth == 0)
        return false;
    static int sequence_num = 0;
    image->Clear();
    double t_epoch = get_system_time(time_stamp);
    image->mutable_header()->set_timestamp_sec(t_epoch);
    image->mutable_header()->set_sequence_num(sequence_num);
    //image->mutable_header()->set_module_name()
    image->set_frame_id("front_6mm");
    image->set_measurement_time(t_epoch);
    image->set_height(RSDSIF.imgHeight);
    image->set_width(RSDSIF.imgWidth);
    image->set_encoding("rgb8");


    std::string image_str(RSDSIF.image, RSDSIF.imgLen);
    image->set_step(RSDSIF.imgLen/RSDSIF.imgHeight);


    std::cout<<"image_str size = "<<image_str.size()<<std::endl;

    image->set_data(image_str);

    sequence_num++;
    //logProtoMsg(*image);

    // FILE* file = fopen("test.txt", "w");
    // int d[] = {4, 3, 9, 9};
    // fwrite(d, sizeof(int), 4, file);
    // fclose(file);


    std::cout<<"send image measurement_time = "<<image->measurement_time()
    <<"encoding = "<<image->encoding()
    <<"seq = "<<image->mutable_header()->sequence_num()
    <<"image size = "<<image->data().size()
    <<std::endl;

    
   return true;
}

void set_radar_front_msg(std::unique_ptr<apollo::drivers::ContiRadar> & contiRadar, double time_stamp){
    //printf("set_radar_front_msg\n");
    contiRadar->Clear();
    contiRadar->CopyFrom(*conti_radar);
    double t_epoch = get_system_time (time_stamp);
    contiRadar->mutable_header()->set_timestamp_sec(t_epoch);

    int objs_num = contiRadar->mutable_contiobs()->size();
    for(int i = 0;i<objs_num;i++){
        contiRadar->mutable_contiobs()->Mutable(i)->mutable_header()->set_timestamp_sec(t_epoch);
    }

    printf("send time stamp = %lf\n", t_epoch);

    //std::string json_str;
    //google::protobuf::util::MessageToJsonString(*contiRadar, &json_str);
    //printf("time stamp = %lf\n", contiRadar->mutable_header()->timestamp_sec());
   // std::cout<<"obs json stamp = "<<json_str<<std::endl;

}

void set_radar_sensor(tRadarSensor* radar_sensor, double time_stamp){
    static int sequence_num = 0;
    double t_epoch = get_system_time (time_stamp);

    tOutQuantsGlob* globalInf = radar_sensor->GlobalInf;
    tOutQuants* outQuants = radar_sensor->ObjList;
    int detected_objs_num = globalInf->nObj;
    tRadObject* radObj_list = radar_sensor->RadObj;

    conti_radar->Clear();
    conti_radar->mutable_header()->set_timestamp_sec(t_epoch);
    conti_radar->mutable_header()->set_sequence_num(sequence_num);
    conti_radar->mutable_header()->set_module_name("conti_radar");
    conti_radar->mutable_header()->set_frame_id("radar_front");

    conti_radar->mutable_object_list_status()->set_nof_objects(detected_objs_num);
    conti_radar->mutable_object_list_status()->set_meas_counter(22800);
    conti_radar->mutable_object_list_status()->set_interface_version(0);

    for(int i = 0;i<detected_objs_num;i++){
        apollo::drivers::ContiRadarObs * obs = conti_radar->add_contiobs();
        obs->mutable_header()->set_timestamp_sec(t_epoch);
        obs->mutable_header()->set_sequence_num(sequence_num);
        obs->mutable_header()->set_module_name("conti_radar");
        obs->mutable_header()->set_frame_id("radar_front");
        obs->set_clusterortrack(0);   //0 = track, 1 = cluster
        obs->set_obstacle_id(outQuants->ObjId);
        obs->set_longitude_dist(outQuants->DistX);   // longitude distance to the radar; (+) = forward; unit = m
        obs->set_lateral_dist(outQuants->DistY);     // lateral distance to the radar; (+) = left; unit = m
        obs->set_longitude_vel(outQuants->VrelX);
        obs->set_lateral_vel(outQuants->VrelY);
        obs->set_rcs(outQuants->RCS);
        obs->set_dynprop( DynPropCM2Apollo[outQuants->DynProp]);
        obs->set_longitude_dist_rms(0);               // longitude distance standard deviation to the radar; (+) = forward; unit = m
        obs->set_lateral_dist_rms(0);               // lateral distance standard deviation to the radar; (+) = left; unit = m
        obs->set_longitude_vel_rms(0);
        obs->set_lateral_vel_rms(0);
        obs->set_probexist(1);//ProbExistCM2Apollo[outQuants->ProbExist]);
        obs->set_meas_state(MeasStatCM2Apollo[outQuants->MeasStat]);
        obs->set_longitude_accel(outQuants->ArelX);
        obs->set_lateral_accel(0);
        obs->set_oritation_angle(outQuants->RelCourseAngle * 180 / PI); //radian->degree
        obs->set_longitude_accel_rms(0);
        obs->set_lateral_accel_rms(0);
        obs->set_oritation_angle_rms(0);
        obs->set_length(outQuants->Length);
        obs->set_width(outQuants->Width);
        obs->set_obstacle_class(1); //TODO 0: point; 1: car; 2: truck; 3: pedestrian; 4: motorcycle; 5: bicycle; 6: wide; 7: unknown

        // std::string json_str;
        // google::protobuf::util::MessageToJsonString(*obs, &json_str);
        // std::cout<<"obs json str = "<<json_str<<std::endl;
        outQuants++;
    }    
    sequence_num++;
}


void set_cloud_proto_msg(std::unique_ptr<apollo::drivers::PointCloud> & lidar, double time_stamp ) {
    static const std::string frame_id = "lidar128";
    static const std::string header_frame_id = "velodyne";
    static int sequence_num = 0;

    double t_epoch = get_system_time (time_stamp);

    std::vector<CloudPoint> cloud = cloud_buffer_ptr->get_fresh_buffer();

    lidar->Clear();
    lidar->mutable_header()->set_frame_id(header_frame_id);
    lidar->mutable_header()->set_sequence_num(sequence_num);
    sequence_num ++ ;

    lidar->set_frame_id(frame_id);
    lidar->set_measurement_time(t_epoch);
    lidar->set_width(cloud.size());
    lidar->set_height(1);

    timelidarsend = t_epoch * 1000;  //ms

    for(CloudPoint p : cloud){
        apollo::drivers::PointXYZIT * point = lidar->add_point();
        point->set_x(p.x);
        point->set_y(p.y);
        point->set_z(p.z);
        point->set_intensity(p.intensity);
    }
}


void set_ins_stat(std::unique_ptr<apollo::drivers::gnss::InsStat> & insStat,double time_stamp){
    static bool isInitialized = false;
    static uint32_t sequence_number = 0U;

    //TODO: zez: the static seqNum will be not reset to 0, if we do not close the thread after first simulation.
//    printf ("send seqNum in insStat is %lu \n", sequence_number);

    double t_epoch = get_system_time (time_stamp);

    insStat->mutable_header()->set_timestamp_sec(t_epoch);
    insStat->mutable_header()->set_sequence_num(sequence_number);
    sequence_number++;

    if (!isInitialized)
    {
        isInitialized = true;
        insStat->set_ins_status(3);
        insStat->set_pos_type(56); //drivers::gnss::SolutionType::INS_RTKFIXED 
    }
}

void set_position_proto_msg(std::unique_ptr<apollo::localization::Gps> &gps, double time_stamp)
{
    static uint32_t sequence_number = 0U;

    double t_epoch = get_system_time (time_stamp);

    gps->mutable_header()->set_timestamp_sec(t_epoch);
    gps->mutable_header()->set_sequence_num(sequence_number);
    sequence_number++;

    // gps->mutable_localization()->mutable_linear_velocity()->set_x(veh_global_velocity_x.load(std::memory_order_relaxed));
    // gps->mutable_localization()->mutable_linear_velocity()->set_y(veh_global_velocity_y.load(std::memory_order_relaxed));
//    gps->mutable_localization()->mutable_linear_velocity()->set_z(0.0);
    
// The Modification in localization for a exactly UTMConversion
#if defined (WITH_CMLOCAL)

    GPScoord gps_value = gps_coord.load(std::memory_order_relaxed);


    tGPS_PosData GPScoords;

    GPScoords.latitude_deg = gps_value.latitude;
    GPScoords.longitude_deg = gps_value.longitude;
    //printf("WITH_CMLOCAL\n latitude = %f longitude = %f", gps_value.latitude, gps_value.longitude);
    //The number 10... from CavPoint, is copied from lgsls.
    GPScoords.elevation = 10.12442684173584;
    //Log("Ref_Veh_FrgpsinConversionCpp: %f\t %f\t %f\n", GPScoords.latitude_deg, GPScoords.longitude_deg, GPScoords.elevation);
    tUTM_PosData UTMcoords;
    UTM_FromGPS2UTM_Conv(&UTMcoords, &GPScoords);

        //Log("Ref_Veh_UTM: %f\t %f\n", UTMcoords.east, UTMcoords.north);

    gps->mutable_localization()->mutable_position()->set_x(UTMcoords.east);
    gps->mutable_localization()->mutable_position()->set_y(UTMcoords.north);
    gps->mutable_localization()->mutable_position()->set_z( GPScoords.elevation);

#else
printf("NOT WITH_CMLOCAL\n");
    GPScoord gps_value = gps_coord.load(std::memory_order_relaxed);
    NorthEast ne = convertLongLat2NE(gps_value);

    gps->mutable_localization()->mutable_position()->set_x(ne.x);
    gps->mutable_localization()->mutable_position()->set_y(ne.y);
    gps->mutable_localization()->mutable_position()->set_z(10.12442684173584);
#endif

    float heading_deg = veh_heading_deg_atomic.load(std::memory_order_relaxed);
    const float qx = 0;
    const float qy = 0;
    float qz = std::sin((heading_deg / 2) * PI / 180.0f);
    float qw = std::cos((heading_deg / 2) * PI / 180.0f);
    gps->mutable_localization()->mutable_orientation()->set_qx(qx);
    gps->mutable_localization()->mutable_orientation()->set_qy(qy);
    gps->mutable_localization()->mutable_orientation()->set_qz(qz);
    gps->mutable_localization()->mutable_orientation()->set_qw(qw);

    gps->mutable_localization()->set_heading(heading_deg);
}

void set_position_proto_msg(std::unique_ptr<apollo::drivers::gnss::GnssBestPose> &bestPose,double time_stamp)
{
    static bool positionInitialized = false;
    static uint32_t sequence_number = 0U;
    static double height_msl = 0.0;

    double t_epoch = get_system_time (time_stamp);

   // Calc the measurement time for gnss best pose according to lgsvl
   // System.DateTime GPSepoch = new System.DateTime(1980, 1, 6, 0, 0, 0, System.DateTimeKind.Utc);
   // measurement_time = (double)(System.DateTime.UtcNow - GPSepoch).TotalSeconds + 18.0f;
    double t_pose = t_epoch - 315964800.0 + 18.0;

    bestPose->mutable_header()->set_timestamp_sec(t_pose);
    bestPose->mutable_header()->set_sequence_num(sequence_number);
    sequence_number++;

    if (!positionInitialized)
    {
        positionInitialized = true;
        bestPose->set_sol_status(apollo::drivers::gnss::SolutionStatus::SOL_COMPUTED);
        bestPose->set_sol_type(apollo::drivers::gnss::SolutionType::NARROW_INT);
        bestPose->set_datum_id(apollo::drivers::gnss::DatumId::WGS84);
        bestPose->set_undulation(0.0f);
        bestPose->set_latitude_std_dev(0.01);
        bestPose->set_longitude_std_dev(0.01);
        bestPose->set_height_std_dev(0.01);
        bestPose->set_base_station_id("0");
        bestPose->set_differential_age(2.0);
        bestPose->set_solution_age(0.0);
        bestPose->set_num_sats_tracked(15);
        bestPose->set_num_sats_in_solution(15);
        bestPose->set_num_sats_l1(15);
        bestPose->set_num_sats_multi(12);
        bestPose->set_extended_solution_status(33);
        bestPose->set_galileo_beidou_used_mask(0);
        bestPose->set_gps_glonass_used_mask(51);
    }

    GPScoord gps_value = gps_coord.load(std::memory_order_relaxed);
    bestPose->set_measurement_time(t_pose);
    bestPose->set_latitude(gps_value.latitude);
    bestPose->set_longitude(gps_value.longitude);
    bestPose->set_height_msl(height_msl);
}

void set_position_proto_msg(std::unique_ptr<apollo::canbus::Chassis> &chassis){
    static bool positionInitialized = false;
    auto chassis_gps = chassis->mutable_chassis_gps();

    if (!positionInitialized)
    {   
        positionInitialized = true;
        chassis_gps->set_gps_valid(true);
        
        chassis_gps->set_is_gps_fault(false);
        chassis_gps->set_is_inferred(false);
        
        chassis_gps->set_num_satellites(15);
 
        chassis_gps->set_altitude(0.0);
        chassis_gps->set_pdop(0.1f);
        chassis_gps->set_hdop(0.1f);
        chassis_gps->set_vdop(0.1f);
        chassis_gps->set_quality(apollo::canbus::GpsQuality::FIX_3D);
    }
        
    GPScoord gps_value = gps_coord.load(std::memory_order_relaxed);
    chassis_gps->set_latitude(gps_value.latitude);
    chassis_gps->set_longitude(gps_value.longitude);
    auto syst_time_now = std::chrono::system_clock::now();
    time_t tt = std::chrono::system_clock::to_time_t(syst_time_now);
    tm local_time = *localtime(&tt);
    chassis_gps->set_year(local_time.tm_year + 1900);
    chassis_gps->set_month(local_time.tm_mon);
    chassis_gps->set_day(local_time.tm_mday);
    chassis_gps->set_hours(local_time.tm_hour);
    chassis_gps->set_minutes(local_time.tm_min);
    chassis_gps->set_seconds(local_time.tm_sec);

    float heading_deg = veh_heading_deg_atomic.load(std::memory_order_relaxed);
    // TODO: ipg: in lgsvl, heading for chassis is opposite to the heading for correct_imu.

    heading_deg = -1 * (heading_deg - 45.3058);

    float compass_heading_deg;
    int quotient = 0;
    std::remquo(heading_deg,45.0,&quotient);

    compass_heading_deg = quotient * 45.0;
    chassis_gps->set_compass_direction(compass_heading_deg);  // TODO: ipg: change with the heading

    chassis_gps->set_heading(heading_deg);
    chassis_gps->set_gps_speed(veh_speed_mps.load(std::memory_order_relaxed));

}

void set_chassis_powertrain_proto_msg(std::unique_ptr<apollo::canbus::Chassis> &chassis,double time_stamp){
    static bool chassisInitialized = false;
    static uint32_t sequence_number = 0U;

    double t_epoch = get_system_time (time_stamp);

    chassis->mutable_header()->set_timestamp_sec(t_epoch);
    chassis->mutable_header()->set_sequence_num(sequence_number);
    sequence_number++;

    if(!chassisInitialized){
        chassisInitialized = true;
        chassis->mutable_header()->set_module_name("chassis");
        chassis->set_engine_started(true);
        chassis->set_odometer_m(0.0);
        chassis->set_fuel_range_m(0.0);
        //zez: those 4 signals is deprecated
//        chassis->set_high_beam_signal(false);
//        chassis->set_low_beam_signal(false);
//        chassis->set_left_turn_signal(false);
//        chassis->set_right_turn_signal(false);
        chassis->set_wiper(false);
        chassis->set_driving_mode(apollo::canbus::Chassis_DrivingMode::Chassis_DrivingMode_COMPLETE_AUTO_DRIVE);
        chassis->set_gear_location(apollo::canbus::Chassis_GearPosition::Chassis_GearPosition_GEAR_DRIVE);
    }

    chassis->set_speed_mps(veh_speed_mps.load(std::memory_order_relaxed));
    chassis->set_engine_rpm(veh_engine_rpm.load(std::memory_order_relaxed));
    chassis->set_throttle_percentage(veh_throttle_sensor_pct.load(std::memory_order_relaxed));
    chassis->set_brake_percentage(veh_brake_sensor_pct.load(std::memory_order_relaxed));
    chassis->set_steering_percentage(veh_steering_sensor_pct.load(std::memory_order_relaxed));
}

void set_position_proto_msg(std::unique_ptr<apollo::localization::CorrectedImu> &imu,double time_stamp)
{
    static uint32_t sequence_number = 0U;

    double t_epoch = get_system_time (time_stamp);

    imu->mutable_header()->set_timestamp_sec(t_epoch);
    imu->mutable_header()->set_sequence_num(sequence_number);
    sequence_number++;

    imu->mutable_imu()->mutable_linear_acceleration()->set_x(veh_global_accelation_x.load(std::memory_order_relaxed));
    imu->mutable_imu()->mutable_linear_acceleration()->set_y(veh_global_accelation_y.load(std::memory_order_relaxed));
    imu->mutable_imu()->mutable_linear_acceleration()->set_z(0.0);

    imu->mutable_imu()->mutable_angular_velocity()->set_x(veh_angular_velocity_x.load(std::memory_order_relaxed));
    imu->mutable_imu()->mutable_angular_velocity()->set_y(veh_angular_velocity_y.load(std::memory_order_relaxed));
    imu->mutable_imu()->mutable_angular_velocity()->set_z(veh_angular_velocity_z.load(std::memory_order_relaxed));

    float heading_deg = veh_heading_deg_atomic.load(std::memory_order_relaxed);

    // the 45.308 is manually measured from apollo
    heading_deg = heading_deg - 45.3058;
    imu->mutable_imu()->set_heading(heading_deg);
    imu->mutable_imu()->mutable_euler_angles()->set_x(0.0);
    imu->mutable_imu()->mutable_euler_angles()->set_y(0.0);
    imu->mutable_imu()->mutable_euler_angles()->set_z(heading_deg / 180.0f * PI);
}

void set_position_proto_msg(std::unique_ptr<apollo::drivers::gnss::Imu> &imu,double time_stamp)
{
    static uint32_t sequence_number = 0U;


    double t_epoch = get_system_time (time_stamp);

    imu->mutable_header()->set_timestamp_sec(t_epoch);
    imu->set_measurement_time(t_epoch);
    imu->set_measurement_span(0.01);
    imu->mutable_header()->set_sequence_num(sequence_number);
    sequence_number++;
    //TODO: ipg: now the map is even, there is no gravity in x,y direction
    imu->mutable_linear_acceleration()->set_x(veh_global_accelation_x.load(std::memory_order_relaxed));
    imu->mutable_linear_acceleration()->set_y(veh_global_accelation_y.load(std::memory_order_relaxed));
    imu->mutable_linear_acceleration()->set_z(9.81001091003418);

    imu->mutable_angular_velocity()->set_x(veh_angular_velocity_x.load(std::memory_order_relaxed));
    imu->mutable_angular_velocity()->set_y(veh_angular_velocity_y.load(std::memory_order_relaxed));
    imu->mutable_angular_velocity()->set_z(veh_angular_velocity_z.load(std::memory_order_relaxed));
}
