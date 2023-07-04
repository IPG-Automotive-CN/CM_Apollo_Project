#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <map>
#include <cmath>
#include <atomic>
#include <chrono>
#include <thread>


#include "modules/control/proto/control_cmd.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/descriptor.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"

/* Added for sensor image */
#include "modules/drivers/proto/sensor_image.pb.h"
//#include "modules/drivers/camera/proto/config.pb.h"

extern "C"
{
#include "bridge_tcp_client_c_interface.h"
}

#include "base64.h"
#include "bridge_tcp_client.h"
#include "cloud_doublebuffer.h"
#include "conversion.h"

#include "cyberbridge.h"

int gasCount = 0;
static constexpr uint8_t BridgeOp_RegisterDesc = 1; 
static constexpr uint8_t BridgeOp_AddReader = 2; 
static constexpr uint8_t BridgeOp_AddWriter = 3; 
static constexpr uint8_t BridgeOp_Publish = 4; 

std::map<std::string, std::string> CyberBridge::NameByMsgType = {};
std::map<std::string, std::pair<std::string, google::protobuf::FileDescriptorProto>> CyberBridge::DescriptorByName = {};

// the flag to make sure the tcp connection only be started once at the init. phase
static int conne_init = 0;

CyberBridge::CyberBridge()
{

    apollo::drivers::ContiRadar contiRadar;

    cloud_double_buffer = std::make_unique<DoubleBuffer>();
    define_double_buffer_ptr(cloud_double_buffer.get());
    
    // Opening TCP connection

   connected = conne_init;

   if (conne_init<1) {
    set_connected(tcp_client_init());
   }


    if(!is_connected()){
        //std::cout<<"Bridge is not connected to Apollo...."<<std::endl;
        return;
    } else {
        //std::cout<<"[CyberBridge::CyberBridge] Bridge is connected to Apollo...."<<std::endl;

        conne_init = 1;
    }


    // Prepare descriptor set
    NameByMsgType = std::map<std::string, std::string>();
    DescriptorByName = std::map<std::string, std::pair<std::string, google::protobuf::FileDescriptorProto>> ();
    auto set = deserializeDescritorSetFile();
    if(set->file_size()==0){
        std::cout<<"[CyberBridge::CyberBridge] Problem File list should be populated:"<<std::endl;
        std::cout<<"[CyberBridge::CyberBridge] Is the protobuf_descriptorset.base64 file at the right location?"<<std::endl;
        return;
    }
    for(auto descriptor : set->file()){
        auto descriptorName = descriptor.name();
        // std::cout<<"descriptorName:"<<descriptorName<<std::endl;

        std::string data;
        descriptor.SerializeToString(&data);
        auto tuple = std::make_pair(data, descriptor);
        DescriptorByName.try_emplace(descriptorName, tuple);
        auto descriptor_msg_type = *descriptor.mutable_message_type();
        for(auto msgType :  descriptor_msg_type){
            std::string fullMsgType = descriptor.package() + "." + msgType.name();
            NameByMsgType.try_emplace(fullMsgType, descriptorName);
        }
    }

    //Add writers
    addWriterInsStat();
    addWriterImu();
    addWriterCorrectedImu();
    addWriterGnssBestPose();
    addWriterOdometry();
    addWriterChassis();
    addWriterCloud();
    addWriterRadarFront();

    /* Added for sensor image */
    addWriterImage();

    //Add reader
    lastControlUpdateTime = 0;
    addReaderControl();
    //TODO: ipg: why here to wait 200?
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

CyberBridge::~CyberBridge(){
    // Destructor
	/* not disconnect tcp connection after simulation stop */
/*	std::cout<<"[~CyberBridge] Stopping the Cyberbridge TCP client"<<std::endl;
    tcp_client_stop();*/
    
	//std::cout<<"[~CyberBridge] Clearing double buffer"<<std::endl;
    delete_double_buffer_ptr();
}


////////////////////////////////////////////////////////////////////////////////////
////                          Util functions                                    ////
////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <unistd.h>

int CyberBridge::is_connected(){
    return connected;
}

void CyberBridge::set_connected(int is_tcp_connected){
    connected = is_tcp_connected;
}

std::string CyberBridge::unpackDescritorSetFile()
{
	// see code for CyberBridge.cs:CyberBridge constructor
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != NULL)
    {
        //std::cout << "[CyberBridge::unpackDescritorSetFile()] Current directory is:" << cwd << std::endl;
    }
    // Open base 64 file
    std::string descr_file_path{"assets/protobuf_descriptorset.base64"};
    std::ifstream file(descr_file_path);
    std::string file_content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    std::string descr_raw = base64_decode(file_content);
   // std::cout << "[CyberBridge::unpackDescritorSetFile()] Current directory is:" << descr_raw << std::endl;
    return descr_raw;
}

std::unique_ptr<google::protobuf::FileDescriptorSet> CyberBridge::deserializeDescritorSetFile(){
    std::string descr_raw = unpackDescritorSetFile();
    auto fd_proto = std::make_unique<google::protobuf::FileDescriptorSet>();
    fd_proto->ParseFromString(descr_raw);
    return fd_proto;
}

std::string & ProtoWriterInterface::get_topic()
{
    return topic;
}

std::string & ProtoWriterInterface::get_type()
{
    return type;
}

template<typename ProtobufType>
ProtoWriter<ProtobufType>::ProtoWriter(){}

template<typename ProtobufType>
ProtoWriter<ProtobufType>::ProtoWriter(std::string topic,
     std::string type)
{
    this->topic = topic;
    this->type = type;
    this->proto = std::make_unique<ProtobufType>();
}

template<typename ProtobufType>
ProtoWriter<ProtobufType>::~ProtoWriter(){};

ProtoWriter<apollo::drivers::gnss::InsStat> CyberBridge::insStatWriter
    { "/apollo/sensor/gnss/ins_stat", "apollo.drivers.gnss.InsStat"};

ProtoWriter<apollo::drivers::gnss::Imu> CyberBridge::imuWriter
    { "/apollo/sensor/gnss/imu", "apollo.drivers.gnss.Imu"};

ProtoWriter<apollo::localization::CorrectedImu> CyberBridge::correctedImuWriter
    { "/apollo/sensor/gnss/corrected_imu", "apollo.localization.CorrectedImu"};

ProtoWriter<apollo::drivers::gnss::GnssBestPose> CyberBridge::gnssBestPoseWriter
    { "/apollo/sensor/gnss/best_pose", "apollo.drivers.gnss.GnssBestPose"};

ProtoWriter<apollo::localization::Gps> CyberBridge::odometryWriter
    { "/apollo/sensor/gnss/odometry", "apollo.localization.Gps"};

ProtoWriter<apollo::canbus::Chassis> CyberBridge::chassisWriter
    {"/apollo/canbus/chassis", "apollo.canbus.Chassis"};

ProtoWriter<apollo::drivers::PointCloud> CyberBridge::cloudWriter
    {"/apollo/sensor/lidar128/compensator/PointCloud2", "apollo.drivers.PointCloud"};

ProtoWriter<apollo::drivers::ContiRadar> CyberBridge::radarFrontWriter
    {"/apollo/sensor/radar/front", "apollo.drivers.ContiRadar"};

ProtoWriter<apollo::drivers::ContiRadar> CyberBridge::radarRearWriter
    {"/apollo/sensor/radar/rear", "apollo.drivers.ContiRadar"};

ProtoWriter<apollo::drivers::Image> CyberBridge::imageWriter
	{"/apollo/sensor/camera/front_6mm/image", "apollo.drivers.Image"};

/* Added for sensor image */
//ProtoWriter<apollo::drivers::CompressedImage> CyberBridge::imageWriter
//	{"/camera/image_raw", "apollo.drivers.CompressedImage"};


////////////////////////////////////////////////////////////////////////////////////
////             Register topics on Apollo server                               ////
////////////////////////////////////////////////////////////////////////////////////

void CyberBridge::addWriterInsStat(){
    //std::cout<<"Run CyberBridge::addWriterInsStat"<<std::endl;
    addWriter(insStatWriter.get_topic(), insStatWriter.get_type());
}

void CyberBridge::addWriterImu(){
    //std::cout<<"Run CyberBridge::addWriterImu"<<std::endl;
    addWriter(imuWriter.get_topic(), imuWriter.get_type());
}

void CyberBridge::addWriterCorrectedImu(){
    //std::cout<<"Run CyberBridge::addWriterCorrectedImu"<<std::endl;
    addWriter(correctedImuWriter.get_topic(), correctedImuWriter.get_type());
}

void CyberBridge::addWriterGnssBestPose(){
    //std::cout<<"Run CyberBridge::addWriterGnssBestPose"<<std::endl;
    addWriter(gnssBestPoseWriter.get_topic(), gnssBestPoseWriter.get_type());
}

void CyberBridge::addWriterOdometry(){
    //std::cout<<"Run CyberBridge::addWriterOdometry"<<std::endl;
    addWriter(odometryWriter.get_topic(), odometryWriter.get_type());
}

void CyberBridge::addWriterChassis(){
    //std::cout<<"Run CyberBridge::addWriterChassis"<<std::endl;
    addWriter(chassisWriter.get_topic(), chassisWriter.get_type());
}

void CyberBridge::addWriterCloud(){
    //std::cout<<"Run CyberBridge::addWriterCloud"<<std::endl;
    addWriter(cloudWriter.get_topic(), cloudWriter.get_type());
}

void CyberBridge::addWriterRadarFront(){
    //std::cout<<"Run CyberBridge::addWriterRadarFront"<<std::endl;
    addWriter(radarFrontWriter.get_topic(), radarFrontWriter.get_type());
}

void CyberBridge::addWriterRadarRear(){
    //std::cout<<"Run CyberBridge::addWriterRadarRear"<<std::endl;
    addWriter(radarRearWriter.get_topic(), radarRearWriter.get_type());
}


void CyberBridge::addWriterImage(){
    //std::cout<<"Run CyberBridge::addWriterImage"<<std::endl;
    addWriter(imageWriter.get_topic(), imageWriter.get_type());
}

void CyberBridge::addReaderControl(){
    //std::cout<<"Run CyberBridge::addReaderControl"<<std::endl;
    addReader("/apollo/control", "apollo.control.ControlCommand");
}


void CyberBridge::addReader(std::string topic, std::string type){
    const std::string & channelBytes = topic;
    const std::string & typeBytes = type;

    std::unique_ptr<std::string> bytes = std::make_unique<std::string>();
    
    bytes->reserve(1024);
    bytes->push_back(BridgeOp_AddReader);
    append32bLittleEndian(bytes.get(), static_cast<uint32_t>(channelBytes.size()));
    bytes->insert( bytes->end(), channelBytes.begin(), channelBytes.end() ); // append channelBytes to bytes
    
    append32bLittleEndian(bytes.get(), static_cast<uint32_t>(typeBytes.size()));
    bytes->insert( bytes->end(), typeBytes.begin(), typeBytes.end() ); // append channelBytes to bytes
    
    //std::cout<<"[addReader]:"<<topic<<std::endl;
    tcp_send(*(bytes.get()));
}

void CyberBridge::addWriter(std::string topic, std::string type){
    std::string descriptorName = NameByMsgType[type];
    auto descriptor = DescriptorByName[descriptorName].second;
    auto descriptors = std::list<std::string>();
    GetDescriptors(descriptors, descriptor);

    int count = descriptors.size();
    
    std::unique_ptr<std::string> bytes = std::make_unique<std::string>();
    bytes->reserve(4096);
    bytes->push_back(BridgeOp_RegisterDesc);
    append32bLittleEndian(bytes.get(), static_cast<uint32_t>(count));
    for(auto desc : descriptors){
        size_t length = desc.size();
        append32bLittleEndian(bytes.get(), static_cast<uint32_t>(length));
        bytes->insert( bytes->end(), desc.begin(), desc.end() ); // append desc to bytes
//        std::cout<<"[addWriter]:"<<topic<< "des " <<desc <<std::endl;
    }

    std::string & channelBytes = topic;
    std::string & typeBytes = type;

    bytes->push_back(BridgeOp_AddWriter);
    append32bLittleEndian(bytes.get(), static_cast<uint32_t>(channelBytes.size()));
    bytes->insert(bytes->end(), channelBytes.begin(), channelBytes.end() ); // append bytes and channelBytes to bytes
    append32bLittleEndian(bytes.get(), static_cast<uint32_t>(typeBytes.size()));
    bytes->insert(bytes->end(), typeBytes.begin(), typeBytes.end()); // append bytes and typeBytes to bytes
    
    //std::cout<<"[addWriter]:"<<topic<<std::endl;
    tcp_send(*(bytes.get()));
}

void CyberBridge::GetDescriptors(std::list<std::string> &descriptors, 
    google::protobuf::FileDescriptorProto &descriptor)
{   
    auto dependencies = *descriptor.mutable_dependency();
    for (auto dependency : dependencies)
    {
        auto desc = DescriptorByName[dependency].second;
        GetDescriptors(descriptors, desc);
    }
    auto bytes = DescriptorByName[descriptor.name()].first;
    descriptors.push_back(bytes);

}


////////////////////////////////////////////////////////////////////////////////////
////               Send messages to Apollo server                               ////
////////////////////////////////////////////////////////////////////////////////////

void CyberBridge::WriteInsStat(double time_stamp){
    static auto msg = std::make_unique<std::string>();
    msg->clear();
    auto & protoWriter = insStatWriter;
    set_ins_stat(protoWriter.proto, time_stamp);
    protoWriter.proto->SerializeToString(msg.get());
    Write(protoWriter.get_topic(), *msg.get());
}

void CyberBridge::WriteImu(double time_stamp){
    static auto msg = std::make_unique<std::string>();
    msg->clear();
    auto & protoWriter = imuWriter;
    set_position_proto_msg(protoWriter.proto, time_stamp);
    protoWriter.proto->SerializeToString(msg.get());
    Write(protoWriter.get_topic(), *msg.get());
}

void CyberBridge::WriteCorrectedImu(double time_stamp){
    static auto msg = std::make_unique<std::string>();
    msg->clear();
    auto & protoWriter = correctedImuWriter;
    set_position_proto_msg(protoWriter.proto, time_stamp);
    protoWriter.proto->SerializeToString(msg.get());
    Write(protoWriter.get_topic(), *msg.get());
}

void CyberBridge::WriteGnssBestPose(double time_stamp){
    static auto msg = std::make_unique<std::string>();
    msg->clear();
    auto & protoWriter = gnssBestPoseWriter;
    set_position_proto_msg(protoWriter.proto, time_stamp);
    protoWriter.proto->SerializeToString(msg.get());
    Write(protoWriter.get_topic(), *msg.get());
}

void CyberBridge::WriteOdometry(double time_stamp){
    static auto msg = std::make_unique<std::string>();
    msg->clear();
    auto & protoWriter = odometryWriter;
    set_position_proto_msg(protoWriter.proto, time_stamp);
    protoWriter.proto->SerializeToString(msg.get());
    Write(protoWriter.get_topic(), *msg.get());
}

void CyberBridge::WriteChassis(double time_stamp){
    static auto msg = std::make_unique<std::string>();
    msg->clear();
    auto & protoWriter = chassisWriter;
    set_chassis_powertrain_proto_msg(protoWriter.proto, time_stamp);
    set_position_proto_msg(protoWriter.proto);
    protoWriter.proto->SerializeToString(msg.get());
    Write(protoWriter.get_topic(), *msg.get());
}

void CyberBridge::WriteCloud(double time_stamp){
    static auto msg = std::make_unique<std::string>();
    msg->clear();
    auto & protoWriter = cloudWriter;
    set_cloud_proto_msg(protoWriter.proto, time_stamp);
    protoWriter.proto->SerializeToString(msg.get());
    Write(protoWriter.get_topic(), *msg.get());
}


void CyberBridge::WriteRadarFront(double time_stamp){
    static auto msg = std::make_unique<std::string>();
    msg->clear();
    auto & protoWriter = radarFrontWriter;
    set_radar_front_msg(protoWriter.proto, time_stamp);
    protoWriter.proto->SerializeToString(msg.get());
    Write(protoWriter.get_topic(), *msg.get());
}

void CyberBridge::WriteImage(double time_stamp){
    static auto msg = std::make_unique<std::string>();
    msg->clear();
    auto & protoWriter = imageWriter;
    set_image_proto_msg(protoWriter.proto, time_stamp);
    //SET_PROTO_FUNCTION_TO_DEFINE(protoWriter.proto);
    protoWriter.proto->SerializeToString(msg.get());
    Write(protoWriter.get_topic(), *msg.get());
}

void CyberBridge::Write(std::string & Topic, std::string & msg)
{
    static std::unique_ptr<std::string> data = std::make_unique<std::string>();
    data->reserve(1024);
    data->clear();
    data->push_back((uint8_t)BridgeOp_Publish);
    append32bLittleEndian(data.get(), Topic.size());
    data->insert( data->end(), Topic.begin(), Topic.end() ); // append bytes and desc to bytes

    append32bLittleEndian(data.get(), msg.size());
    data->insert( data->end(), msg.begin(), msg.end() ); // append bytes and desc to bytes

    tcp_send(*(data.get()));
}

////////////////////////////////////////////////////////////////////////////////////
////             Parse received data for vehicle control                        ////
////////////////////////////////////////////////////////////////////////////////////

static std::atomic<float> throttle;
static std::atomic<float> brake;
static std::atomic<float> steering;

static std::atomic<double> timeCtrl;
static std::atomic<double> timelidar;
static std::atomic<float> lateralErr;
static std::atomic<float> headErr;

static std::atomic<double> timeTotal;
static std::atomic<double> timeExcee;

static std::atomic<unsigned int> seqNum;


unsigned int get_seqNum_value() {
	unsigned tmp = seqNum.load(std::memory_order_relaxed);
	return tmp;
}


float get_throttle_value() {
	float tmp = throttle.load(std::memory_order_relaxed);
	return tmp;
}

float get_brake_value() {
	float tmp = brake.load(std::memory_order_relaxed);
	return tmp;
}

float get_steering_value() {
	float tmp = steering.load(std::memory_order_relaxed);
	return tmp;
}

double get_timespec_value() {
	double tmp = timeCtrl.load(std::memory_order_relaxed);
	return tmp;
}

double get_lidartimespec_value() {
	double tmp = timelidar.load(std::memory_order_relaxed);
	return tmp;
}


float get_headErr_value() {
	float tmp = headErr.load(std::memory_order_relaxed);
	return tmp;
}

float get_lateralErr_value() {
	float tmp = lateralErr.load(std::memory_order_relaxed);
	return tmp;
}


double get_timeexcee_value() {
	double tmp = timeExcee.load(std::memory_order_relaxed);
	return tmp;
}

double get_timetotal_value() {
	double tmp = timeTotal.load(std::memory_order_relaxed);
	return tmp;
}



// This function just receives Vehicle Control 
bool CyberBridge::ReceiveControl(std::string &channel, std::string &message) // Inspired from ReceivePublish
{
    static auto controlProto = std::make_unique<apollo::control::ControlCommand> ();

    // see code for CyberBridge.cs:CyberBridge constructor
    // https://github.com/lgsvl/simulator/tree/master/Assets/Scripts/Bridge/Cyber
    if (channel.compare("/apollo/control")==0) // The topic is /apollo/control
    {
        controlProto->Clear();
        controlProto->ParseFromString(message);
//        std::cout<<"\n"<<controlProto->DebugString()<<"\n";  //ipg: maybe we can write a file here

        // TODO: ipg: not used the internal time from the apollo control msgs
        auto time_now = std::chrono::system_clock::now();
        lastControlUpdateTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
        time_now.time_since_epoch()).count()*1e-9; // seconds
        //std::cout<<"\n"<<"Apollo lastControlUpdateTime:"<<lastControlUpdateTime<<std::endl;

        //get timestamp from apollo ctrl msgs to analyse the latency problem
        if (controlProto-> has_header()) {
        	timeCtrl.store(controlProto -> mutable_header()-> timestamp_sec(),std::memory_order_relaxed);
        	timelidar.store(controlProto -> mutable_header()-> lidar_timestamp(),std::memory_order_relaxed);
        	seqNum.store(controlProto -> mutable_header()-> sequence_num(),std::memory_order_relaxed);
        //    std::cout<<"\n"<<"Apollo HeaderProto->timestamp :"<<timeCtrl<<std::endl;
        //    printf ("the time apollo is %lf !\n", timeCtrl*1000  );
        }

        //get latError and headError from apollo ctrl msgs to analyse the latency problem
        if (controlProto-> has_debug()) {
        	lateralErr.store(controlProto -> mutable_debug()-> mutable_simple_lat_debug() -> lateral_error(),std::memory_order_relaxed);
        	headErr.store(controlProto -> mutable_debug()-> mutable_simple_lat_debug() -> heading_error(),std::memory_order_relaxed);
            //std::cout<<"\n"<<"Apollo HeaderProto->timestamp :"<<controlProto -> mutable_header()-> timestamp_sec()<<std::endl;
        }

        //get other signals from apollo ctrl msgs to analyse the latency problem
        if (controlProto-> has_latency_stats()) {
        	timeTotal.store(controlProto -> mutable_latency_stats()->total_time_ms(),std::memory_order_relaxed);
        	timeExcee.store(controlProto -> mutable_latency_stats()->total_time_exceeded(),std::memory_order_relaxed);
            //std::cout<<"\n"<<"Apollo HeaderProto->timestamp :"<<controlProto -> mutable_header()-> timestamp_sec()<<std::endl;
        }

        if (controlProto->has_throttle() and std::isfinite(controlProto->throttle()))
        { /* Number should not be NaN or infinite (has happened) */

            gasCount += 1;

            throttle.store(controlProto->throttle(), std::memory_order_relaxed);
            //std::cout<<"\n"<<"Apollo controlProto->throttle:"<<controlProto->throttle()<<std::endl;
        }

        if (controlProto->has_brake() and std::isfinite(controlProto->brake()))
        { /* Number should not be NaN or infinite (has happened) */
            brake.store(controlProto->brake(), std::memory_order_relaxed);
            //std::cout<<"\n"<<"controlProto->brake:"<<controlProto->brake()<<std::endl;
        }
        if (controlProto->has_steering_target() and std::isfinite(controlProto->steering_target()))
        { /* Number should not be NaN or infinite (has happened) */
            steering.store(controlProto->steering_target(), std::memory_order_relaxed);
            //std::cout<<"\n"<<"controlProto->steering_target:"<<controlProto->steering_target()<<std::endl;
        }
    }
    else
    {
        std::cout<<"[CyberBridge::ReceiveControl] Received message on channel '"<<channel<<"' which nobody expected"<<std::endl;
    }
    return true;
}
