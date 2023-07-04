#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <cassert>
#include <vector>
#include <map>
#include <list>

#include "google/protobuf/descriptor.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"

/* Added for sensor image */
#include "modules/drivers/proto/sensor_image.pb.h"
//#include "modules/drivers/camera/proto/config.pb.h"

extern "C" {
#include "cyberbridge_C_interface.h"
}

#include "cloud_doublebuffer.h"
#include "bridge_tcp_client.h"
#include "cyberbridge.h"
#include "conversion.h"

static CyberBridge * bridge = nullptr;

void construct_bridge(){
    bridge = new CyberBridge();
}

void destroy_bridge(){
    //std::cout<<"Destroying Bridge"<<std::endl;
    if(bridge) {
        // Delete bridge if it has not been deleted before
        delete bridge;
    }
    bridge = nullptr;
}

int is_bridge_connected(){
    if(!bridge){
        return 0;
    }
    return bridge->is_connected();
}


int set_bridge_connection_state(int is_tcp_connected){
    if(!bridge){
        return 0;
    } else {
        bridge->set_connected(is_tcp_connected);
        return 1;
    }
}

void init_descriptorsetfile(){
    if(!bridge){
        std::cout<<"[init_descriptorsetfile] Bridge has not been initialized !!!"<<std::endl;
        return;
    }
    std::unique_ptr<google::protobuf::FileDescriptorSet> descr = bridge->deserializeDescritorSetFile();
}

void send_low_framerate(double time_stamp){
    // Call this every 0.085s
    if(!bridge){
        std::cout<<"[send_position] Bridge has not been initialized !!!"<<std::endl;
        return;
    }
    bridge->WriteInsStat(time_stamp);
    bridge->WriteGnssBestPose(time_stamp);
    bridge->WriteOdometry(time_stamp);

    bridge->WriteChassis(time_stamp);
}

void send_conti_radar(double time_stamp){
     if(!bridge){
        std::cout<<"[send_position] Bridge has not been initialized !!!"<<std::endl;
        return;
    }
    bridge->WriteRadarFront(time_stamp);
}

void send_high_framerate(double time_stamp){
    // Call this every 0.01s
    if(!bridge){
        std::cout<<"[send_chassis] Bridge has not been initialized !!!"<<std::endl;
        return;
    }
//    bridge->WriteChassis();
    bridge->WriteImu(time_stamp);
    bridge->WriteCorrectedImu(time_stamp);
}

void send_cloud(double time_stamp){
    if(!bridge){
        std::cout<<"[send_cloud] Bridge has not been initialized !!!"<<std::endl;
        return;
    }
    bridge->WriteCloud(time_stamp);
}

void send_radar_sensor(double time_stamp){
    if(!bridge){
        std::cout<<"[send_cloud] Bridge has not been initialized !!!"<<std::endl;
        return;
    }
    bridge->WriteRadarFront(time_stamp);
}

void send_image(double time_stamp){
    if(!bridge){
        std::cout<<"[send_cloud] Bridge has not been initialized !!!"<<std::endl;
        return;
    }
    bridge->WriteImage(time_stamp);
}


// void send_rador_sensor_msg( double time_stamp){
//     if(!bridge){
//         std::cout<<"[send_cloud] Bridge has not been initialized !!!"<<std::endl;
//         return;
//     }
//     bridge->WriteRadarFront(time_stamp);
// }

//void send_image(){
//    if(!bridge){
//        std::cout<<"[send_cloud] Bridge has not been initialized !!!"<<std::endl;
//        return;
//    }
//    bridge->WriteImage();
//}

unsigned int get_apollo_seqNum_value() {
	return get_seqNum_value () ;
}


float get_apollo_brake_value(){
    return get_brake_value();
}

float get_apollo_steering_value(){
    return get_steering_value();
}

float get_apollo_throttle_value(){
    return get_throttle_value();
}

double get_apollo_timespec_value(){
//	double time = get_timespec_value();
//	 printf ("1.the time apollo is %lf !\n", time*1000  );

    return get_timespec_value();
}

double get_apollo_lidartimespec_value(){
    return get_lidartimespec_value();
}

double get_apollo_timeexcee_value(){
    return get_timeexcee_value();
}

double get_apollo_timetotal_value(){
    return get_timetotal_value();
}


float get_apollo_headErr_value(){
    return get_headErr_value();
}

float get_apollo_lateralErr_value(){
    return get_lateralErr_value();
}

double get_lastCtrl_time(){
	double tmp;
	tmp = bridge->lastControlUpdateTime;
    return tmp;
}

// uint32_t get32le(size_t offset) {
//   return tcp_buffer_in[offset + 0] | (tcp_buffer_in[offset + 1] << 8) |
//          (tcp_buffer_in[offset + 2] << 16) | (tcp_buffer_in[offset + 3] << 24);
// }

void receive_control_data(unsigned char * tcp_buffer_in, size_t tcp_buf_size){
    // op_type(1字节)|channel_size(4字节)|channel(未知)|msg_size(4字节)|msg(未知)
    if (sizeof(uint8_t) + 2 * sizeof(uint32_t) > tcp_buf_size) {
        return;
    }
    
    size_t offset = sizeof(uint8_t);

    uint32_t channel_length = tcp_buffer_in[offset + 0] | (tcp_buffer_in[offset + 1] << 8) | (tcp_buffer_in[offset + 2] << 16) | (tcp_buffer_in[offset + 3] << 24);
    offset += sizeof(uint32_t);

    //std::cout<<"channel_length = "<<channel_length<<std::endl;

    if (offset + channel_length > tcp_buf_size) {
        std::cout<<"channel_length larger than tcp_buf_size"<<std::endl;
        return;
    }
    std::string channel(reinterpret_cast<char*>(&tcp_buffer_in[offset]), channel_length);
    offset += channel_length;
    //std::cout<<"channel = "<<channel<<std::endl;
    uint32_t message_length = tcp_buffer_in[offset + 0] | (tcp_buffer_in[offset + 1] << 8) | (tcp_buffer_in[offset + 2] << 16) | (tcp_buffer_in[offset + 3] << 24);
    offset += sizeof(uint32_t);
    if (offset + message_length > tcp_buf_size) {
        std::cout<<"message_lenth larger than tcp_buf_size"<<std::endl;
        return;
    }
    //std::cout<<"message_length = "<<message_length<<std::endl;
    std::string message(reinterpret_cast<char*>(&tcp_buffer_in[offset]), message_length);
    offset += message_length;


    // static std::string buffer;
    // buffer.reserve(1096);
    // buffer.insert(buffer.end(), &tcp_buffer_in[0], &tcp_buffer_in[tcp_buf_size]);
    if (bridge != nullptr){
        bridge->ReceiveControl(channel, message);
    }
}

int is_apollo_control_data_fresh(){
    if(bridge == nullptr){
        return false;
    }
    auto time_now = std::chrono::system_clock::now();
    double t_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(
        time_now.time_since_epoch()).count()*1e-9; // seconds
    // if the last ctrl msgs is not received within 1 s .Not used it, the 1s is for testing , it should be shorter.
    if(t_epoch > 1 + bridge->lastControlUpdateTime){
        //printf ("control data is not fresh!\n" );
    	return false;
    } else {
        return true;
    }
}
