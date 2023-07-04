#include <mutex>
#include <vector>
#include <memory>
#include <tuple>


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
#include "modules/drivers/proto/sensor_image.pb.h"

#include "cloud_doublebuffer.h"
#include "conversion.h"

std::tuple<std::vector<CloudPoint> *, int> DoubleBuffer::update_target_buffer(){
    int write_index;
    mutex.lock();
    if(now_reading_index > 0){
        write_index = !now_reading_index;
    } else {
        write_index = !index_to_newest;
    }
    mutex.unlock();

    // Set target buffer
    std::vector<CloudPoint> * cloud;
    if(write_index == 0){
        cloud = cloud_buffer_0.get();
    } else {
        cloud = cloud_buffer_1.get();
    }
    cloud->clear(); // Clear buffer before it is written into
    auto cloud_and_index_tuple = std::make_tuple(cloud, write_index);
    return cloud_and_index_tuple;
}

void DoubleBuffer::release_new_index(int & write_index, double timestamp){
    // Update index
    mutex.lock();
    index_to_newest = write_index;
    last_timestamp = timestamp;
    mutex.unlock();
}

DoubleBuffer::DoubleBuffer(){
    //std::cout<<"DoubleBuffer construction"<<std::endl;
    mutex.unlock();
    index_to_newest = 0;
    now_reading_index = -1;
    last_timestamp = 0.0f;
    cloud_buffer_0 = std::make_unique<std::vector<CloudPoint>>();
    cloud_buffer_1 = std::make_unique<std::vector<CloudPoint>>();

    cloud_buffer_0.get()->reserve(512*1024);
    cloud_buffer_1.get()->reserve(512*1024);
    cloud_buffer_0.get()->clear();
    cloud_buffer_1.get()->clear();
};


DoubleBuffer::~DoubleBuffer(){
    //std::cout<<"[~DoubleBuffer] DoubleBuffer destruction"<<std::endl;
}

void DoubleBuffer::set(float * x_array, float * y_array, float * z_array,
                        size_t number_elements, double timestamp){

    auto [cloud, write_index] = update_target_buffer();

    // Populate vector
    for (size_t i = 0; i < number_elements; i++)
    {
        CloudPoint p = {
            .x = x_array[i],
            .y = y_array[i],
            .z = z_array[i],
            .intensity = 128.0f,
        };
        cloud->push_back(p);
    }

    // Finished populating vector
    release_new_index(write_index, timestamp);
}

std::vector<CloudPoint> DoubleBuffer::get_fresh_buffer(){
    std::vector<CloudPoint> * cloud_ptr;
    mutex.lock();
    now_reading_index = index_to_newest;
    mutex.unlock();
    if(now_reading_index == 0){
        cloud_ptr = cloud_buffer_0.get();
    } else {
        cloud_ptr = cloud_buffer_1.get();
    }

    //std::vector<CloudPoint> cloud_copy(*cloud_ptr);
    mutex.lock();
    now_reading_index = -1;
    mutex.unlock();
    return *cloud_ptr;
}

double DoubleBuffer::getLastTimestamp(){
    return last_timestamp;
}

void DoubleBuffer::mutex_unlock(){
    mutex.unlock();
}





