#ifndef CLOUD_DOUBLE_BUFFER_H


struct CloudPoint
{
    float x;
    float y;
    float z;
    float intensity;
};

class DoubleBuffer
{
private:
    std::mutex mutex;
    volatile int index_to_newest;
    volatile int now_reading_index;
    double last_timestamp;
    std::unique_ptr<std::vector<CloudPoint>> cloud_buffer_0;
    std::unique_ptr<std::vector<CloudPoint>> cloud_buffer_1;
    std::tuple<std::vector<CloudPoint> *, int> update_target_buffer();
    void release_new_index(int & write_index, double timestamp);
public:
    DoubleBuffer();
    ~DoubleBuffer();
    void set(float * x_array, float * y_array, float * z_array,
             size_t number_elements, double timestamp);
    double getLastTimestamp(void);
    std::vector<CloudPoint> get_fresh_buffer();
    void mutex_unlock();
};

#endif /* CLOUD_DOUBLE_BUFFER_H */

