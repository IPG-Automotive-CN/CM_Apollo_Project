
#ifndef CYBBERBRIDGE_C_INTERFACE_H

unsigned int get_apollo_seqNum_value(void);

float get_apollo_throttle_value(void);
float get_apollo_brake_value(void);
float get_apollo_steering_value(void);
float get_apollo_headErr_value(void);
float get_apollo_lateralErr_value(void);


double get_apollo_timespec_value(void);
double get_apollo_lidartimespec_value(void);
double get_lastCtrl_time(void);
double get_apollo_timeexcee_value(void);
double get_apollo_timetotal_value(void);

void construct_bridge();
void destroy_bridge();
void init_descriptorsetfile(void);
void send_low_framerate(double time_stamp);
void send_high_framerate(double time_stamp);
void send_cloud(double time_stamp);
void send_radar_sensor(double time_stamp);

//void send_rador_sensor_msg(double time_stamp)

void reset_cloud();
int is_bridge_connected();
void receive_control_data(unsigned char * tcp_buffer_in, size_t tcp_buf_size);
int is_apollo_control_data_fresh();
int set_bridge_connection_state(int is_tcp_connected);

/*Added for sensor image*/
void send_image(double time_stamp);


void send_conti_radar();

#endif /* CYBBERBRIDGE_C_INTERFACE_H */
