class ProtoWriterInterface{
protected:
    std::string topic;
    std::string type;

public:
    std::string& get_topic();
    std::string& get_type();
};


template<typename ProtobufType>
class ProtoWriter : public ProtoWriterInterface
{
private:
    ProtoWriter();
public:
    std::unique_ptr<ProtobufType> proto;
    ProtoWriter(std::string topic, std::string type);
    ~ProtoWriter();
};


class CyberBridge
{
protected:
    static std::map<std::string, std::string> NameByMsgType;
    static std::map<std::string, std::pair<std::string, google::protobuf::FileDescriptorProto>> DescriptorByName;
    
    static ProtoWriter<apollo::drivers::gnss::InsStat> insStatWriter;
    static ProtoWriter<apollo::drivers::gnss::Imu> imuWriter;
    static ProtoWriter<apollo::localization::CorrectedImu> correctedImuWriter;
    static ProtoWriter<apollo::drivers::gnss::GnssBestPose> gnssBestPoseWriter;
    static ProtoWriter<apollo::localization::Gps> odometryWriter;
    static ProtoWriter<apollo::canbus::Chassis> chassisWriter;
    static ProtoWriter<apollo::drivers::PointCloud> cloudWriter;
    static ProtoWriter<apollo::drivers::ContiRadar> radarFrontWriter;
    static ProtoWriter<apollo::drivers::ContiRadar> radarRearWriter;

    /* Added for image */
    static ProtoWriter<apollo::drivers::Image> imageWriter;


    void addReader(std::string topic, std::string type);
    void addWriter(std::string topic, std::string type);
    std::string unpackDescritorSetFile();
    volatile int connected;

public:
    CyberBridge();
    ~CyberBridge();

    std::unique_ptr<DoubleBuffer> cloud_double_buffer;
    double lastControlUpdateTime;
    
    bool ReceiveControl(std::string &channel, std::string &message);
    void Write(std::string & Topic, std::string & msg);

    void addWriterInsStat();
    void addWriterImu();
    void addWriterCorrectedImu();
    void addWriterGnssBestPose();
    void addWriterOdometry();
    void addWriterChassis();
    void addWriterCloud();
    void addWriterRadarFront();
    void addWriterRadarRear();

    /* Added for image */
    void addWriterImage();

    void addReaderControl();

    void WriteInsStat(double time_stamp);
    void WriteImu(double time_stamp);
    void WriteCorrectedImu(double time_stamp);
    void WriteGnssBestPose(double time_stamp);
    void WriteOdometry(double time_stamp);
    void WriteChassis(double time_stamp);
    void WriteCloud(double time_stamp);
    void WriteRadarFront(double time_stamp);
    void WriteRadarRear(double time_stamp);

    /* Added for image */
    void WriteImage(double time_stamp);

    void set_connected(int is_tcp_connected);
    int is_connected();
    std::unique_ptr<google::protobuf::FileDescriptorSet> deserializeDescritorSetFile();

    void GetDescriptors(std::list<std::string> &descriptors, 
                google::protobuf::FileDescriptorProto &descriptor);
    
};

unsigned int get_seqNum_value();

float get_throttle_value();
float get_brake_value();
float get_steering_value();
float get_headErr_value();
float get_lateralErr_value();

double get_timespec_value();
double get_lidartimespec_value();
double get_timeexcee_value();
double get_timetotal_value();
