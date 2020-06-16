#include <imu_node.h>
#include <sensor_msgs/Imu.h>
#include <ros/package.h>

#define DEGREES_TO_RADIANS(angle) (angle / 180.0 * M_PI)
#define GRAVITY 9.7964

using namespace LibSerial;

template <typename _T>
CalibData<_T>::CalibData()
{
    mis_mat_ << _T(1), -0, 0,
        0, _T(1), -0,
        -0, 0, _T(1);

    scale_mat_ << 0, _T(0), _T(0),
        _T(0), 0, _T(0),
        _T(0), _T(0), 0;

    bias_vec_ << 0, 0, 0;

    update();
}

template <typename _T>
bool CalibData<_T>::load(std::string filename)
{
    std::ifstream file(filename.data());
    if (file.is_open())
    {
        _T mat[9] = {0};

        for (int i = 0; i < 9; i++)
            file >> mat[i];

        mis_mat_ = Eigen::Map<const Eigen::Matrix<_T, 3, 3, Eigen::RowMajor>>(mat);

        for (int i = 0; i < 9; i++)
            file >> mat[i];

        scale_mat_ = Eigen::Map<const Eigen::Matrix<_T, 3, 3, Eigen::RowMajor>>(mat);

        for (int i = 0; i < 3; i++)
            file >> mat[i];

        bias_vec_ = Eigen::Map<const Eigen::Matrix<_T, 3, 1>>(mat);

        update();

        return true;
    }
    return false;
}

template <typename _T>
void CalibData<_T>::update()
{
    ms_mat_ = mis_mat_ * scale_mat_;
}

//==========================

ImuNode::ImuNode() : imu_topic_("/imu"), imu_frame_("imu_frame"), serial_port_name_("/dev/ttyUSB0"),
                     imu_data_record_path_("/tmp/imu_raw.log"), publish_rate_(200), enable_record_(false),
                     debug_raw_data_(false), compensate_with_calib_(false), acc_calib_file_("acc.calib"),
                     gyro_calib_file_("gyro.calib")
{
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    n.param<std::string>("serial_port_name", serial_port_name_, "/dev/ttyUSB0");
    n.param<std::string>("imu_topic", imu_topic_, "/imu");
    std::cout<<"debug_raw_data_:"<<imu_topic_<<std::endl;
    n.param<std::string>("imu_frame", imu_frame_, "/imu_frame");
    std::cout<<"imu_frame_:"<<imu_frame_<<std::endl;
    n.param<std::string>("imu_data_record_path", imu_data_record_path_, "/tmp/imu_raw.log");
    std::cout<<"debug_raw_data_:"<<imu_data_record_path_<<std::endl;

    nh.param<int>("publish_rate", publish_rate_, 200);
    nh.param<bool>("debug_raw_data", debug_raw_data_, false);
    nh.param<bool>("conpensate_with_calib", compensate_with_calib_, false);
    nh.param<std::string>("acc_calib_file", acc_calib_file_, "acc.calib");
    nh.param<std::string>("gyro_calib_file", gyro_calib_file_, "gyro.calib");
    std::cout<<"debug_raw_data_:"<<debug_raw_data_<<" compensate_with_calib_:"<<compensate_with_calib_<<" acc_calib_file_:"<<acc_calib_file_<<std::endl;
    pub_imu_ = n.advertise<sensor_msgs::Imu>(imu_topic_, 50);
    record_control_ = n.advertiseService("set_record", &ImuNode::RecordImu, this);

    Init();

    imu_data_record_.open(imu_data_record_path_, std::ios::out | std::ios::app);
    if (!imu_data_record_.is_open())
    {
        ROS_INFO(" open imu_data_record_: %s failed !!!!!! ", imu_data_record_path_.c_str());
    }

    if (compensate_with_calib_)
    {
        std::string path = ros::package::getPath("imu_node") + "/param/";
        ROS_INFO("compensate_with_calib_ : %s  | %s | %s", path.c_str(), acc_calib_file_.c_str(), gyro_calib_file_.c_str());
        acc_calib_.load(path + acc_calib_file_);
        gyro_calib_.load(path + gyro_calib_file_);
    }
}

ImuNode::~ImuNode()
{
    imu_data_record_.close();
}

bool ImuNode::RecordImu(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    enable_record_ = req.data;
    res.success = true;

    return true;
}

void ImuNode::Init()
{
    try
    {
        imu_serial_port_.Open(serial_port_name_);
    }
    catch (OpenFailed)
    {
        ROS_ERROR("Open %s Failed! Please Check the serial port number and authority!", serial_port_name_);
        return;
    }
    imu_serial_port_.SetBaudRate(BaudRate::BAUD_115200);
    imu_serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    imu_serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    imu_serial_port_.SetParity(Parity::PARITY_NONE);
    imu_serial_port_.SetStopBits(StopBits::STOP_BITS_1);
    imu_serial_port_.FlushIOBuffers();
}

bool ImuNode::PublishImuMessage(const double &acc_x, const double &acc_y, const double &acc_z,
                                const double &gyro_x, const double &gyro_y, const double &gyro_z,
                                const double &w, const double &x, const double &y, const double &z)
{
    const double orientation_covariance[9] = {DEGREES_TO_RADIANS(0.01), 0, 0,
                                              0, DEGREES_TO_RADIANS(0.01), 0,
                                              0, 0., DEGREES_TO_RADIANS(0.09)};
    const double angular_velocity_covariance[9] = {DEGREES_TO_RADIANS(0.0025), 0, 0,
                                                   0, DEGREES_TO_RADIANS(0.0025), 0,
                                                   0, 0, DEGREES_TO_RADIANS(0.0025)};
    const double linear_acceleration_covariance[9] = {0.0004, 0, 0,
                                                      0, 0.0004, 0,
                                                      0, 0, 0.0004};

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation.x = x;
    imu_msg.orientation.y = y;
    imu_msg.orientation.z = z;
    imu_msg.orientation.w = w;

    imu_msg.linear_acceleration.x = acc_x;
    imu_msg.linear_acceleration.y = acc_y;
    imu_msg.linear_acceleration.z = acc_z;

    imu_msg.angular_velocity.x = gyro_x;
    imu_msg.angular_velocity.y = gyro_y;
    imu_msg.angular_velocity.z = gyro_z;

    for (int i = 0; i < 9; i++)
    {
        imu_msg.orientation_covariance[i] = orientation_covariance[i];
    }

    for (int i = 0; i < 9; i++)
    {
        imu_msg.angular_velocity_covariance[i] = angular_velocity_covariance[i];
    }
    for (int i = 0; i < 9; i++)
    {
        imu_msg.linear_acceleration_covariance[i] = linear_acceleration_covariance[i];
    }

    pub_imu_.publish(imu_msg);

    return true;
}

void ImuNode::LoopRun()
{
    int16_t acc[3];
    //double acc_data[3]; // use publish imu
    Eigen::Vector3d acc_data;
    Eigen::Vector3d calibed_acc_data;
    int16_t gyro[3];
    Eigen::Vector3d gyro_data;
    Eigen::Vector3d calibed_gyro_data;
    int16_t mag[3];
    float eular[3];
    float quat[4];
    uint8_t id;

    imu_data_decode_init(); // init imu
    int buffer_size = 200;
    uint8_t buffer_temp[buffer_size];
    memset(buffer_temp, 0, buffer_size);
    uint8_t *ptr = &buffer_temp[0];

    uint8_t ms_timeout = 0;
    ros::Rate pub_hz(publish_rate_);
    while (ros::ok())
    {
        try
        {
            imu_serial_port_.ReadByte(*ptr, ms_timeout);
            //printf(" %x ", *ptr);
            //ROS_ERROR(" ,%x ", *ptr);
        }
        catch (ReadTimeout)
        {
            ROS_ERROR(" Timeout of uart read imu data !!!!!! ");
            continue;
        }

        Packet_Decode(*ptr);
        if (flag_one_frame) // a frame data
        {
            flag_one_frame = false;
            static ros::Time start_time = ros::Time::now();

            get_raw_acc(acc);
            get_raw_gyo(gyro);
            get_raw_mag(mag);
            get_eular(eular);
            get_quat(quat);
            get_id(&id);

            for (int i = 0; i < 3; i++)
            {
                acc_data[i] = acc[i] * 0.001f * GRAVITY; // to g (m/s^2)
                gyro_data[i] = gyro[i] * 0.1f * M_PI / 180.0f;
            }

            if(compensate_with_calib_)
            {
                calibed_acc_data = acc_calib_.unbiasNormalize(acc_data);
                calibed_gyro_data = gyro_calib_.unbiasNormalize(gyro_data);
            }

            if (debug_raw_data_)
            {
                std::cout << " acc_data: " << acc_data[0] << " " << acc_data[1] << " " << acc_data[2] << std::endl;
                std::cout << " gyro_data: " << gyro_data[0] << " " << gyro_data[1] << " " << gyro_data[2] << std::endl;
            }

            ros::Time now = ros::Time::now();
            if (enable_record_)
            {
                imu_data_record_ << (now - start_time).toSec() << "  " << (now - start_time).toNSec() << "  " << acc_data[0] << "  " << acc_data[1] << "  " << acc_data[2] << "  " << gyro_data[0] << "  " << gyro_data[1] << "  " << gyro_data[2] << std::endl;
            }

            PublishImuMessage(acc_data[0], acc_data[1], acc_data[2], gyro_data[0], gyro_data[1], gyro_data[2], quat[0], quat[1], quat[2], quat[3]);

            memset(&buffer_temp[0], 0, buffer_size);
            ptr = &buffer_temp[0];
            
            pub_hz.sleep();
            //continue;
        }
        ptr++;
        ros::spinOnce();
        //pub_hz.sleep();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_node");

    ImuNode imu_node;
    imu_node.LoopRun();

    ros::spin();
    return 0;
}
