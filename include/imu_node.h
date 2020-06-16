#pragma once

#include <SerialPort.h>
#include <string>
#include "ros/ros.h"
#include "std_srvs/SetBool.h"

#include "imu_data_decode.h"
#include "packet.h"
#include <Eigen/Core>

#include <fstream>

template <typename _T>
class CalibData
{
public:
    CalibData();
    bool Init(const _T &mis_yz = _T(0), const _T &mis_zy = _T(0), const _T &mis_zx = _T(0),
              const _T &mis_xz = _T(0), const _T &mis_xy = _T(0), const _T &mis_yx = _T(0),
              const _T &s_x = _T(1), const _T &s_y = _T(1), const _T &s_z = _T(1),
              const _T &b_x = _T(0), const _T &b_y = _T(0), const _T &b_z = _T(0));

    bool load(std::string filename);

    inline Eigen::Matrix<_T, 3, 1> unbiasNormalize(const Eigen::Matrix<_T, 3, 1> &raw_data) const
    {
        return ms_mat_ * (raw_data - bias_vec_);
    };

private:
    void update();
    Eigen::Matrix<_T, 3, 3> mis_mat_;
    Eigen::Matrix<_T, 3, 3> scale_mat_;
    Eigen::Matrix<_T, 3, 1> bias_vec_;

    Eigen::Matrix<_T, 3, 3> ms_mat_;
};

class ImuNode
{
public:
    ImuNode();
    ~ImuNode();

    void Init();
    void LoopRun();
    bool PublishImuMessage(const double &acc_x, const double &acc_y, const double &acc_z,
                           const double &gyro_x, const double &gyro_y, const double &gyro_z,
                           const double &w, const double &x, const double &y, const double &z);

    bool RecordImu(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

private:
    std::string imu_topic_;
    std::string imu_frame_;

    LibSerial::SerialPort imu_serial_port_;
    std::string serial_port_name_;

    std::ofstream imu_data_record_;
    std::string imu_data_record_path_;

    ros::Publisher pub_imu_;
    int publish_rate_;

    bool enable_record_;
    ros::ServiceServer record_control_;

    bool debug_raw_data_;
    bool compensate_with_calib_;
    CalibData<double>  acc_calib_;
    CalibData<double>  gyro_calib_;
    std::string acc_calib_file_;
    std::string gyro_calib_file_;
};
