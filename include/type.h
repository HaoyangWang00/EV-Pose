//
// Created by mpl on 23-4-5.
//

#ifndef EVPose_TYPE_H
#define EVPose_TYPE_H

#include <memory>
#include <vector>
#include <Eigen/Core>

namespace EVPose
{

struct ImuMsg
{
    typedef std::shared_ptr<ImuMsg> Ptr;
    typedef std::shared_ptr<ImuMsg const> ConstPtr;

    ImuMsg(const double ts, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr):ts(ts), acc(acc), gyr(gyr)
    {}

    double ts;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
};

struct IMUData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<IMUData> Ptr;
    typedef std::shared_ptr<IMUData const> ConstPtr;
    IMUData() = default;

    IMUData(const double &gx, const double &gy, const double &gz,
            const double &ax, const double &ay, const double &az,
            const double &t) : gyroscope_(gx, gy, gz), accelerometer_(ax, ay, az), timestamp_(t) {}

    //IMU raw data
    Eigen::Vector3d gyroscope_;
    Eigen::Vector3d accelerometer_;

    //Data timestamp
    double timestamp_;

    //IMU sensor parameters, no need to set multiple times
    //IMU sensor continuous Gaussian white noise
    static double sigma_gyroscope_;
    static double sigma_accelerometer_;

    //IMU sensor continous random walk
    static double sigma_gyroscope_walk_;
    static double sigma_accelerometer_walk_;

    //IMU sensor data collection interval
    static double delta_timestamp_;

    //For the correspondences between discrete and continuous parameters, see the introduction of IMU model
    //Square of discrete random walk of IMU sensor
    static double gyroscope_noise_rw2_;
    static double accelerometer_noise_rw2_;

    //Square of discrete noise of IMU data
    static double gyroscope_bias_rw2_;
    static double accelerometer_bias_rw2_;

    //Noise matrix
    static Eigen::Matrix3d gyroscope_meas_covariance_;
    static Eigen::Matrix3d accelerometer_meas_covariance_;

    //random walk matrix
    static Eigen::Matrix3d gyroscope_bias_rw_covariance_;
    static Eigen::Matrix3d accelerometer_bias_rw_covariance_;

};

struct EventMsg
{
    typedef std::shared_ptr<EventMsg> Ptr;
    typedef std::shared_ptr<EventMsg const> ConstPtr;

    EventMsg(double ts, size_t x, size_t y, bool p):ts(ts), x(x), y(y), p(p)
    {}

    double ts;
    size_t x, y;
    bool p;
};


struct FrameData
{
    FrameData(std::vector<EventMsg::ConstPtr> event,  double ts):
            eventData(std::move(event)), stamp(std::move(ts))
    {}

    std::vector<EventMsg::ConstPtr> eventData;
    double stamp;
};


struct Point {

    typedef std::shared_ptr<Point> Ptr;
    typedef std::shared_ptr<Point const> ConstPtr;

    Point(double x, double y, double z, double xg, double yg, double zg):
            x(x), y(y), z(z), xg(xg), yg(yg), zg(zg)
    {};

    double x;
    double y;
    double z;

    double xg;//3D gradient
    double yg;
    double zg;
};

typedef std::shared_ptr<std::vector<Point>> pCloud; 

}

#endif //EVPose_TYPE_H
