#include <IMUPreintegrator.h>

using namespace std;
using namespace EVPose;


IMUPreintegrator::IMUPreintegrator() {

    delta_p_.setZero();
    delta_v_.setZero();
    delta_rot_.setIdentity();

    jacobian_v_bias_gyroscope_.setZero();
    jacobian_v_bias_accelerometer_.setZero();

    jacobian_p_bias_gyroscope_.setZero();
    jacobian_p_bias_accelerometer_.setZero();

    jacobian_rot_bias_gyroscope_.setZero();

    cov_p_v_rot_.setZero();

    delta_time_ = 0.0;
    //T_cam_imu<<1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
    T_cam_imu << 0.017248643674008135, -0.9998037138739959, 0.009747718459772736, 0.07733078169916466,
            0.012834636469124028, -0.009526963092989282, -0.999872246379971, -0.016637889364465353,
            0.9997688514842376, 0.017371548520172697, 0.01266779001636642, -0.14481844113148515,
            0, 0, 0, 1;

}


void
IMUPreintegrator::Update(const Eigen::Vector3d &omega, const Eigen::Vector3d &acc, const double &delta_timestamp_) {
    double dt2 = delta_timestamp_ * delta_timestamp_;

    //Step-1: Calculate IMU pre-integral rotation increment and Jacobian
    Eigen::Matrix3d dR = Sophus::SO3::exp(omega * delta_timestamp_).matrix();

     //adding svd to be more stable;
    //std::cout<<"-----imu-----"<<std::endl;
    //std::cout<<Utility::rot2cayley(dR)<<std::endl;
    Eigen::Matrix3d newR = T_cam_imu.block(0, 0, 3, 3)* dR;
    //Eigen::JacobiSVD<Eigen::Matrix3d> svd(newR, Eigen::ComputeFullU | Eigen::ComputeFullV);
    //dR = svd.matrixU() * svd.matrixV().transpose();
    //std::cout<<Utility::rot2cayley(dR)<<std::endl;

    Eigen::Matrix3d Jr_R = Sophus::SO3::JacobianR(omega * delta_timestamp_);

    //Step 2: Calculate the uncertainity variance of the noise error of the IMU pre-integrator observation equation
    Eigen::Matrix3d I3x3;
    I3x3.setIdentity();

    Matrix9d A;
    A.setIdentity();
    //  The block size is 3x3, the starting position (6,6)
    A.block<3, 3>(6, 6) = dR.transpose();
    A.block<3, 3>(3, 6) = -delta_rot_ * Sophus::SO3::hat(acc) * delta_timestamp_;
    A.block<3, 3>(0, 6) = -0.5 * delta_rot_ * Sophus::SO3::hat(acc) * dt2;
    A.block<3, 3>(0, 3) = I3x3 * delta_timestamp_;

    //Split the B in the paper into two parts
    Eigen::Matrix<double, 9, 3> Bg;
    Bg.setZero();
    Bg.block<3, 3>(6, 0) = Jr_R * delta_timestamp_;

    Eigen::Matrix<double, 9, 3> Ca;
    Ca.setZero();
    Ca.block<3, 3>(3, 0) = delta_rot_ * delta_timestamp_;
    Ca.block<3, 3>(0, 0) = 0.5 * delta_rot_ * dt2;

    cov_p_v_rot_ = A * cov_p_v_rot_ * A.transpose() +
                   Bg * IMUData::gyroscope_meas_covariance_ * Bg.transpose() +
                   Ca * IMUData::accelerometer_meas_covariance_ * Ca.transpose();

    //Step-3: Calculate Jacobian, used to correct the biased IMU pre-integrated measurement value

    //Recursion based on supplementary material formula
    jacobian_p_bias_accelerometer_ += jacobian_v_bias_accelerometer_ * delta_timestamp_ - 0.5 * delta_rot_ * dt2;
    jacobian_p_bias_gyroscope_ += jacobian_v_bias_gyroscope_ * delta_timestamp_ -
                                  0.5 * delta_rot_ * Sophus::SO3::hat(acc) * jacobian_rot_bias_gyroscope_ * dt2;

    jacobian_v_bias_accelerometer_ += -delta_rot_ * delta_timestamp_;
    jacobian_v_bias_gyroscope_ += -delta_rot_ * Sophus::SO3::hat(acc) * jacobian_rot_bias_gyroscope_ * delta_timestamp_;

    jacobian_rot_bias_gyroscope_ = dR.transpose() * jacobian_rot_bias_gyroscope_ - Jr_R * delta_timestamp_;

    //Step-4: Calculate IMU pre-integrated measurement value
    //delta_rot_ = T_cam_imu.block(0,0,3,3) * delta_rot_;
    //Eigen::JacobiSVD<Eigen::Matrix3d> svd(T_cam_imu.block(0,0,3,3), Eigen::ComputeFullU | Eigen::ComputeFullV);
    //delta_rot_ = svd.matrixU() * svd.matrixV().transpose();
    delta_p_ += delta_v_ * delta_timestamp_ + 0.5 * delta_rot_ * acc * dt2;
    delta_v_ += delta_rot_ * acc * delta_timestamp_;
    delta_rot_ = NormalizedRotationM(delta_rot_ * dR);
    delta_time_ += delta_timestamp_;
}


void IMUPreintegrator::reset() {
    delta_p_.setZero();
    delta_v_.setZero();
    delta_rot_.setIdentity();

    jacobian_v_bias_gyroscope_.setZero();
    jacobian_v_bias_accelerometer_.setZero();

    jacobian_p_bias_gyroscope_.setZero();
    jacobian_p_bias_accelerometer_.setZero();

    jacobian_rot_bias_gyroscope_.setZero();

    cov_p_v_rot_.setZero();

    delta_time_ = 0.0;
}
