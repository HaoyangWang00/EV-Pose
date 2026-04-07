#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>

static std::ofstream file("/home/lxy/event_camera/EVPose/result.txt",  std::ios::out | std::ios::trunc);

#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include "GenericFunctor.h"


    struct Pose {
        Eigen::Matrix4d matrix;
        double timestamp;
    };
    using namespace Eigen;
    struct TrajectoryErrorFunctor {
        typedef double Scalar;
        enum { InputsAtCompileTime = Dynamic, ValuesAtCompileTime = Dynamic };
        typedef Matrix<Scalar, Dynamic, 1> InputType;
        typedef Matrix<Scalar, Dynamic, 1> ValueType;
        typedef Matrix<Scalar, Dynamic, Dynamic> JacobianType;

        //const vector<Vector3d>& positions;
        //const vector<Vector3d>& euler_angles;
        int window_size;

        TrajectoryErrorFunctor(int win_size)
                : window_size(win_size) {}

        int inputs() const { return window_size * 6; }
        int values() const { return window_size * 6; }

        int operator()(const InputType& x, ValueType& fvec) const {
            fvec.resize(inputs());
            fvec.setZero();
            for (int j = 1; j < window_size - 1; ++j) {
                fvec.segment<3>((j - 1) * 3 + 0) = x.segment<3>((j + 1) * 6 + 0) - 2 * x.segment<3>(j * 6 + 0) + x.segment<3>((j - 1) * 6 + 0);
                //fvec.segment<3>((j - 1) * 6 + 3) = x.segment<3>((j + 1) * 6 + 3) - 2 * x.segment<3>(j * 6 + 3) + x.segment<3>((j - 1) * 6 + 3);
            }
            //std::cout<<fvec;
            return 0;
        }
    };

    class SlidingWindowPoseOptimizer {
    public:

        SlidingWindowPoseOptimizer(int window_size, int polynomial_order)
                : window_size_(window_size) {
        }

        void addPose(const Eigen::Matrix4d &matrix, double timestamp) {
            poses_.push_back({matrix, timestamp});
            if (poses_.size() > window_size_) {
                poses_.erase(poses_.begin()); // Remove the oldest pose
            }
            optimize(); // Perform optimization after adding a pose
        }

        void optimize() {
            if (poses_.size() < window_size_) {
                std::cerr << "Not enough poses to perform optimization" << std::endl;
                return;
            }

            // Set up optimization variables
            int num_params = poses_.size() * 6; // 6 DOF per pose
            Eigen::VectorXd params = Eigen::VectorXd::Zero(num_params);
            //Eigen::Matrix<double, 30, 1> params;
            // Initialize optimization variables from poses
            for (int i = 0; i < poses_.size(); ++i) {
                params.segment<3>(i * 6 + 0) = poses_[i].matrix.block<3, 1>(0, 3); // Translation
                Eigen::Vector3d euler_angles = poses_[i].matrix.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
                params.segment<3>(i * 6 + 3) = euler_angles; // Euler angles
            }

            TrajectoryErrorFunctor functor(window_size_);
            NumericalDiff<TrajectoryErrorFunctor> numDiff(functor);
            LevenbergMarquardt<NumericalDiff<TrajectoryErrorFunctor>> lm(numDiff);

            lm.parameters.ftol = 1e-4;
            lm.parameters.xtol = 1e-4;
            lm.parameters.maxfev = 1000;
            lm.parameters.factor = 10.0;

            int iteration = 0;
            std::cout << "Start optimization\n";
            while (true) {
                lm.minimizeInit(params);
                //lm.minimizeInit(Eigen::VectorXd::Zero(num_params));
                //std::cout << params << std::endl;

                //params.resize(30,1);
                //Eigen::Matrix<double,-1,1> p_=params.cast<double>();
                Eigen::LevenbergMarquardtSpace::Status status = lm.minimizeOneStep(params);
                std::cout << "Iteration: " << iteration++ << ", Status: " << status << std::endl;
                if (status == Eigen::LevenbergMarquardtSpace::ImproperInputParameters ||
                    status == Eigen::LevenbergMarquardtSpace::RelativeReductionTooSmall) {
                    std::cout << "Terminating due to improper input parameters or small diagonal elements."
                              << std::endl;
                    if (status == Eigen::LevenbergMarquardtSpace::RelativeReductionTooSmall) {
                        std::cout << "RelativeReductionTooSmall\n";
                    }
                    if (status == Eigen::LevenbergMarquardtSpace::ImproperInputParameters) {
                        std::cout << "ImproperInputParameters\n";
                    }
                    break;
                }
                if (iteration >= 20) {
                    break;
                }
            }

            //std::cout << "Final optimized params:\n" << params << std::endl;

            // Update poses with optimized values
            for (int i = 0; i < poses_.size(); ++i) {
                poses_[i].matrix.block<3, 1>(0, 3) = params.segment<3>(i * 6 + 0); // Update translation
                //Eigen::Matrix3d rotation;
                //rotation = Eigen::AngleAxisd(params[i * 6 + 3], Eigen::Vector3d::UnitX())
                //           * Eigen::AngleAxisd(params[i * 6 + 4], Eigen::Vector3d::UnitY())
                //           * Eigen::AngleAxisd(params[i * 6 + 5], Eigen::Vector3d::UnitZ());
                //poses_[i].matrix.block<3, 3>(0, 0) = rotation; // Update rotation
            }
            savePose(poses_[0]); // Save the updated pose
        }

        void savePose(const Pose &pose) {
            Eigen::Quaterniond q(pose.matrix.block<3, 3>(0, 0));
            std::string str = std::to_string(pose.timestamp);
            str = str.substr(0, str.find('.') + 7);
            file << str << " " << pose.matrix.block<3, 1>(0, 3)(0)
                 << " " << pose.matrix.block<3, 1>(0, 3)(1)
                 << " " << pose.matrix.block<3, 1>(0, 3)(2)
                 << " " << q.coeffs().x()
                 << " " << q.coeffs().y()
                 << " " << q.coeffs().z()
                 << " " << q.coeffs().w()
                 << std::endl;
        }

        const std::vector<Pose> &getPoses() const {
            return poses_;
        }

        typedef std::shared_ptr<SlidingWindowPoseOptimizer> Ptr;

    private:
        std::vector<Pose> poses_;
        int window_size_;
    };

