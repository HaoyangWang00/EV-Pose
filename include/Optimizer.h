//
// Created by mpl on 23-3-15.
//

#ifndef EVPose_OPTIMIZER_H
#define EVPose_OPTIMIZER_H

#include "EventCamera.h"
#include "Frame.h"
#include "type.h"
#include "CamBasedProblemLM.h"
#include <unsupported/Eigen/NumericalDiff>
#include "utility"
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include "IMUPreintegrator.h"
namespace EVPose
{

class Optimizer
{

public:
    typedef std::shared_ptr<Optimizer> Ptr;

    Optimizer(std::string configPath, EventCamera::Ptr eventCam);

public:

    bool OptimizeEventProblem(TimeSurface::Ptr ts, pCloud cloud, const Eigen::Matrix4d& Tinit, const Eigen::Matrix4d& Twl,
                              Eigen::Matrix4d& result, Eigen::VectorXd& ix);


    void OptimizeOneFrame(Frame::Ptr f, std::vector<Point::Ptr> points);
    void SamplePointCloud(const std::vector<Point::Ptr>& src, std::vector<Point::Ptr>& dest, int sampleN = 500);


protected:
    EventCamera::Ptr mEventCam; 

    CamBasedProblemLM::Ptr mEventProblem;
    CamBasedProblemConfig::Ptr mCamBasedProblemConfig;
    // for projection
    int mPatchSizeX, mPatchSizeY;
    int mPointCulling;
    int mNormalFlow;
    int mVisualize;
    double mzMin;
    double mzMax;
};


}



#endif //EVPose_OPTIMIZER_H
