//
// Created by mpl on 23-3-9.
//

#ifndef CANNY_EVIT_FRAME_H
#define CANNY_EVIT_FRAME_H

#include <vector>
#include <Eigen/Core>
#include "TimeSurface.h"
#include "EventCamera.h"

namespace EVPose {

class Frame {

public:
    typedef std::shared_ptr<Frame> Ptr;
    typedef std::shared_ptr<Frame const> ConstPtr;

    Frame(TimeSurface::Ptr Ts,  double stamp, EventCamera::Ptr eventCam);

public:
    double mStamp;

    EventCamera::Ptr mEventCam;
    TimeSurface::Ptr mTs;

   Eigen::Matrix4d T;
   Eigen::VectorXd ix;



};



}

#endif //CANNY_EVIT_FRAME_H
