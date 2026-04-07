//
// Created by mpl on 23-4-3.
//

#include "CamBasedProblemLM.h"
#include "utility.h"

#include <stdlib.h>
#include <thread>
#include <cmath>

using namespace EVPose;

CamBasedProblemLM::CamBasedProblemLM(CamBasedProblemConfig::Ptr config, EventCamera::Ptr EventCam)
:GenericFunctor<double>(6, 0), mConfig(config), mEventCamera(EventCam), mCounter(0), mNumBatches(0)
{
    mPatchSize = mConfig->patchSize_X_ * mConfig->patchSize_Y_;
    computeJ_G(Eigen::Matrix<double, 6, 1>::Zero(), mJ_G_0);
}

void CamBasedProblemLM::setConfig(CamBasedProblemConfig::Ptr config)
{
    mConfig = config;
}

void CamBasedProblemLM::setStochasticSampling(size_t offset, size_t N)
{
    mResItemsStochSampled.clear();
    mResItemsStochSampled.reserve(N); 
    for (size_t i = 0; i < N; i++) {
        if (offset + i >= mResItems.size())
            break;
        mResItemsStochSampled.push_back(mResItems[offset + i]);
    }
    mNumPoints = mResItemsStochSampled.size();
    resetNumberValues(mNumPoints * mPatchSize);

    if (mConfig->debug) {
        std::cout << "offset: " << offset << std::endl;
        std::cout << "N: " << N << std::endl;
        std::cout << "ResItems_.size: " << mResItems.size() << std::endl;
        std::cout << "ResItemsStochSampled_.size: " << mResItemsStochSampled.size()<< std::endl;
    }
}

void CamBasedProblemLM::setProblem(TimeSurface::Ptr Ts, pCloud cloud, Eigen::Matrix4d Twc, Eigen::Matrix4d Twl, Eigen::VectorXd ix)
{
    // std::cout<<"------- problem init -------"<<std::endl;
    mTs = Ts;
    mCloud = cloud;
    mRwc = Twc.block<3, 3>(0, 0);
    mtwc = Twc.block<3, 1>(0, 3);

    int numPoints = mCloud->size();
    if (numPoints >  mConfig->MAX_REGISTRATION_POINTS_)
        numPoints = mConfig->MAX_REGISTRATION_POINTS_;
    mResItems.clear();
    mResItems.resize(numPoints);
    mNumPoints = numPoints;

    Eigen::Matrix3d Rcw = Twc.block<3, 3>(0, 0).transpose();
    Eigen::Vector3d tcw = -Rcw * Twc.block<3, 1>(0, 3);
    Eigen::Matrix3d Rlw = Twl.block<3, 3>(0, 0).transpose();
    Eigen::Vector3d tlw = -Rlw * Twl.block<3, 1>(0, 3);

    for (int i = 0; i < numPoints; i++)
    {
        if (mConfig->StochasticSampling)
            std::swap(mCloud->at(i), mCloud->at(i + rand() % (mCloud->size() - i)));
        
        Eigen::Vector3d p_w(mCloud->at(i).x, mCloud->at(i).y, mCloud->at(i).z);
        Eigen::Vector3d pg_w(mCloud->at(i).xg, mCloud->at(i).yg, mCloud->at(i).zg); 

        Eigen::Vector3d p_c = Rcw * p_w + tcw;
        Eigen::Vector3d pg_c = Rcw * pg_w;
        Eigen::Vector3d p_l = Rlw * p_w + tlw;
        
        Eigen::Vector2d x_c = mEventCamera->World2Cam(p_c);
        Eigen::Vector2d xg_c = mEventCamera->World2Cam(pg_c);
        Eigen::Vector2d x_l = mEventCamera->World2Cam(p_l);

        Eigen::Vector2d gradFlow = x_c - xg_c;       
        Eigen::Vector2d opticalFlow = x_l - x_c;     

        mResItems[i].initialize(p_w(0), p_w(1), p_w(2), pg_w(0), pg_w(1), pg_w(2));
        mResItems[i].grad_ = gradFlow;
        mResItems[i].optical_flow_ = opticalFlow;
        mResItems[i].weight_ = 1;

        if(mConfig->usingPointCulling)
        {
            if (mCounter < 500) continue;

            int colSize = mTs->getTsAnnfPositiveCol().cols;
            int rowSize = mTs->getTsAnnfPositiveCol().rows;
            Eigen::MatrixXd depth_matrix_positive(rowSize, colSize);
            Eigen::MatrixXd index_matrix_positive(rowSize, colSize);
            Eigen::MatrixXd depth_matrix_negative(rowSize, colSize);
            Eigen::MatrixXd index_matrix_negative(rowSize, colSize);
            Eigen::MatrixXd depth_matrix_neutral(rowSize, colSize);
            Eigen::MatrixXd index_matrix_neutral(rowSize, colSize);
            
            depth_matrix_positive.fill(-1); index_matrix_positive.fill(-1);
            depth_matrix_negative.fill(-1); index_matrix_negative.fill(-1);
            depth_matrix_neutral.fill(-1);  index_matrix_neutral.fill(-1);

            double cosTheta = 0.0;
            if (isValidFlow(gradFlow) && isValidFlow(opticalFlow)) {
                cosTheta = gradFlow.dot(opticalFlow) / (gradFlow.norm() * opticalFlow.norm());
            }
            
            double absCosTheta = std::abs(cosTheta);
            
            double thresh_a = 0.15; 
            double thresh_b = 0.30; 

            bool pass_filtering = true;
            int prePolarity = 0;

            if (absCosTheta < thresh_a || std::isnan(cosTheta)) {
                pass_filtering = false; // Filter out all events
            } 
            else 
            {
                if (absCosTheta < thresh_b) {
                    prePolarity = 0; 
                } else {
                    prePolarity = cosTheta > 0 ? 1 : -1; 
                }
            }

            mResItems[i].polarity_ = prePolarity;

            int row = -1, col = -1;
            if (pass_filtering) {
                if (prePolarity == 1) {
                    col = mTs->getTsAnnfPositiveCol().at<double>(x_c(1), x_c(0));
                    row = mTs->getTsAnnfPositiveRow().at<double>(x_c(1), x_c(0));
                } else if (prePolarity == -1) {
                    col = mTs->getTsAnnfNegativeCol().at<double>(x_c(1), x_c(0));
                    row = mTs->getTsAnnfNegativeRow().at<double>(x_c(1), x_c(0));
                } else {
                    col = mTs->getTsAnnfCol().at<double>(x_c(1), x_c(0));
                    row = mTs->getTsAnnfRow().at<double>(x_c(1), x_c(0));
                }

                if (col != -1 && row != -1) {
                    double dist_to_event = std::sqrt(std::pow(x_c(0) - col, 2) + std::pow(x_c(1) - row, 2));
                    if (dist_to_event > 10.0) pass_filtering = false;
                } else {
                    pass_filtering = false;
                }
            }

            if (pass_filtering) {
                int px = std::round(x_c(0));
                int py = std::round(x_c(1));
                if (px >= 0 && px < mEventCamera->getWidth() && py >= 0 && py < mEventCamera->getHeight()) {
                    Eigen::Vector2d ts_grad;
                    if (prePolarity == 1) {
                        ts_grad(0) = mTs->mInverseTsPositiveGradMatXE.at<double>(py, px);
                        ts_grad(1) = mTs->mInverseTsPositiveGradMatYE.at<double>(py, px);
                    } else if (prePolarity == -1) {
                        ts_grad(0) = mTs->mInverseTsNegativeGradMatXE.at<double>(py, px);
                        ts_grad(1) = mTs->mInverseTsNegativeGradMatYE.at<double>(py, px);
                    } else {
                        ts_grad(0) = mTs->mInverseTsGradMatXE.at<double>(py, px);
                        ts_grad(1) = mTs->mInverseTsGradMatYE.at<double>(py, px);
                    }
                    
                    if (ts_grad.norm() > 1e-4 && gradFlow.norm() > 1e-4) {
                        double cos_structure = std::abs(ts_grad.dot(gradFlow) / (ts_grad.norm() * gradFlow.norm()));
                        if (cos_structure < 0.5) pass_filtering = false; 
                    }
                }
            }

            if (!pass_filtering) {
                mResItems[i].weight_ = 0;
            } else {
                if (prePolarity == 1) pointCulling(col, row, depth_matrix_positive, index_matrix_positive, p_c(2), i);
                else if (prePolarity == -1) pointCulling(col, row, depth_matrix_negative, index_matrix_negative, p_c(2), i);
                else pointCulling(col, row, depth_matrix_neutral, index_matrix_neutral, p_c(2), i);
            }
        }
    }

    mNumBatches = std::max(mResItems.size() / mConfig->BATCH_SIZE_, size_t(1));
    resetNumberValues(numPoints * mPatchSize);
    mCounter++;
}

void CamBasedProblemLM::pointCulling(int col, int row, Eigen::MatrixXd& depthMatrix, Eigen::MatrixXd& indexMatrix, double depth, int index)
{
    if (col == -1 || row == -1)
        mResItems[index].weight_ = 0;
    else if(depthMatrix(row, col) == -1)
    {
        depthMatrix(row, col) = depth;            
        indexMatrix(row, col) = index;            
    }
    else if(depthMatrix(row, col) > depth)
    {
        mResItems[indexMatrix(row, col)].weight_ = 0;
        depthMatrix(row, col) = depth;            
        indexMatrix(row, col) = index;            
    }
}

Eigen::Matrix4d CamBasedProblemLM::getPose()
{
    Eigen::Matrix4d ret = Eigen::Matrix4d::Identity();
    ret.block<3,3>(0, 0) = mRwc;
    ret.block<3,1>(0, 3) = mtwc;
    return ret;
}

void CamBasedProblemLM::addMotionUpdate(const Eigen::Matrix<double, 6, 1> &dx)
{
    Eigen::Vector3d dc = dx.block<3, 1>(0, 0);
    Eigen::Vector3d dt = dx.block<3, 1>(3, 0);
    Eigen::Matrix3d dR = Utility::cayley2rot(dc);
    Eigen::Matrix3d newR = dR * mRwc;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(newR, Eigen::ComputeFullU | Eigen::ComputeFullV);
    mRwc = svd.matrixU() * svd.matrixV().transpose();
    mtwc = dt + dR * mtwc;
}

int CamBasedProblemLM::operator()(const Eigen::Matrix<double, 6, 1> &x, Eigen::VectorXd &fvec) const
{
    Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();
    getWarpingTransformation(Tcw, x);

    std::vector<Job> jobs(mConfig->NUM_THREAD);
    for (size_t i = 0; i < mConfig->NUM_THREAD; i++) {
        jobs[i].pvRI = const_cast<ResidualItems *>(&mResItemsStochSampled);
        jobs[i].pTs = mTs;
        jobs[i].Tcw = const_cast<Eigen::Matrix4d *>(&Tcw);
        jobs[i].threadID = i;
    }

    std::vector<std::thread> threads;
    for (size_t i = 0; i < mConfig->NUM_THREAD; i++)
        threads.emplace_back(std::bind(&CamBasedProblemLM::thread, this, jobs[i]));
    for (auto &thread : threads)
        if (thread.joinable())
            thread.join();

    if (mConfig->LSnorm_ == "l2")
    {
        for (size_t i = 0; i < mResItemsStochSampled.size(); i++) {
            ResidualItem &ri = const_cast<ResidualItem &>(mResItemsStochSampled[i]);
            fvec.segment(i * ri.residual_.size(), ri.residual_.size()) = ri.residual_; 
        }
    }
    else if (mConfig->LSnorm_ == "Huber")
    {
        for (size_t i = 0; i < mResItemsStochSampled.size(); i++) {
            ResidualItem &ri = const_cast<ResidualItem &>(mResItemsStochSampled[i]);
            double irls_weight = 1.0;
            if (ri.residual_(0) > mConfig->huber_threshold_)
                irls_weight = mConfig->huber_threshold_ / ri.residual_(0);
            fvec[i] = sqrt(irls_weight) * ri.residual_(0);
        }
    }

    return 0;
}


int CamBasedProblemLM::df(const Eigen::Matrix<double, 6, 1> &x, Eigen::MatrixXd &fjac) const
{
    if (x != Eigen::Matrix<double, 6, 1>::Zero()) {
        std::cerr << "The Jacobian is not evaluated at Zero !!!!!!!!!!!!!";
        exit(-1);
    }
    fjac.resize(m_values, 6);

    Eigen::Matrix3d dT_dInvPi = mRwc.transpose(); 
    Eigen::Matrix<double, 3, 2> dInvPi_dx_constPart;
    dInvPi_dx_constPart.setZero();
    dInvPi_dx_constPart(0, 0) = 1.0 / mEventCamera->getProjectionMatrix()(0, 0);
    dInvPi_dx_constPart(1, 1) = 1.0 / mEventCamera->getProjectionMatrix()(1, 1);
    Eigen::Matrix<double, 3, 2> J_constPart = dT_dInvPi * dInvPi_dx_constPart;

    Eigen::MatrixXd fjacBlock;
    fjacBlock.resize(mNumPoints, 12);
    Eigen::Matrix4d Tcw = Eigen::Matrix4d::Identity();
    Tcw.block<3, 3>(0, 0) = mRwc.transpose();
    Tcw.block<3, 1>(0, 3) = -mRwc.transpose() * mtwc;

    const double P11 = mEventCamera->getProjectionMatrix()(0, 0);
    const double P12 = mEventCamera->getProjectionMatrix()(0, 1);
    const double P14 = mEventCamera->getProjectionMatrix()(0, 3);
    const double P21 = mEventCamera->getProjectionMatrix()(1, 0);
    const double P22 = mEventCamera->getProjectionMatrix()(1, 1);
    const double P24 = mEventCamera->getProjectionMatrix()(1, 3);

    for (size_t i = 0; i < mNumPoints; i++) {
        const ResidualItem &ri = mResItemsStochSampled[i];
        Eigen::Vector3d pw = ri.p_;
        Eigen::Vector3d pc = Tcw.block<3,3>(0, 0) * pw + Tcw.block<3,1>(0, 3);
        Eigen::Vector2d xc = mEventCamera->World2Cam(pc);

        if (!isValidPatch(xc, mEventCamera->getUndistortRectifyMask(), mConfig->patchSize_X_, mConfig->patchSize_Y_)) {
            fjacBlock.row(i) = Eigen::Matrix<double, 1, 12>::Zero();
        }
        else
        {
            Eigen::MatrixXd gx, gy;
            Eigen::Vector2d grad;
            if(mConfig->usingNormalFlow)
            {
                if(ri.polarity_>0){
                    patchInterpolation(mTs->mInverseTsPositiveGradMatXE, xc, gx, true);
                    patchInterpolation(mTs->mInverseTsPositiveGradMatYE, xc, gy, true);
                    grad = Eigen::Vector2d(gx(0, 0) / 8, gy(0, 0) / 8); 
                }
                else if(ri.polarity_<0){
                    patchInterpolation(mTs->mInverseTsNegativeGradMatXE, xc, gx, true);
                    patchInterpolation(mTs->mInverseTsNegativeGradMatYE, xc, gy, true);
                    grad = Eigen::Vector2d(gx(0, 0) / 8, gy(0, 0) / 8); 
                }
                else{
                    patchInterpolation(mTs->mInverseTsGradMatXE, xc, gx, true);
                    patchInterpolation(mTs->mInverseTsGradMatYE, xc, gy, true);
                    grad = Eigen::Vector2d(gx(0, 0) / 8, gy(0, 0) / 8); 
                }
            }
            else
            {
                patchInterpolation(mTs->mInverseTsGradMatXE, xc, gx, true);
                patchInterpolation(mTs->mInverseTsGradMatYE, xc, gy, true);
                grad = Eigen::Vector2d(gx(0, 0) / 8, gy(0, 0) / 8); 
            }

            Eigen::Vector4d p4d;
            p4d.setIdentity();
            p4d.block<3,1>(0,0) = ri.p_;

            Eigen::Vector4d p3d = Tcw * p4d;
            Eigen::Matrix<double, 2, 3> dPi_dT;
            dPi_dT.setZero();
            dPi_dT.block<2, 2>(0, 0) = mEventCamera->getProjectionMatrix().block<2, 2>(0, 0) / p3d(2);
            const double z2 = pow(p3d(2), 2);
            dPi_dT(0, 2) = -(P11 * p3d(0) + P12 * p3d(1) + P14) / z2;
            dPi_dT(1, 2) = -(P21 * p3d(0) + P22 * p3d(1) + P24) / z2;

            Eigen::Matrix<double, 3, 12> dT_dG;
            dT_dG.setZero();
            dT_dG.block<3, 3>(0, 0) = p3d(0) * Eigen::Matrix3d::Identity();
            dT_dG.block<3, 3>(0, 3) = p3d(1) * Eigen::Matrix3d::Identity();
            dT_dG.block<3, 3>(0, 6) = p3d(2) * Eigen::Matrix3d::Identity();
            dT_dG.block<3, 3>(0, 9) = Eigen::Matrix3d::Identity();
            
            fjacBlock.row(i) = (grad).transpose() * dPi_dT * dT_dG;
            fjacBlock.row(i) = ri.weight_ * fjacBlock.row(i);
        }
    }
    
    fjac = -fjacBlock * mJ_G_0;
    return 0;
}

void CamBasedProblemLM::thread(Job &job) const
{
    ResidualItems &vRI = *job.pvRI;
    TimeSurface::ConstPtr TsObs = job.pTs;
    const Eigen::Matrix4d &Tcw = *job.Tcw;
    size_t threadID = job.threadID;
    size_t numPoint = vRI.size();

    size_t wx = mConfig->patchSize_X_;
    size_t wy = mConfig->patchSize_Y_;
    size_t residualDim = wx * wy;

    for (size_t i = threadID; i < numPoint; i += mConfig->NUM_THREAD)
    {
        ResidualItem &ri = vRI[i];
        ri.residual_ = Eigen::VectorXd(residualDim);

        Eigen::Vector3d pw = ri.p_;
        Eigen::Vector3d pc = Tcw.block<3,3>(0, 0) * pw + Tcw.block<3,1>(0, 3);
        Eigen::Vector2d xc = mEventCamera->World2Cam(pc);

        if (!isValidPatch(xc, mEventCamera->getUndistortRectifyMask(), mConfig->patchSize_X_, mConfig->patchSize_Y_)) {
            ri.residual_.setConstant(255.0);
        }
        else
        {
            Eigen::MatrixXd tau1;
            bool success = false;
            if(mConfig->usingNormalFlow)
            {
                if (ri.polarity_ == 1 )
                {
                    success = patchInterpolation(mTs->mInverseTsPositiveMatE, xc, tau1, true);
                }
                else if (ri.polarity_ == -1)
                {
                    success = patchInterpolation(mTs->mInverseTsNegativeMatE, xc, tau1, true);
                }
                else
                {
                    success = patchInterpolation(mTs->mInverseTsMatE, xc, tau1, true);
                }
            }
            else
            {
                success = patchInterpolation(mTs->mInverseTsMatE, xc, tau1, true);
            }

            if(success)
            {
                for (size_t y = 0; y < wy; y++)
                    for (size_t x = 0; x < wx; x++)
                    {
                        size_t index = y * wx + x;
                        ri.residual_[index] = tau1(y, x)  * ri.weight_; 
                    }
            }
            else
            {
                ri.residual_.setConstant(255.0);
            }
        }
    }
}

void CamBasedProblemLM::getWarpingTransformation(Eigen::Matrix4d &warpingTransf, const Eigen::Matrix<double, 6, 1> &x) const
{
    Eigen::Matrix3d Rcw;
    Eigen::Vector3d tcw;
    Eigen::Vector3d dc = x.block<3, 1>(0, 0);
    Eigen::Vector3d dt = x.block<3, 1>(3, 0);
    Eigen::Matrix3d dR = Utility::cayley2rot(dc);
    Eigen::Matrix3d newRcw = mRwc.transpose() * dR.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(newRcw, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Rcw = svd.matrixU() * svd.matrixV().transpose();
    if (Rcw.determinant() < 0.0) {
        std::cout << "oops the matrix is left-handed\n";
        exit(-1);
    }
    tcw = -Rcw * (dt + dR * mtwc);
    warpingTransf.block<3, 3>(0, 0) = Rcw;
    warpingTransf.block<3, 1>(0, 3) = tcw;
}

bool CamBasedProblemLM::patchInterpolation(const Eigen::MatrixXd &img, const Eigen::Vector2d &location,
                                        Eigen::MatrixXd &patch, bool debug) const
{
    int wx = mConfig->patchSize_X_;
    int wy = mConfig->patchSize_Y_;
    Eigen::Vector2i SrcPatch_UpLeft, SrcPatch_DownRight;
    SrcPatch_UpLeft << floor(location[0]) - (wx - 1) / 2, floor(location[1]) - (wy - 1) / 2;
    SrcPatch_DownRight << floor(location[0]) + (wx - 1) / 2, floor(location[1]) + (wy - 1) / 2;

    if (SrcPatch_UpLeft[0] < 0 || SrcPatch_UpLeft[1] < 0) return false;
    if (SrcPatch_DownRight[0] >= img.cols() || SrcPatch_DownRight[1] >= img.rows()) return false;

    Eigen::Vector2d double_indices;
    double_indices << location[1], location[0];

    std::pair<int, int> lower_indices(floor(double_indices[0]), floor(double_indices[1]));
    std::pair<int, int> upper_indices(lower_indices.first + 1, lower_indices.second + 1);

    double q1 = upper_indices.second - double_indices[1];
    double q2 = double_indices[1] - lower_indices.second;
    double q3 = upper_indices.first - double_indices[0];
    double q4 = double_indices[0] - lower_indices.first;

    int wx2 = wx + 1;
    int wy2 = wy + 1;
    if (SrcPatch_UpLeft[1] + wy >= img.rows() || SrcPatch_UpLeft[0] + wx >= img.cols()) return false;
    Eigen::MatrixXd SrcPatch = img.block(SrcPatch_UpLeft[1], SrcPatch_UpLeft[0], wy2, wx2);

    Eigen::MatrixXd R;
    R = q1 * SrcPatch.block(0, 0, wy2, wx) + q2 * SrcPatch.block(0, 1, wy2, wx);

    patch = q3 * R.block(0, 0, wy, wx) + q4 * R.block(1, 0, wy, wx);
    return true;
}

void CamBasedProblemLM::computeJ_G(const Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 12, 6> &J_G)
{
    assert(x.size() == 6);
    assert(J_G.rows() == 12 && J_G.cols() == 6);
    double c1, c2, c3, k;
    double c1_sq, c2_sq, c3_sq, k_sq;
    c1 = x(0); c2 = x(1); c3 = x(2);
    c1_sq = pow(c1, 2); c2_sq = pow(c2, 2); c3_sq = pow(c3, 2);
    k = 1 + pow(c1, 2) + pow(c2, 2) + pow(c3, 2);
    k_sq = pow(k, 2);
    Eigen::Matrix3d A1, A2, A3;
    
    A1(0, 0) = 2 * c1 / k - 2 * c1 * (1 + c1_sq - c2_sq - c3_sq) / k_sq;
    A1(0, 1) = -2 * c2 / k - 2 * c2 * (1 + c1_sq - c2_sq - c3_sq) / k_sq;
    A1(0, 2) = -2 * c3 / k - 2 * c3 * (1 + c1_sq - c2_sq - c3_sq) / k_sq;
    A1(1, 0) = 2 * c2 / k - 4 * c1 * (c1 * c2 + c3) / k_sq;
    A1(1, 1) = 2 * c1 / k - 4 * c2 * (c1 * c2 + c3) / k_sq;
    A1(1, 2) = 2 / k - 4 * c3 * (c1 * c2 + c3) / k_sq;
    A1(2, 0) = 2 * c3 / k - 4 * c1 * (c1 * c3 - c2) / k_sq;
    A1(2, 1) = -2 / k + 4 * c2 * (c1 * c3 - c2) / k_sq;
    A1(2, 2) = 2 * c1 / k - 4 * c3 * (c1 * c3 - c2) / k_sq;
    
    A2(0, 0) = 2 * c2 / k - 4 * c1 * (c1 * c2 - c3) / k_sq;
    A2(0, 1) = 2 * c1 / k - 4 * c2 * (c1 * c2 - c3) / k_sq;
    A2(0, 2) = -2 / k - 4 * c3 * (c1 * c2 - c3) / k_sq;
    A2(1, 0) = -2 * c1 / k - 2 * c1 * (1 - c1_sq + c2_sq - c3_sq) / k_sq;
    A2(1, 1) = 2 * c2 / k - 2 * c2 * (1 - c1_sq + c2_sq - c3_sq) / k_sq;
    A2(1, 2) = -2 * c3 / k - 2 * c3 * (1 - c1_sq + c2_sq - c3_sq) / k_sq;
    A2(2, 0) = 2 / k - 4 * c1 * (c1 + c2 * c3) / k_sq;
    A2(2, 1) = 2 * c3 / k - 4 * c2 * (c1 + c2 * c3) / k_sq;
    A2(2, 2) = 2 * c2 / k - 4 * c3 * (c1 + c2 * c3) / k_sq;
    
    A3(0, 0) = 2 * c3 / k - 4 * c1 * (c2 + c1 * c3) / k_sq;
    A3(0, 1) = 2 / k - 4 * c2 * (c2 + c1 * c3) / k_sq;
    A3(0, 2) = 2 * c1 / k - 4 * c3 * (c2 + c1 * c3) / k_sq;
    A3(1, 0) = -2 / k - 4 * c1 * (c2 * c3 - c1) / k_sq;
    A3(1, 1) = 2 * c3 / k - 4 * c2 * (c2 * c3 - c1) / k_sq;
    A3(1, 2) = 2 * c2 / k - 4 * c3 * (c2 * c3 - c1) / k_sq;
    A3(2, 0) = -2 * c1 / k - 2 * c1 * (1 - c1_sq - c2_sq + c3_sq) / k_sq;
    A3(2, 1) = -2 * c2 / k - 2 * c2 * (1 - c1_sq - c2_sq + c3_sq) / k_sq;
    A3(2, 2) = 2 * c3 / k - 2 * c3 * (1 - c1_sq - c2_sq + c3_sq) / k_sq;

    Eigen::Matrix3d O33 = Eigen::MatrixXd::Zero(3, 3);
    Eigen::Matrix3d I33 = Eigen::MatrixXd::Identity(3, 3);
    J_G.block<3, 3>(0, 0) = A1; J_G.block<3, 3>(0, 3) = O33;
    J_G.block<3, 3>(3, 0) = A2; J_G.block<3, 3>(3, 3) = O33;
    J_G.block<3, 3>(6, 0) = A3; J_G.block<3, 3>(6, 3) = O33;
    J_G.block<3, 3>(9, 0) = O33; J_G.block<3, 3>(9, 3) = I33;
}

bool CamBasedProblemLM::isValidFlow(const Eigen::Vector2d &flow)
{
    if ((flow(0) == 0 && flow(1) == 0) || std::isnan(flow(0)) || std::isnan(flow(1)))
        return false;
    return true;
}

bool CamBasedProblemLM::isValidPoint(Eigen::Vector2d &p) const
{
    if (p(0) < 0 || p(0) > mEventCamera->getWidth()  ||
        p(1) < 0 || p(1) > mEventCamera->getHeight() )
        return false;
    return true;
}

bool CamBasedProblemLM::isValidPatch(Eigen::Vector2d &patchCentreCoord, Eigen::MatrixXi &mask, size_t wx, size_t wy) const
{
    if (patchCentreCoord(0) < (wx - 1) / 2 || patchCentreCoord(0) > mEventCamera->getWidth() - (wx - 1) / 2 - 1 ||
        patchCentreCoord(1) < (wy - 1) / 2 || patchCentreCoord(1) > mEventCamera->getHeight() - (wy - 1) / 2 - 1)
        return false;
    if (mask(patchCentreCoord(1) - (wy - 1) / 2, patchCentreCoord(0) - (wx - 1) / 2) < 125) return false;
    if (mask(patchCentreCoord(1) - (wy - 1) / 2, patchCentreCoord(0) + (wx - 1) / 2) < 125) return false;
    if (mask(patchCentreCoord(1) + (wy - 1) / 2, patchCentreCoord(0) - (wx - 1) / 2) < 125) return false;
    if (mask(patchCentreCoord(1) + (wy - 1) / 2, patchCentreCoord(0) + (wx - 1) / 2) < 125) return false;
    return true;
}

void CamBasedProblemLM::computeTransformation(const Eigen::Matrix4d &T1, const Eigen::Matrix4d &T2) const {
    Eigen::Matrix3d R1 = T1.block<3, 3>(0, 0);
    Eigen::Vector3d t1 = T1.block<3, 1>(0, 3);
    Eigen::Matrix3d R2 = T2.block<3, 3>(0, 0);
    Eigen::Vector3d t2 = T2.block<3, 1>(0, 3);

    Eigen::Matrix3d R = R1.transpose() * R2;
    Eigen::Vector3d t = R1.transpose() * (t2 - t1);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
}