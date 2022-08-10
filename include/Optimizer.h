/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class LoopClosing;

class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);
    int static PoseOptimization(Frame* pFrame);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);

    // optimize the pose with acoustic ranging distance results
    void static PoseOptimizationDistance(Frame *pFrame, vector<g2o::SE3Quat> groundTruth); 
    void static PoseOptimizationDistance(Eigen::Vector3d &pose_est, Eigen::Vector3d &pose_gt, cv::RNG *rng=NULL); 

};


// extension for optimize pose with acoustic ranging results

// vertex only consider translation
class VertexTran : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void setToOriginImpl() override {
        _estimate << 0, 0, 0;
    }

    virtual void oplusImpl(const double* update) override {
        _estimate += Eigen::Vector3d(update);
    }

    virtual bool read(std::istream& in) override { return true; }

    virtual bool write(std::ostream& out) const override { return true; }
};

// unit edge with pose only, considering distance
class EdgeDist : public g2o::BaseUnaryEdge<1, double, VertexTran> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeDist(Eigen::Vector3d pos) : BaseUnaryEdge(), _pos(pos)
    {}

    virtual void computeError() override {
        const VertexTran* v = static_cast<VertexTran*>(_vertices[0]);
        Eigen::Vector3d T = v->estimate();
        _error(0, 0) = _measurement - (T - _pos).dot(T - _pos);
    }

    virtual void linearizeOplus() override {
        const VertexTran* v = static_cast<VertexTran*>(_vertices[0]);
        Eigen::Vector3d T = v->estimate();
        _jacobianOplusXi[0] = (-2 * (T(0, 0) - _pos(0, 0)));
        _jacobianOplusXi[1] = (-2 * (T(1, 0) - _pos(1, 0)));
        _jacobianOplusXi[2] = (-2 * (T(2, 0) - _pos(2, 0)));
    }

    virtual bool read(std::istream& in) override { return true; }

    virtual bool write(std::ostream& out) const override { return true; }

private:
    Eigen::Vector3d _pos; // position of the other user
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
