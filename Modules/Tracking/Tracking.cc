/**
* This file is part of Mini-SLAM
*
* Copyright (C) 2021 Juan J. Gómez Rodríguez and Juan D. Tardós, University of Zaragoza.
*
* Mini-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Mini-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with Mini-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include <Utils/Geometry.h>
#include "Tracking.h"

#include "Features/FAST.h"
#include "Features/ORB.h"

#include "Map/KeyFrame.h"
#include "Map/MapPoint.h"
#include "Matching/DescriptorMatching.h"

#include "Optimization/g2oBundleAdjustment.h"

using namespace std;

Tracking::Tracking(){}

Tracking::Tracking(Settings& settings, std::shared_ptr<FrameVisualizer>& visualizer,
                    std::shared_ptr<MapVisualizer>& mapVisualizer, std::shared_ptr<Map> map) {
    currFrame_ = Frame(settings.getFeaturesPerImage(),settings.getGridCols(),settings.getGridRows(),
                       settings.getImCols(),settings.getImRows(), settings.getNumberOfScales(), settings.getScaleFactor(),
                       settings.getCalibration(),settings.getDistortionParameters());
    prevFrame_ = Frame(settings.getFeaturesPerImage(),settings.getGridCols(),settings.getGridRows(),
                       settings.getImCols(),settings.getImRows(),settings.getNumberOfScales(), settings.getScaleFactor(),
                       settings.getCalibration(),settings.getDistortionParameters());

    featExtractor_ = shared_ptr<Feature>(new FAST(settings.getNumberOfScales(),settings.getScaleFactor(),settings.getFeaturesPerImage()*2,20,7));
    descExtractor_ = shared_ptr<Descriptor>(new ORB(settings.getNumberOfScales(),settings.getScaleFactor()));

    vMatches_ = vector<int>(settings.getFeaturesPerImage());

    vPrevMatched_ = vector<cv::Point2f>(settings.getFeaturesPerImage());

    status_ = NOT_INITIALIZED;
    bFirstIm_ = true;
    bMotionModel_ = false;

    monoInitializer_ = MonocularMapInitializer(settings.getFeaturesPerImage(),settings.getCalibration(),settings.getEpipolarTh(),settings.getMinCos());

    visualizer_ = visualizer;
    mapVisualizer_ = mapVisualizer;

    pMap_ = map;

    nLastKeyFrameId = 0;
    nFramesFromLastKF_ = 0;

    bInserted = false;

    settings_ = settings;
}

bool Tracking::doTracking(const cv::Mat &im, const double timestamp,Sophus::SE3f &Tcw) {
    currIm_ = im.clone();

    //Update previous frame
    if(status_ != NOT_INITIALIZED)
        prevFrame_.assign(currFrame_);

    currFrame_.setIm(currIm_);
    currFrame_.setTimestamp(timestamp);

    //Extract features in the current image
    extractFeatures(im);

    visualizer_->drawCurrentFeatures(currFrame_.getKeyPointsDistorted(),currIm_);

    //If no map is initialized, perform monocular initialization
    if(status_ == NOT_INITIALIZED){
        if(monocularMapInitialization()){
            status_ = GOOD;
            Tcw = currFrame_.getPose();

            //Update motion model
            updateMotionModel();

            return true;
        }
        else{
            return false;
        }
    }
    //SLAM is initialized and tracking was good, track new frame
    else if(status_ == GOOD){
        //Mapping may has added/deleted MapPoints
        updateLastMapPoints();
        if(cameraTracking()){
            if(trackLocalMap()){
                //Check if we need to insert a new KeyFrame into the system
                if(needNewKeyFrame()){
                    promoteCurrentFrameToKeyFrame();
                }

                //Update motion model
                updateMotionModel();

                Tcw = currFrame_.getPose();

                visualizer_->drawCurrentFrame(currFrame_);

                return true;
            }
            else{
                status_ = LOST;
                return false;
            }
        }
        else{
            status_ = LOST;
            return false;
        }
    }
    //Camera tracking failed last frame, try to rellocalise
    else{
        //Not implemented yet

        visualizer_->drawCurrentFrame(currFrame_);

        return false;
    }
}

void Tracking::updateLastMapPoints() {
    if(bInserted){
        vector<shared_ptr<MapPoint>> vMps = pMap_->getKeyFrame(nLastKeyFrameId)->getMapPoints();
        Sophus::SE3f Tcw = pMap_->getKeyFrame(nLastKeyFrameId)->getPose();
        prevFrame_.setPose(Tcw);

        for(size_t i = 0; i < vMps.size(); i++){
            if(vMps[i]){
                prevFrame_.setMapPoint(i,vMps[i]);
            }
            else{
                prevFrame_.setMapPoint(i,nullptr);
            }
        }

        bInserted = false;
    }
}

void Tracking::extractFeatures(const cv::Mat &im) {
    //Extracf image features
    featExtractor_->extract(im,currFrame_.getKeyPointsDistorted());

    //Compute descriptors to extracted features
    descExtractor_->describe(im,currFrame_.getKeyPointsDistorted(),currFrame_.getDescriptors());

    //Distribute keys and undistort them
    currFrame_.distributeFeatures();
}

bool Tracking::monocularMapInitialization() {
    //Set first frame received as the reference frame
    if(bFirstIm_){
        monoInitializer_.changeReference(currFrame_.getKeyPoints());
        prevFrame_.assign(currFrame_);

        bFirstIm_ = false;

        visualizer_->setReferenceFrame(prevFrame_.getKeyPointsDistorted(),currIm_);

        for(size_t i = 0; i < vPrevMatched_.size(); i++){
            vPrevMatched_[i] = prevFrame_.getKeyPoint(i).pt;
        }

        return false;
    }

    //Find matches between previous and current frame
    int nMatches = searchForInitializaion(prevFrame_,currFrame_,settings_.getMatchingInitTh(),vMatches_,vPrevMatched_);

    //visualizer_->drawFrameMatches(currFrame_.getKeyPointsDistorted(),currIm_,vMatches_);

    //If not enough matches found, updtate reference frame
    if(nMatches < 70){
        monoInitializer_.changeReference(currFrame_.getKeyPoints());
        prevFrame_.assign(currFrame_);

        visualizer_->setReferenceFrame(prevFrame_.getKeyPointsDistorted(),currIm_);

        for(size_t i = 0; i < vPrevMatched_.size(); i++){
            vPrevMatched_[i] = prevFrame_.getKeyPoint(i).pt;
        }

        return false;
    }

    //Try to initialize by finding an Essential matrix
    Sophus::SE3f Tcw;
    vector<Eigen::Vector3f> v3DPoints;
    v3DPoints.reserve(vMatches_.capacity());
    vector<bool> vTriangulated(vMatches_.capacity(),false);
    if(!monoInitializer_.initialize(currFrame_.getKeyPoints(), vMatches_, nMatches, Tcw, v3DPoints, vTriangulated)){
        return false;
    }

    //Get map scale
    vector<float> vDepths;
    for(int i = 0; i < vTriangulated.size(); i++){
        if(vTriangulated[i])
            vDepths.push_back(v3DPoints[i](2));
    }

    nth_element(vDepths.begin(),vDepths.begin()+vDepths.size()/2,vDepths.end());
    const float scale = vDepths[vDepths.size()/2];

    //Create map
    Tcw.translation() = Tcw.translation() / scale;

    currFrame_.setPose(Tcw);

    int nTriangulated = 0;

    for(size_t i = 0; i < vTriangulated.size(); i++){
        if(vTriangulated[i]){
            Eigen::Vector3f v = v3DPoints[i] / scale;
            shared_ptr<MapPoint> pMP(new MapPoint(v));

            prevFrame_.setMapPoint(i,pMP);
            currFrame_.setMapPoint(vMatches_[i],pMP);

            pMap_->insertMapPoint(pMP);

            nTriangulated++;
        }
    }

    cout << "Map initialized with " << nTriangulated << " MapPoints" << endl;

    shared_ptr<KeyFrame> kf0(new KeyFrame(prevFrame_));
    shared_ptr<KeyFrame> kf1(new KeyFrame(currFrame_));

    pMap_->insertKeyFrame(kf0);
    pMap_->insertKeyFrame(kf1);

    //Set observations into the map
    vector<shared_ptr<MapPoint>>& vMapPoints = kf0->getMapPoints();
    for(size_t i = 0; i < vMapPoints.size(); i++){
        auto pMP = vMapPoints[i];
        if(pMP){
            //Add observation
            pMap_->addObservation(0,pMP->getId(),i);
            pMap_->addObservation(1,pMP->getId(),vMatches_[i]);
        }
    }

    //Run a Bundle Adjustment to refine the solution
    bundleAdjustment(pMap_.get());

    triangulateExtraMapPoints();

    Tcw = kf1->getPose();
    currFrame_.setPose(Tcw);

    updateMotionModel();

    pLastKeyFrame_ = kf1;
    nLastKeyFrameId = kf1->getId();

    mapVisualizer_->updateCurrentPose(Tcw);

    bInserted = true;

    return true;
}

void Tracking::triangulateExtraMapPoints() {
    //Match features without a MapPoint matched
    shared_ptr<KeyFrame> pKF1 = pMap_->getKeyFrame(0);
    shared_ptr<KeyFrame> pKF2 = pMap_->getKeyFrame(1);

    vector<shared_ptr<MapPoint>> vMPs1 = pKF1->getMapPoints();
    vector<shared_ptr<MapPoint>> vMPs2 = pKF2->getMapPoints();

    cv::Mat desc1 = pKF1->getDescriptors();
    cv::Mat desc2 = pKF2->getDescriptors();

    Sophus::SE3f T1w = pKF1->getPose();
    Sophus::SE3f T2w = pKF2->getPose();

    Sophus::SE3f T21 = T2w*T1w.inverse();
    Eigen::Matrix<float,3,3> E = computeEssentialMatrixFromPose(T21);

    shared_ptr<CameraModel> calibration = pKF1->getCalibration();

    int nNew = 0;

    for(size_t i = 0; i < vMPs1.size(); i++){
        if(vMPs1[i]){
            continue;
        }

        Eigen::Vector3f ray1 = (E*calibration->unproject(pKF1->getKeyPoint(i).pt).transpose()).normalized();

        int bestDist = 255;
        size_t bestIdx;
        for(size_t j = 0; j < vMPs2.size(); j++){
            if(vMPs2[j]){
                continue;
            }

            int dist = HammingDistance(desc1.row(i),desc2.row(j));

            if(dist > 50){
                continue;
            }

            Eigen::Vector3f ray2 = calibration->unproject(pKF2->getKeyPoint(j).pt).normalized();

            float err = fabs(M_PI/2 - acos(ray2.dot(ray1)));
            if(err < settings_.getEpipolarTh()){
                bestDist = dist;
                bestIdx = j;
            }
        }
        if(bestDist < settings_.getMatchingForTriangulationTh()){
            //Get matched KeyPoints
            cv::KeyPoint kp1 = pKF1->getKeyPoint(i);
            cv::KeyPoint kp2 = pKF2->getKeyPoint(bestIdx);

            //Check parallax between rays
            Eigen::Vector3f ray1 = calibration->unproject(kp1.pt.x,kp1.pt.y).normalized();
            Eigen::Vector3f ray2 = calibration->unproject(kp2.pt.x,kp2.pt.y).normalized();

            Eigen::Vector3f ray1_w = T1w.inverse().rotationMatrix() * ray1;
            Eigen::Vector3f ray2_w = T2w.inverse().rotationMatrix() * ray2;

            const float cosParallax = cosRayParallax(ray1_w,ray2_w);

            if(cosParallax < settings_.getMinCos()) {
                //Triangulate a 3D point
                Eigen::Vector3f p3D;
                triangulate(ray1, ray2, T1w, T2w, p3D);

                //Check that the triangulated point lies in front of the cameras
                Eigen::Vector3f p3D1 = T1w * p3D;
                Eigen::Vector3f p3D2 = T2w * p3D;

                if (p3D1(2) <= 0) {
                    continue;
                }

                if (p3D2(2) <= 0) {
                    continue;
                }

                //Check reprojection error
                cv::Point2f uv1 = calibration->project(p3D1);
                float sigmaSquared1 = pKF1->getSigma2(kp1.octave);
                if (squaredReprojectionError(kp1.pt, uv1) > 5.991 * sigmaSquared1) {
                    continue;
                }

                cv::Point2f uv2 = calibration->project(p3D2);
                float sigmaSquared2 = pKF2->getSigma2(kp2.octave);
                if (squaredReprojectionError(kp2.pt, uv2) > 5.991 * sigmaSquared2) {
                    continue;
                }

                shared_ptr<MapPoint> pMP(new MapPoint(p3D));

                pMap_->insertMapPoint(pMP);

                pKF1->setMapPoint(i, pMP);
                pKF2->setMapPoint(bestIdx, pMP);

                pMap_->addObservation(pKF1->getId(), pMP->getId(), i);
                pMap_->addObservation(pKF2->getId(), pMP->getId(), bestIdx);

                nNew++;
            }
        }
    }
    bundleAdjustment(pMap_.get());
}

bool Tracking::cameraTracking() {
    //Set pose estimation for the current frame with the motion model
    Sophus::SE3f currPose;
    if(bMotionModel_){
        currPose = motionModel_ * prevFrame_.getPose();
    }
    else{
        currPose = prevFrame_.getPose();
        bMotionModel_ = true;
    }

    currFrame_.setPose(currPose);

    //Match features between current and previous frame
    int nMatches = guidedMatching(prevFrame_,currFrame_,settings_.getMatchingGuidedTh(),vMatches_,1);

    if(nMatches < 20){
        nMatches = guidedMatching(prevFrame_,currFrame_,settings_.getMatchingGuidedTh(),vMatches_,2);
    }

    //Run a pose optimization
    nFeatTracked_ = poseOnlyOptimization(currFrame_);

    //currFrame_.checkAllMapPointsAreGood();

    //Update MapDrawer
    currPose = currFrame_.getPose();
    mapVisualizer_->updateCurrentPose(currPose);

    //We enforce a minimum of 10 MapPoint matches to consider the estimation as good
    return nFeatTracked_ >= 10;
}

bool Tracking::trackLocalMap() {
    //Get local map from the last KeyFrame
    unordered_set<ID> sLocalMapPoints, sLocalKeyFrames, sFixedKeyFrames;
    pMap_->getLocalMapOfKeyFrame(nLastKeyFrameId,sLocalMapPoints,sLocalKeyFrames,sFixedKeyFrames);

    //Keep a record of the already tracked map points
    vector<shared_ptr<MapPoint>>& vTrackedMapPoints = currFrame_.getMapPoints();

    //Match local MapPoints that are not being currently tracked
    int nMatches = searchWithProjection(currFrame_,settings_.getMatchingByProjectionTh(),sLocalMapPoints,pMap_.get());

    //Run a pose optimization
    nFeatTracked_ = poseOnlyOptimization(currFrame_);

    //Update MapDrawer
    Sophus::SE3f currPose = currFrame_.getPose();
    mapVisualizer_->updateCurrentPose(currPose);

    //We enforce a minimum of 20 MapPoint matches to consider the estimation as good
    return nFeatTracked_ >= 20;
}

bool Tracking::needNewKeyFrame() {
    return false;
}

void Tracking::promoteCurrentFrameToKeyFrame() {
    //Promote current frame to KeyFrame
    pLastKeyFrame_ = shared_ptr<KeyFrame>(new KeyFrame(currFrame_));

    //Insert KeyFrame into the map
    pMap_->insertKeyFrame(pLastKeyFrame_);

    //Add all obsevations into the map
    nLastKeyFrameId = pLastKeyFrame_->getId();
    vector<shared_ptr<MapPoint>>& vMapPoints = pLastKeyFrame_->getMapPoints();
    for(int i = 0; i < vMapPoints.size(); i++){
        MapPoint* pMP = vMapPoints[i].get();
        if(pMP)
            pMap_->addObservation(nLastKeyFrameId,pMP->getId(),i);
    }
    pMap_->checkKeyFrame(pLastKeyFrame_->getId());

    bInserted = true;
}

std::shared_ptr<KeyFrame> Tracking::getLastKeyFrame() {
    shared_ptr<KeyFrame> toReturn = pLastKeyFrame_;
    pLastKeyFrame_ = nullptr;

    return toReturn;
}

void Tracking::updateMotionModel() {
    motionModel_ = currFrame_.getPose() * prevFrame_.getPose().inverse();
}
