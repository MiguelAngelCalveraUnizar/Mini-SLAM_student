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

#include "KeyFrame.h"

using namespace std;

long unsigned int KeyFrame::nNextId_=0;

KeyFrame::KeyFrame(Frame &f) {
    vKeys_ = f.getKeyPoints();
    descriptors_ = f.getDescriptors().clone();
    vMapPoints_ = f.getMapPoints();

    Tcw_ = f.getPose();

    calibration_ = f.getCalibration();

    grid_ = f.getGrid();

    nId_ = nNextId_++;

    int nScales = f.getNumberOfScales();

    vScaleFactor_.resize(nScales);
    vInvScaleFactor_.resize(nScales);
    vSigma2_.resize(nScales);
    vInvSigma2_.resize(nScales);

    for(int i = 0; i < nScales; i++){
        vScaleFactor_[i] = f.getScaleFactor(i);
        vInvScaleFactor_[i] = f.getInvScaleFactor(i);
        vSigma2_[i] = f.getSigma2(i);
        vInvSigma2_[i] = f.getInvSigma2(i);
    }

    gridElementWidthInv_ = f.getGridElementWidhtInv();
    gridElementHeightInv_ = f.getGridElementHeighInv();
    minCol_ = f.getMinCol();
    minRow_ = f.getMinRow();

    timestamp_ = f.getTimestamp();
}

Sophus::SE3f KeyFrame::getPose() {
    return Tcw_;
}

void KeyFrame::setPose(Sophus::SE3f &Tcw) {
    Tcw_ = Tcw;
}

cv::KeyPoint KeyFrame::getKeyPoint(size_t idx) {
    return vKeys_[idx];
}

std::vector<std::shared_ptr<MapPoint> > & KeyFrame::getMapPoints() {
    return vMapPoints_;
}

cv::Mat & KeyFrame::getDescriptors() {
    return descriptors_;
}

int KeyFrame::getNummberOfMapPoints() {
    int nMps = 0;
    for(shared_ptr<MapPoint> pMP : vMapPoints_){
        if(pMP)
            nMps++;
    }

    return nMps;
}

float KeyFrame::computeSceneMedianDepth() {
    vector<float> vDephts;
    vDephts.reserve(vMapPoints_.size());

    for(shared_ptr<MapPoint> pMP : vMapPoints_){
        if(pMP){
            Eigen::Vector3f p3Dc = Tcw_ * pMP->getWorldPosition();
            vDephts.push_back(p3Dc(2));
        }
    }

    nth_element(vDephts.begin(),vDephts.begin() + vDephts.size()/2,vDephts.end());
    return vDephts[vDephts.size()/2];
}

void KeyFrame::setMapPoint(size_t idx, std::shared_ptr<MapPoint> pMP) {
    if(!pMP){
        assert(vMapPoints_[idx]);
    }
    vMapPoints_[idx] = pMP;
}

std::shared_ptr<CameraModel> KeyFrame::getCalibration() {
    return calibration_;
}

long unsigned int KeyFrame::getId() {
    return  nId_;
}

float KeyFrame::getScaleFactor(int octave) {
    return vScaleFactor_[octave];
}

float KeyFrame::getInvScaleFactor(int octave) {
    return vInvScaleFactor_[octave];
}

float KeyFrame::getSigma2(int octave) {
    return vSigma2_[octave];
}

float KeyFrame::getInvSigma2(int octave) {
    return vInvSigma2_[octave];
}

int KeyFrame::getNumberOfScales() {
    return (int)vScaleFactor_.size();
}

void KeyFrame::getFeaturesInArea(const float x, const float y, const float radius, const int minLevel, const int maxLevel, std::vector<size_t> &vIndices) {
    vIndices.clear();

    int nMinCol,nMinRow;
    int nMaxCol,nMaxRow;
    int dummy;

    //Get range of cells to check
    if(!posInGrid(x-radius,y,nMinCol,dummy)){
        nMinCol = 0;
    }
    if(!posInGrid(x,y-radius,dummy,nMinRow)){
        nMinRow = 0;
    }
    if(!posInGrid(x+radius,y,nMaxCol,dummy)){
        nMaxCol = grid_.size()-1;
    }
    if(!posInGrid(x,y+radius,dummy,nMaxRow)){
        nMaxRow = grid_[0].size()-1;
    }

    for(size_t c = nMinCol; c <= nMaxCol; c++){
        for(size_t r = nMinRow; r <= nMaxRow; r++){
            for(size_t i = 0; i < grid_[c][r].size(); i++){
                size_t keyIndex = grid_[c][r][i];

                int octave = vKeys_[keyIndex].octave;
                if(octave >= minLevel && octave <= maxLevel){
                    cv::Point2f p = vKeys_[keyIndex].pt - cv::Point2f(x,y);
                    if(fabs(p.x)<radius && fabs(p.y)<radius)
                        vIndices.push_back(keyIndex);
                }
            }
        }
    }
}

bool KeyFrame::posInGrid(const float x, const float y, int &col, int &row) {
    col = round((x-minCol_)*gridElementWidthInv_);
    row = round((y-minRow_)*gridElementHeightInv_);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(col<0.0f || col>=grid_.size() || row<0.0f || row>=grid_[0].size())
        return false;

    return true;
}

void KeyFrame::setTimestamp(double ts) {
    timestamp_ = ts;
}

double KeyFrame::getTimestamp() {
    return timestamp_;
}