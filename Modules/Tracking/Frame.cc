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


#include "Frame.h"

using namespace std;

Frame::Frame(){}

Frame::Frame(const int nFeatures, const int nGridCols, const int nGridRows,
             const int nImCols, const int nImRows, int nScales, float fScaleFactor,
             const std::shared_ptr<CameraModel> calibration,
             const vector<float>& vDistortion){
    vKeys_ = vector<cv::KeyPoint>(nFeatures);
    vKeysDis_ = vector<cv::KeyPoint>(nFeatures);
    descriptors_ = cv::Mat(nFeatures,32,CV_8U);
    vMapPoints_ = vector<shared_ptr<MapPoint>>(nFeatures,nullptr);

    calibration_ = calibration;

    //Compute image boundaries as distortion can change typical values
    computeImageBoundaries(vDistortion,nImCols,nImRows);

    //Initialize feature grid
    initializeGrid(nGridCols,nGridRows,nFeatures);

    vDistortion_ = vDistortion;
    if(vDistortion.size() != 0){
        undistMat_ = cv::Mat(nFeatures,2,CV_32F);
    }

    //Compute scale factor and uncertainties
    nScales_ = nScales;

    vScaleFactor_.resize(nScales);
    vInvScaleFactor_.resize(nScales);
    vSigma2_.resize(nScales);
    vInvSigma2_.resize(nScales);

    vScaleFactor_[0]=1.0f;
    vSigma2_[0]=1.0f;
    for(int i=1; i<nScales; i++){
        vScaleFactor_[i] = vScaleFactor_[i-1] * fScaleFactor;
        vSigma2_[i]=vScaleFactor_[i]*vScaleFactor_[i];
    }

    for(int i=0; i<nScales; i++){
        vInvScaleFactor_[i] = 1.0f / vScaleFactor_[i];
        vInvSigma2_[i]=1.0f/vSigma2_[i];
    }

}

void Frame::setPose(Sophus::SE3f &Tcw) {
    Tcw_ = Tcw;
}

std::vector<cv::KeyPoint>& Frame::getKeyPoints() {
    return vKeys_;
}

std::vector<cv::KeyPoint>& Frame::getKeyPointsDistorted() {
    return vKeysDis_;
}

cv::KeyPoint Frame::getKeyPoint(const size_t idx) {
    return vKeys_[idx];
}

Grid Frame::getGrid() {
    return grid_;
}

void Frame::setMapPoint(size_t idx, std::shared_ptr<MapPoint> pMP) {
    vMapPoints_[idx] = pMP;
}

cv::Mat& Frame::getDescriptors() {
    return descriptors_;
}

const Sophus::SE3f Frame::getPose() const {
    return Tcw_;
}

std::vector<std::shared_ptr<MapPoint> > & Frame::getMapPoints() {
    return vMapPoints_;
}

std::shared_ptr<MapPoint> Frame::getMapPoint(const size_t idx) {
    return vMapPoints_[idx];
}

void Frame::clearMapPoints() {
    fill(vMapPoints_.begin(),vMapPoints_.end(), nullptr);
}

std::shared_ptr<CameraModel> Frame::getCalibration() {
    return calibration_;
}

void Frame::assign(Frame &F) {
    //No memory reallocation is performed, faster!!
    vKeys_.swap(F.vKeys_);
    vKeysDis_.swap(F.vKeysDis_);
    vMapPoints_.swap(F.vMapPoints_);
    fill(F.vMapPoints_.begin(),F.vMapPoints_.end(), nullptr);
    cv::swap(descriptors_, F.descriptors_);

    //Swap feature grid
    for(size_t col = 0; col < grid_.size(); col++){
        for(size_t row = 0; row < grid_[col].size(); row++){
            grid_[col][row].swap(F.grid_[col][row]);
        }
    }

    Tcw_ = F.Tcw_;

    im_ = F.im_.clone();
}

void Frame::distributeFeatures() {
    if(vDistortion_.size() != 0){
        undistortKeys();
    }
    else{
        vKeys_ = vKeysDis_;
    }

    //Clear contents of the grid without releasing the memory
    for(size_t col = 0; col < grid_.size(); col++){
        for(size_t row = 0; row < grid_[col].size(); row++){
            grid_[col][row].clear();
        }
    }

    //Assign each of the extracted features to one of the grid cells
    for(size_t i = 0; i < vKeys_.size(); i++){
        const cv::KeyPoint &kp = vKeys_[i];

        int nColGrid, nRowGrid;
        if(posInGrid(kp.pt.x,kp.pt.y,nColGrid,nRowGrid))
            grid_[nColGrid][nRowGrid].push_back(i);
    }
}

void Frame::initializeGrid(const int nGridCols, const int nGridRows, const int nFeatures) {
    int nReserve = 0.5*nFeatures/(nGridCols*nGridRows);
    grid_ = Grid(nGridCols,vector<vector<size_t>>(nGridRows,vector<size_t>(nReserve)));

    gridElementWidthInv_ = static_cast<float>(nGridCols)/static_cast<float>(maxCol_-minCol_);
    gridElementHeightInv_ = static_cast<float>(nGridRows)/static_cast<float>(maxRow_-minRow_);
}

void Frame::computeImageBoundaries(const vector<float> &vDistortion, const int nImCols, const int nImRows) {
    if(vDistortion.size() == 0){
        minCol_ = 0.0f;
        minRow_ = 0.0f;
        maxCol_ = nImCols;
        maxRow_ = nImRows;
    }
    else{
        cv::Matx<float,4,2> corners(0.0f,0.0f,
                                    nImCols, 0.0f,
                                    0.0f, nImRows,
                                    nImCols,nImRows);

        //Build K matrix
        cv::Matx33f K(calibration_->getParameter(0), 0.0f, calibration_->getParameter(2),
                      0.0f, calibration_->getParameter(1), calibration_->getParameter(3),
                      0.0f, 0.0f, 1.0f);

        //For some reason, OpenCV does not allow the output matrix to be a Matx
        cv::Mat output;

        //Undistort corners
        cv::undistortPoints(corners,output,K,vDistortion,cv::Mat(),K);

        minCol_ = min(output.at<float>(0,0),output.at<float>(2,0));
        minRow_ = min(output.at<float>(0,1),output.at<float>(1,1));
        maxCol_ = max(output.at<float>(1,0),output.at<float>(3,0));
        maxRow_ = max(output.at<float>(2,1),output.at<float>(3,1));
    }
}

void Frame::undistortKeys() {
    cv::Mat undistMat(vKeysDis_.size(),2,CV_32F);
    for(int i = 0; i < vKeysDis_.size(); i++){
        undistMat.at<float>(i,0) = vKeysDis_[i].pt.x;
        undistMat.at<float>(i,1) = vKeysDis_[i].pt.y;
    }

    //Build K matrix
    cv::Matx33f K(calibration_->getParameter(0), 0.0f, calibration_->getParameter(2),
                  0.0f, calibration_->getParameter(1), calibration_->getParameter(3),
                  0.0f, 0.0f, 1.0f);

    undistMat = undistMat.reshape(2);
    cv::undistortPoints(undistMat,undistMat,K,vDistortion_,cv::Mat(),K);
    undistMat.reshape(1);

    for(int i = 0; i < vKeysDis_.size(); i++){
        vKeys_[i].pt.x = undistMat.at<float>(i,0);
        vKeys_[i].pt.y = undistMat.at<float>(i,1);
        vKeys_[i].octave = vKeysDis_[i].octave;
        vKeys_[i].size = vKeysDis_[i].size;
        vKeys_[i].angle = vKeysDis_[i].angle;
    }
}

bool Frame::posInGrid(const float x, const float y, int &col, int &row) {
    col = round((x-minCol_)*gridElementWidthInv_);
    row = round((y-minRow_)*gridElementHeightInv_);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(col<0.0f || col>=grid_.size() || row<0.0f || row>=grid_[0].size())
        return false;

    return true;
}

void Frame::getFeaturesInArea(const float x, const float y, const float radius, const int minLevel, const int maxLevel, std::vector<size_t> &vIndices) {
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

int Frame::getNumberOfScales() {
    return nScales_;
}

float Frame::getScaleFactor(int octave) {
    return vScaleFactor_[octave];
}

float Frame::getInvScaleFactor(int octave) {
    return vInvScaleFactor_[octave];
}

float Frame::getSigma2(int octave) {
    return vSigma2_[octave];
}

float Frame::getInvSigma2(int octave) {
    return vInvSigma2_[octave];
}

float Frame::getGridElementWidhtInv(){
    return gridElementWidthInv_;
}
float Frame::getGridElementHeighInv(){
    return gridElementHeightInv_;
}

float Frame::getMinCol(){
    return minCol_;
}
float Frame::getMinRow(){
    return minRow_;
}

void Frame::setIm(cv::Mat& im){
    im_ = im.clone();
}

cv::Mat Frame::getIm(){
    return im_.clone();
}

void Frame::checkAllMapPointsAreGood(){
    std::unordered_set<long unsigned int> sIds;
    for(size_t i = 0; i < vMapPoints_.size(); i++){
        std::shared_ptr<MapPoint> pMP = vMapPoints_[i];
        if(pMP){
            assert(sIds.count(pMP->getId()) == 0);
            sIds.insert(pMP->getId());

            Eigen::Vector3f p3Dc = Tcw_ * pMP->getWorldPosition();
            cv::Point2f puv = calibration_->project(p3Dc);
            cv::Point2f uv = vKeys_[i].pt;
            cv::Point2f perr = puv-uv;
            float err = perr.dot(perr*vInvSigma2_[vKeys_[i].octave]);
            assert(err < 6.0f);
        }
    }
}

void Frame::setTimestamp(double ts) {
    timestamp_ = ts;
}

double Frame::getTimestamp() {
    return timestamp_;
}

