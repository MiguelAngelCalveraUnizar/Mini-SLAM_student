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

#include "Calibration/PinHole.h"

#include "Settings.h"

using namespace std;

Settings::Settings(){}

Settings::Settings(const std::string& configFile) {
    //Open settings file
    cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
    if(!fSettings.isOpened()){
        cerr << "[ERROR]: could not open configuration file at: " << configFile << endl;
        cerr << "Aborting..." << endl;

        exit(-1);
    }

    //Read camera calibration
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    vector<float> vCalibration = {fx,fy,cx,cy};

    calibration_ = shared_ptr<CameraModel>(new PinHole(vCalibration));

    //Read (if exists) distortion parameters
    if(!fSettings["Camera.k1"].empty()){
        if(!fSettings["Camera.k3"].empty()){
            vDistortion_.resize(5);
            vDistortion_[4] = fSettings["Camera.k3"];
        }
        else{
            vDistortion_.resize(4);
        }

        vDistortion_[0] = fSettings["Camera.k1"];
        vDistortion_[1] = fSettings["Camera.k2"];
        vDistortion_[2] = fSettings["Camera.p1"];
        vDistortion_[3] = fSettings["Camera.p2"];
    }

    //Read image dimensions
    imCols_ = fSettings["Camera.cols"];
    imRows_ = fSettings["Camera.rows"];

    //Read Feature extractor parameters
    nFeatures_ = fSettings["FeatureExtractor.nFeatures"];
    nScales_ = fSettings["FeatureExtractor.nScales"];
    fScaleFactor_ = fSettings["FeatureExtractor.fScaleFactor"];

    //Read feature grid parameters
    nGridCols_ = fSettings["FeatureGrid.nGridCols"];
    nGridRows_ = fSettings["FeatureGrid.nGridRows"];

    //Read Epipolar threshold
    fEpipolarTh_ = fSettings["Epipolar.th"];

    //Read matching thresholds
    nMatchingInitTh_ = fSettings["Matching.initialization"];
    nMatchingGuidedTh_ = fSettings["Matching.guidedMatching"];
    nMatchingProjectionTh_ = fSettings["Matching.searchByProjection"];
    nMatchingTriangulationTh_ = fSettings["Matching.searchForTriangulation"];
    nMatchingFuseTh_ = fSettings["Matching.fuse"];

    nMinCommonObs_ = fSettings["Map.minObs"];
    fMinCos_ = fSettings["Triangulation.minCos"];
}

ostream &operator<<(std::ostream& output, const Settings& settings){
    output << "SLAM settings: " << endl;

    output << "\t-Camera parameters: [ ";
    output << settings.calibration_->getParameter(0) << " , " << settings.calibration_->getParameter(1) << " , ";
    output << settings.calibration_->getParameter(2) << " , " << settings.calibration_->getParameter(3) << " ]" << endl;

    if(!settings.vDistortion_.empty()){
        output << "\t-Distortion parameters: [ ";
        output << settings.vDistortion_[0] << " , " << settings.vDistortion_[1] << " , ";
        output << settings.vDistortion_[2] << " , " << settings.vDistortion_[3];

        if(settings.vDistortion_.size() == 5)
            cout << " , " << settings.vDistortion_[4];

        cout << " ]" << endl;
    }

    output << "\t-Image dimensions: [ ";
    output << settings.imCols_ << " , " << settings.imRows_ << " ]" << endl;

    output << "\t-Features to extract per image: " << settings.nFeatures_ << endl;

    output << "\t-Number of image scales: " << settings.nScales_ << endl;

    output << "\t-Scale factor: " << settings.fScaleFactor_ << endl;

    output << "\t-Feature grid dimensions: [ " << settings.nGridCols_ << " , " << settings.nGridRows_ << " ]" << endl;

    return output;
}

std::shared_ptr<CameraModel> Settings::getCalibration() {
    return calibration_;
}

std::vector<float> Settings::getDistortionParameters() {
    return vDistortion_;
}

int Settings::getImCols() {
    return imCols_;
}

int Settings::getImRows() {
    return imRows_;
}

int Settings::getFeaturesPerImage() {
    return nFeatures_;
}

int Settings::getNumberOfScales() {
    return nScales_;
}

float Settings::getScaleFactor() {
    return fScaleFactor_;
}

int Settings::getGridCols() {
    return nGridCols_;
}

int Settings::getGridRows() {
    return nGridRows_;
}

float Settings::getEpipolarTh(){
    return fEpipolarTh_;
}

int Settings::getMatchingInitTh(){
    return nMatchingInitTh_;
}

int Settings::getMatchingGuidedTh(){
    return nMatchingGuidedTh_;
}

int Settings::getMatchingByProjectionTh(){
    return nMatchingProjectionTh_;
}

int Settings::getMatchingForTriangulationTh(){
    return nMatchingTriangulationTh_;
}

int Settings::getMatchingFuseTh(){
    return nMatchingFuseTh_;
}

int Settings::getMinCommonObs(){
    return nMinCommonObs_;
}

float Settings::getMinCos(){
    return fMinCos_;
}