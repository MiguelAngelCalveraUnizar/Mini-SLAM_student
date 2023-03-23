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

#include "EurocVisualLoader.h"

#include <iostream>
#include <fstream>

#include <sys/stat.h>

using namespace std;

EurocVisualLoader::EurocVisualLoader(std::string folderPath, std::string timesPath, std::string groundTruthPath) {
    ifstream fTimes;
    fTimes.open(timesPath.c_str());
    vTimeStamps_.reserve(5000);
    vImgsPairs_.reserve(5000);

    string sPathLeft = "mav0/cam0/data";
    string sPathRight = "mav0/cam1/data";

    if(!fTimes.is_open()){
        cerr << "[EurocVisualLoader]: Could not load dataset at " << folderPath << endl;
        return;
    }

    while(!fTimes.eof()){
        string s;
        getline(fTimes,s);
        if(!s.empty()){
            stringstream ss;
            ss << s;
            string imLeft = folderPath + "/" + sPathLeft + "/" + ss.str() + ".png";
            string imRight = folderPath + "/" + sPathRight + "/" + ss.str() + ".png";
            vImgsPairs_.push_back(make_pair(imLeft, imRight));
            double t;
            ss >> t;
            vTimeStamps_.push_back(t/1e9);
        }
    }

    cv::Mat im;
    getLeftImage(0,im);
    imSize_ = im.size();

    //Check if the ground truth has been already processed
    string gtProcessedPath = folderPath + "/left_gt.txt";
    struct stat buffer;
    if(stat(gtProcessedPath.c_str(),&buffer) != 0){   //Not processed
        cout << "[EurocVisualLoader]: processing ground truth ... " << endl;

        loadBodyGroundTruth(groundTruthPath,gtProcessedPath);
    }
    else{
        cout << "[EurocVisualLoader]: loading processed ground truth ... " << endl;

        loadLeftGroundTruth(gtProcessedPath);
    }

}

bool EurocVisualLoader::getLeftImage(size_t idx, cv::Mat& im) {
    if(idx >= vTimeStamps_.size()) return false;

    //cout << "[EurocVisualLoader]: loading image at " << vImgsPairs_[idx].first << endl;
    im = cv::imread(vImgsPairs_[idx].first, cv::IMREAD_UNCHANGED);

    return true;
}

bool EurocVisualLoader::getRightImage(size_t idx, cv::Mat& im) {
    if(idx >= vTimeStamps_.size()) return false;

    im = cv::imread(vImgsPairs_[idx].first, cv::IMREAD_UNCHANGED);

    return true;
}

bool EurocVisualLoader::getTimeStamp(size_t idx, double &timestamp) {
    if(idx >= vTimeStamps_.size()) return false;

    timestamp = vTimeStamps_[idx];

    return true;
}

int EurocVisualLoader::getLenght() {
    return (int)vTimeStamps_.size();
}

cv::Size EurocVisualLoader::getImageSize() {
    return imSize_;
}

void EurocVisualLoader::loadBodyGroundTruth(std::string path, string outFile) {
    //Transformate reference
    Eigen::Matrix3f R;
    R << 0.0148655429818, -0.999880929698, 0.00414029679422,
            0.999557249008, 0.0149672133247, 0.025715529948,
            -0.0257744366974, 0.00375618835797, 0.999660727178;
    Eigen::Vector3f t;
    t << -0.0216401454975, -0.064676986768, 0.00981073058949;
    Sophus::SE3f Tbs(R,t);

    //Open file
    ifstream fGroundTruth;
    fGroundTruth.open(path.c_str());

    if(!fGroundTruth.is_open()){
        cerr << "[EurocVisualLoader]: Could not load ground truth at " << path << endl;
        return;
    }

    ofstream processedGT;
    processedGT.open(outFile);

    if(!processedGT.is_open()){
        cerr << "[EurocVisualLoader]: Could not open output file at " << outFile << endl;
        return;
    }

    //Skip first line
    string s;
    getline(fGroundTruth,s);

    //Format:
    //#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z []
    while(!fGroundTruth.eof()){
        string s;
        getline(fGroundTruth,s);
        if(!s.empty()){
            //Change all commas to blank spaces
            replace(s.begin(),s.end(),',',' ');
            stringstream ss;
            ss << s;
            double timeStamp, vx, vy, vz, qw, qx, qy, qz;
            ss >> timeStamp >> vx >> vy >> vz >> qw >> qx >> qy >> qz;

            Sophus::SE3f pose(Eigen::Quaternionf(qw, qx, qy, qz),Eigen::Vector3f(vx, vy, vz));
            Sophus::SE3f finalPose = pose * Tbs;

            groundTruth_[timeStamp/1e9] = finalPose;

            processedGT << fixed <<  timeStamp << "," << " " << finalPose.unit_quaternion().w() << "," << finalPose.unit_quaternion().x();
            processedGT << "," << finalPose.unit_quaternion().y() << "," << finalPose.unit_quaternion().z();
            processedGT << "," << finalPose.translation()[0] << "," << finalPose.translation()[1] << ","  << finalPose.translation()[2] << endl;
        }
    }

    fGroundTruth.close();
    processedGT.close();
}

void EurocVisualLoader::loadLeftGroundTruth(std::string path) {
    //Open file
    ifstream fGroundTruth;
    fGroundTruth.open(path.c_str());

    if(!fGroundTruth.is_open()){
        cerr << "[EurocVisualLoader]: Could not load ground truth at " << path << endl;
        return;
    }

    string s;

    //Format:
    //#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z []
    while(!fGroundTruth.eof()){
        string s;
        getline(fGroundTruth,s);
        if(!s.empty()){
            //Change all commas to blank spaces
            replace(s.begin(),s.end(),',',' ');
            stringstream ss;
            ss << s;
            double timeStamp, vx, vy, vz, qw, qx, qy, qz;
            ss >> timeStamp >> qw >> qx >> qy >> qz >> vx >> vy >> vz;

            Sophus::SE3f pose(Eigen::Quaternionf(qw, qx, qy, qz),Eigen::Vector3f(vx, vy, vz));

            groundTruth_[timeStamp/1e9] = pose;

        }
    }
    fGroundTruth.close();
}

bool EurocVisualLoader::getGroundTruth(double timestamp, Sophus::SE3f& gtPose) {
    if(!groundTruth_.count(timestamp)){
        cerr << "[EurocVisualLoader]: Ground Truth not found with ts " << timestamp << endl;
        return false;
    }

    gtPose = groundTruth_[timestamp];
    return true;
}

Sophus::SE3f EurocVisualLoader::getClosestGroundTruth(double timestamp) {
    double minDiff = std::numeric_limits<double>::max();
    double bestTs;

    for(auto keyValue : groundTruth_){
        double diff = fabs(keyValue.first - timestamp);
        if(diff < minDiff){
            minDiff = diff;
            bestTs = keyValue.first;
        }
    }
    return groundTruth_[bestTs];
}