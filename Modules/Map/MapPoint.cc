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

#include "MapPoint.h"

using namespace std;

long unsigned int MapPoint::nNextId_=0;

MapPoint::MapPoint(Eigen::Vector3f &p3d) {
    position3D_ = p3d;

    nId_ = nNextId_++;
}

Eigen::Vector3f MapPoint::getWorldPosition() {
    return position3D_;
}

void MapPoint::setWorldPosition(Eigen::Vector3f &p3d) {
    position3D_ = p3d;
}

void MapPoint::setDescriptor(cv::Mat &desc) {
    mDescriptor_ = desc.clone();
}

cv::Mat & MapPoint::getDescriptor() {
    return mDescriptor_;
}

void MapPoint::setNormalOrientation(Eigen::Vector3f &v) {
    normalOrientation_ = v;
}

Eigen::Vector3f MapPoint::getNormalOrientation() {
    return normalOrientation_;
}

long unsigned int MapPoint::getId() {
    return  nId_;
}

void MapPoint::setMinDistanceInvariance(float minDistance){
    fMinDistance_ = minDistance;
}

void MapPoint::setMaxDistanceInvariance(float maxDistance){
    fMaxDistance_ = maxDistance;
}

float MapPoint::getMinDistanceInvariance(){
    return 0.8 * fMinDistance_;
}

float MapPoint::getMaxDistanceInvariance(){
    return 1.2 * fMaxDistance_;
}