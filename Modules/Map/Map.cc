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

#include "Map.h"

#include "Matching/DescriptorMatching.h"

using namespace std;

Map::Map(){}

Map::Map(float minCommonObs){
    minCommonObs_ = minCommonObs;
}

void Map::insertMapPoint(std::shared_ptr<MapPoint> pMP) {
    mMapPoints_[pMP->getId()] = pMP;
    mMapPointObs_[pMP->getId()].clear();
}

void Map::insertKeyFrame(std::shared_ptr<KeyFrame> pKF) {
    mKeyFrames_[pKF->getId()] = pKF;
    mKeyFrameObs_[pKF->getId()].clear();
    mCovisibilityGraph_[pKF->getId()].clear();
}

void Map::removeMapPoint(ID id) {
    //Update observation graph
    unordered_map<ID,size_t> mMPObsToDelete = mMapPointObs_[id];
    for(pair<ID,size_t> pair : mMPObsToDelete){
        ID kfId = pair.first;
        size_t idx = pair.second;

        this->removeObservation(kfId,id);
        mKeyFrames_[kfId]->setMapPoint(idx, nullptr);
    }

    mMapPoints_.erase(id);
    mMapPointObs_.erase(id);
}

std::shared_ptr<KeyFrame> Map::getKeyFrame(ID id) {
    if(mKeyFrames_.count(id) != 0)
        return mKeyFrames_[id];
    else
        return nullptr;
}

std::shared_ptr<MapPoint> Map::getMapPoint(ID id) {
    if(mMapPoints_.count(id) != 0)
        return mMapPoints_[id];
    else
        return nullptr;
}

void Map::addObservation(ID kfId, ID mpId, size_t idx) {
    assert(mKeyFrames_.count(kfId) != 0);
    assert(mMapPoints_.count(mpId) != 0);
    assert(mKeyFrameObs_[kfId].count(mpId) == 0);
    assert(mMapPointObs_[mpId].count(kfId) == 0);

    mKeyFrameObs_[kfId][mpId] = idx;
    mMapPointObs_[mpId][kfId] = idx;

    //Update the covisibility graph
    for(pair<ID,size_t> pair : mMapPointObs_[mpId]){
        ID covKfId = pair.first;
        if(kfId == covKfId)
            continue;

        if(mCovisibilityGraph_[kfId].count(covKfId) == 0){
            mCovisibilityGraph_[kfId][covKfId] = 1;
        }
        else{
            mCovisibilityGraph_[kfId][covKfId]++;
        }

        if(mCovisibilityGraph_[covKfId].count(kfId) == 0){
            mCovisibilityGraph_[covKfId][kfId] = 1;
        }
        else{
            mCovisibilityGraph_[covKfId][kfId]++;
        }
    }

    //Update normal and descriptor
    updateOrientationAndDescriptor(mpId);
}

void Map::removeObservation(ID kfId, ID mpId) {
    assert(mKeyFrames_.count(kfId) != 0);
    assert(mMapPoints_.count(mpId) != 0);
    assert(mKeyFrameObs_[kfId].count(mpId) != 0);
    assert(mMapPointObs_[mpId].count(kfId) != 0);

    mKeyFrameObs_[kfId].erase(mpId);
    mMapPointObs_[mpId].erase(kfId);

    for(pair<ID,size_t> pair : mMapPointObs_[mpId]){
        ID covKfId = pair.first;

        mCovisibilityGraph_[kfId][covKfId]--;
        mCovisibilityGraph_[covKfId][kfId]--;
    }
}

bool sortBySec(const pair<ID,int> &a, const pair<ID,int> &b){
    return (a.second < b.second);
}

std::vector<std::pair<ID,int>> Map::getCovisibleKeyFrames(ID kfId) {
    assert(mCovisibilityGraph_.count(kfId) != 0);

    vector<std::pair<ID, int>> vObservations(mCovisibilityGraph_[kfId].begin(), mCovisibilityGraph_[kfId].end());
    sort(vObservations.begin(), vObservations.end(), sortBySec);

    return vObservations;
}

int Map::numberOfCommonObservationsBetweenKeyFrames(ID kf1, ID kf2) {
    assert(mCovisibilityGraph_[kf1][kf2] == mCovisibilityGraph_[kf2][kf1]);

    return mCovisibilityGraph_[kf1][kf2];
}

std::unordered_map<ID, std::shared_ptr<MapPoint> > & Map::getMapPoints() {
    return mMapPoints_;
}

std::unordered_map<ID, std::shared_ptr<KeyFrame> > & Map::getKeyFrames() {
    return mKeyFrames_;
}

void Map::getLocalMapOfKeyFrame(ID kfId, std::set<ID> &sLocalMapPointsIds, std::set<ID> &sLocalKeyFramesIds, std::set<ID> &sLocalFixedKeyFramesIds) {
    set<ID> sAllKFs;

    sLocalKeyFramesIds.insert(kfId);
    for(pair<ID,size_t> pair : mKeyFrameObs_[kfId]){
        sLocalMapPointsIds.insert(pair.first);
    }

    for(auto pair : mCovisibilityGraph_[kfId]){
        ID kfCovId = pair.first;
        int nObs = pair.second;

        if(nObs > minCommonObs_){
            sLocalKeyFramesIds.insert(kfCovId);

            for(auto pairCov : mKeyFrameObs_[kfCovId]){
                sLocalMapPointsIds.insert(pairCov.first);
            }
        }
    }

    for(auto mpId : sLocalMapPointsIds){
        for(auto pair : mMapPointObs_[mpId]){
            sAllKFs.insert(pair.first);
        }
    }

    set<ID> sDifference;
    set_difference(sAllKFs.begin(),sAllKFs.end(),sLocalKeyFramesIds.begin(),sLocalKeyFramesIds.end(),inserter(sDifference,sDifference.begin()));

    sLocalFixedKeyFramesIds = set<ID>(sDifference.begin(),sDifference.end());
}

void Map::fuseMapPoints(ID mp1, ID mp2) {
    //Decide which point we keep alive and which we kill
    int obs1 = mMapPointObs_[mp1].size();
    int obs2 = mMapPointObs_[mp2].size();

    ID mpToKeep = (obs1 > obs2) ? mp1 : mp2;
    ID mpToDelete = (obs1 > obs2) ? mp2 : mp1;

    shared_ptr<MapPoint> pMP = mMapPoints_[mpToKeep];

    unordered_map<ID,size_t> mMPObsToDelete = mMapPointObs_[mpToDelete];

    for(pair<ID,size_t> pair : mMPObsToDelete){
        ID kfId = pair.first;
        size_t idx = pair.second;

        this->removeObservation(kfId,mpToDelete);
        mKeyFrames_[kfId]->setMapPoint(idx, nullptr);

        if(this->isMapPointInKeyFrame(mpToKeep,kfId) == -1){
            this->addObservation(kfId,mpToKeep,idx);
            mKeyFrames_[kfId]->setMapPoint(idx,pMP);
        }
    }

    removeMapPoint(mpToDelete);
}

int Map::isMapPointInKeyFrame(ID mp, ID kf) {
    int idx = -1;
    if(mKeyFrameObs_[kf].count(mp) != 0){
        idx = mKeyFrameObs_[kf][mp];
    }

    return idx;
}

int Map::getNumberOfObservations(ID mp) {
    return mMapPointObs_[mp].size();
}

void Map::updateOrientationAndDescriptor(ID mpId) {
    Eigen::Vector3f normal = Eigen::Vector3f::Zero();
    shared_ptr<MapPoint> pMP = mMapPoints_[mpId];

    int nObservations = mMapPointObs_[mpId].size();
    cv::Mat mAllDescriptors(nObservations,32,CV_8U);
    vector<ID> vKfIds(nObservations);

    int i = 0;
    for(pair<ID,size_t> pair : mMapPointObs_[mpId]){
        ID kfId = pair.first;
        size_t idxInKf = pair.second;

        mKeyFrames_[kfId]->getDescriptors().row(idxInKf).copyTo(mAllDescriptors.row(i));

        Eigen::Vector3f normali = pMP->getWorldPosition() - mKeyFrames_[kfId]->getPose().inverse().translation();
        normal = normal + normali.normalized();

        vKfIds[i] = kfId;

        i++;
    }

    normal = (normal/i).normalized();
    pMP->setNormalOrientation(normal);

    float bestMedian = 255;
    int obsIdx = 0;
    vector<int> vDistances(nObservations,0);
    for(int i = 0; i < nObservations; i++){
        for(int j = 0; j < nObservations; j++){
            vDistances[j] = HammingDistance(mAllDescriptors.row(i),mAllDescriptors.row(j));
        }
        nth_element(vDistances.begin(),vDistances.begin()+nObservations/2,vDistances.end());
        if(vDistances[nObservations/2] < bestMedian){
            bestMedian = vDistances[nObservations/2];
            obsIdx = i;
        }
    }

    shared_ptr<KeyFrame> pRefKf = mKeyFrames_[vKfIds[obsIdx]];

    float dist = (pMP->getWorldPosition() - pRefKf->getPose().inverse().translation()).norm();
    int refOctave = pRefKf->getKeyPoint(obsIdx).octave;

    float maxDistInv = dist * pRefKf->getScaleFactor(refOctave);
    float minDistInv = maxDistInv/pRefKf->getScaleFactor(pRefKf->getNumberOfScales()-1);
    pMP->setMaxDistanceInvariance(maxDistInv);
    pMP->setMinDistanceInvariance(minDistInv);

    cv::Mat bestDesc = mAllDescriptors.row(obsIdx);
    mMapPoints_[mpId]->setDescriptor(bestDesc);
}