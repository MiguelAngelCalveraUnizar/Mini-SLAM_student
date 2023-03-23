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

    //Insert to the covisibility graph
    mMapPointGraph_[pMP->getId()] = nullptr;
}

void Map::insertKeyFrame(std::shared_ptr<KeyFrame> pKF) {
    mKeyFrames_[pKF->getId()] = pKF;

    //Insert to the covisibility graph
    mKeyFrameGraph_[pKF->getId()] = nullptr;
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
    //Create new node related to the observation
    GraphNode_ node(new GraphNode(kfId,mpId,idx));

    //First search for the KeyFrame node position
    assert(mKeyFrames_.count(kfId) != 0);
    GraphNode_ kfNode = mKeyFrameGraph_[kfId];
    if(!kfNode){
        //No node associated to the KeyFrame so we set current node
        node->pPrevMapPoint_ = nullptr;
        node->pNextMapPoint_ = nullptr;

        mKeyFrameGraph_[kfId] = node;
    }
    else{
        //Iterate until we find the correct position
        while(kfNode->nMapPointId_ < mpId && kfNode->pNextMapPoint_){
            kfNode = kfNode->pNextMapPoint_;
        }

        //Position is at the end of the list
        if(kfNode->nMapPointId_ < mpId){
            assert(!kfNode->pNextMapPoint_);

            node->pPrevMapPoint_ = kfNode;
            node->pNextMapPoint_ = kfNode->pNextMapPoint_;

            //Update previous node
            node->pPrevMapPoint_->pNextMapPoint_ = node;
        }
        //Node is not the last one
        else if(kfNode->nMapPointId_ > mpId){
            node->pPrevMapPoint_ = kfNode->pPrevMapPoint_;
            node->pNextMapPoint_ = kfNode;

            kfNode->pPrevMapPoint_ = node;

            if(!node->pPrevMapPoint_) {
                mKeyFrameGraph_[kfId] = node;
            }
            else{
                node->pPrevMapPoint_->pNextMapPoint_ = node;
            }
        }
        else{
            assert(false);
        }
    }

    //Then search for the MapPoint node position
    assert(mMapPoints_.count(mpId) != 0);
    GraphNode_ mpNode = mMapPointGraph_[mpId];
    if(!mpNode){
        node->pPrevKeyFrame_ = nullptr;
        node->pNextKeyFrame_ = nullptr;

        mMapPointGraph_[mpId] = node;
    }
    else{
        while(mpNode->nKeyFrameId_ < kfId && mpNode->pNextKeyFrame_){
            mpNode = mpNode->pNextKeyFrame_;
        }

        //Position is at the end of the list
        if(mpNode->nKeyFrameId_ < kfId){
            assert(!mpNode->pNextKeyFrame_);

            node->pPrevKeyFrame_ = mpNode;
            node->pNextKeyFrame_ = mpNode->pNextKeyFrame_;

            //Update neighbour nodes
            node->pPrevKeyFrame_->pNextKeyFrame_ = node;
        }
        //Node is not the last one
        else{
            node->pPrevKeyFrame_ = mpNode->pPrevKeyFrame_;
            node->pNextKeyFrame_ = mpNode;

            mpNode->pPrevKeyFrame_ = node;

            if(!node->pPrevKeyFrame_){
                mMapPointGraph_[mpId] = node;
            }
            else{
                node->pPrevKeyFrame_->pNextKeyFrame_ = node;
            }
        }
    }

    //Update normal and descriptor
    updateOrientationAndDescriptor(mpId);

}

void Map::removeObservation(ID kfId, ID mpId) {
    //Get MapPoint node
    GraphNode_ node = mMapPointGraph_[mpId];

    while(node->nKeyFrameId_ != kfId){
        node = node->pNextKeyFrame_;
    }

    assert(node->nKeyFrameId_ == kfId && node->nMapPointId_ == mpId);

    //Update KeyFrame dimension
    if(node->pPrevKeyFrame_){
        node->pPrevKeyFrame_->pNextKeyFrame_ = node->pNextKeyFrame_;
    }
    if(node->pNextKeyFrame_){
        node->pNextKeyFrame_->pPrevKeyFrame_ = node->pPrevKeyFrame_;

        if(!node->pPrevKeyFrame_)
            mMapPointGraph_[mpId] = node->pNextKeyFrame_;
    }

    //Update MapPoint dimension
    if(node->pPrevMapPoint_){
        node->pPrevMapPoint_->pNextMapPoint_ = node->pNextMapPoint_;
    }
    if(node->pNextMapPoint_){
        node->pNextMapPoint_->pPrevMapPoint_ = node->pPrevMapPoint_;

        if(!node->pPrevMapPoint_)
            mKeyFrameGraph_[kfId] = node->pNextMapPoint_;
    }

    //The MapPoint has no observations, remove it from the map
    if(!node->pNextKeyFrame_ && !node->pPrevKeyFrame_){
        mMapPointGraph_.erase(mpId);
        mMapPoints_.erase(mpId);
    }
    else{
        //Update normal and descriptor
        updateOrientationAndDescriptor(mpId);
    }
    checkKeyFrame(kfId);
}

bool sortBySec(const pair<ID,int> &a, const pair<ID,int> &b){
    return (a.second < b.second);
}

std::vector<std::pair<ID,int>> Map::getCovisibleKeyFrames(ID kfId) {
    vector<std::pair<ID,int>> vObservations;

    GraphNode_ node = mKeyFrameGraph_[kfId];
    if(!node){
        return vObservations;
    }
    else{
        //Keep record of common observations
        unordered_map<ID,int> mnObservations;
        while(node){
            //Get Keyframes that see this MapPoint
            GraphNode_ mapPointNode = mMapPointGraph_[node->nMapPointId_];
            while(mapPointNode){
                if(!mnObservations.count(mapPointNode->nKeyFrameId_)){
                    mnObservations[mapPointNode->nKeyFrameId_] = 1;
                }
                else{
                    mnObservations[mapPointNode->nKeyFrameId_]++;
                }

                mapPointNode = mapPointNode->pNextKeyFrame_;
            }

            node = node->pNextMapPoint_;
        }

        vObservations = vector<pair<ID,int>>(mnObservations.begin(),mnObservations.end());
        sort(vObservations.begin(),vObservations.end(),sortBySec);

        return vObservations;
    }
}

int Map::numberOfCommonObservationsBetweenKeyFrames(ID kf1, ID kf2) {
    int nObs = 0;

    GraphNode_ node1 = mKeyFrameGraph_[kf1];
    GraphNode_ node2 = mKeyFrameGraph_[kf2];

    while(node1 && node2){
        if(node1->nMapPointId_ == node2->nMapPointId_){
            nObs++;
            node1 = node1->pNextMapPoint_;
            node2 = node2->pNextMapPoint_;
        }
        else if(node1->nMapPointId_ < node2->nMapPointId_){
            node1 = node1->pNextMapPoint_;
        }
        else{
            node2 = node2->pNextMapPoint_;
        }
    }

    return nObs;
}

std::unordered_map<ID, std::shared_ptr<MapPoint> > & Map::getMapPoints() {
    return mMapPoints_;
}

std::unordered_map<ID, std::shared_ptr<KeyFrame> > & Map::getKeyFrames() {
    return mKeyFrames_;
}

void Map::getLocalMapOfKeyFrame(ID kfId, std::unordered_set<ID> &sLocalMapPointsIds, std::unordered_set<ID> &sLocalKeyFramesIds, std::unordered_set<ID> &sLocalFixedKeyFramesIds) {
    unordered_set<ID> frontier, explored, nextFrontier;
    frontier.insert(kfId);

    for(int i = 0; i < 2; i++){
        for(ID expandingId : frontier){
            explored.insert(expandingId);

            //Add this KeyFrame to the local map
            sLocalKeyFramesIds.insert(expandingId);

            //Get MapPoints of the current KeyFrame
            GraphNode_ node = mKeyFrameGraph_[expandingId];
            while(node){
                //Add MapPoint to the local map
                sLocalMapPointsIds.insert(node->nMapPointId_);

                //Check if we add KeyFrames that see this MapPoint
                GraphNode_ mpNode = mMapPointGraph_[node->nMapPointId_];
                while(mpNode){
                    if(frontier.count(mpNode->nKeyFrameId_) == 0 && explored.count(mpNode->nKeyFrameId_) == 0 && sLocalKeyFramesIds.count(mpNode->nKeyFrameId_) == 0){
                        if(numberOfCommonObservationsBetweenKeyFrames(expandingId,mpNode->nKeyFrameId_) > minCommonObs_){
                            //Add to the next frontier
                            nextFrontier.insert(mpNode->nKeyFrameId_);
                        }
                    }
                    mpNode = mpNode->pNextKeyFrame_;
                }

                node = node->pNextMapPoint_;
            }
        }
        frontier.clear();
        frontier = nextFrontier;
        nextFrontier.clear();
    }

    sLocalFixedKeyFramesIds = frontier;
}

void Map::fuseMapPoints(ID mp1, ID mp2) {
    //Decide which point we keep alive and which we kill
    int obs1 = getNumberOfObservations(mp1);
    int obs2 = getNumberOfObservations(mp2);

    ID mpToKeep = (obs1 > obs2) ? mp1 : mp2;
    ID mpToDelete = (obs1 > obs2) ? mp2 : mp1;

    shared_ptr<MapPoint> pMP = mMapPoints_[mpToKeep];

    //Fuse!
    GraphNode_ node = mMapPointGraph_[mpToDelete];
    while(node){
        ID kfIdNode = node->nKeyFrameId_;
        ID mpIdNode = node->nMapPointId_;
        int idx = node->idxInKf_;

        if(isMapPointInKeyFrame(mpToKeep,node->nKeyFrameId_) == -1){
            addObservation(kfIdNode,mpToKeep,idx);
            mKeyFrames_[kfIdNode]->setMapPoint(idx,pMP);
        }
        else{
            mKeyFrames_[kfIdNode]->setMapPoint(idx,nullptr);
        }

        removeObservation(kfIdNode,mpIdNode);

        checkKeyFrame(kfIdNode);

        node = mMapPointGraph_[mpToDelete];
    }

    updateOrientationAndDescriptor(mpToKeep);

    mMapPoints_.erase(mpToDelete);
    mMapPointGraph_.erase(mpToDelete);
}

int Map::isMapPointInKeyFrame(ID mp, ID kf) {
    int idx = -1;
    GraphNode_ node = mMapPointGraph_[mp];

    while(node){
        if(node->nKeyFrameId_ == kf){
            idx = node->idxInKf_;
            break;
        }
        node = node->pNextKeyFrame_;
    }

    return idx;
}

int Map::getNumberOfObservations(ID mp) {
    int nObs = 0;
    GraphNode_ node = mMapPointGraph_[mp];

    while(node){
        nObs++;
        node = node->pNextKeyFrame_;
    }

    return nObs;
}

void Map::updateOrientationAndDescriptor(ID mpId) {
    //Compute most distinctive descriptor of the MapPoint
    int nObservations = 0;
    GraphNode_ obsNode = mMapPointGraph_[mpId];
    while(obsNode){
        nObservations++;
        obsNode = obsNode->pNextKeyFrame_;
    }

    cv::Mat mAllDescriptors(nObservations,32,CV_8U);
    vector<ID> vKfIds(nObservations);
    obsNode = mMapPointGraph_[mpId];
    int i = 0;
    Eigen::Vector3f normal = Eigen::Vector3f::Zero();
    shared_ptr<MapPoint> pMP = mMapPoints_[mpId];
    assert(pMP);
    while(obsNode){
        mKeyFrames_[obsNode->nKeyFrameId_]->getDescriptors().row(obsNode->idxInKf_).copyTo(mAllDescriptors.row(i));

        Eigen::Vector3f normali = pMP->getWorldPosition() - mKeyFrames_[obsNode->nKeyFrameId_]->getPose().inverse().translation();
        normal = normal + normali.normalized();

        vKfIds[i] = obsNode->nKeyFrameId_;

        i++;
        obsNode = obsNode->pNextKeyFrame_;
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