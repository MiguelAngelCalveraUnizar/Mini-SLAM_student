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

/*
 * Author: Juan J. Gómez Rodríguez (jjgomez@unizar.es)
 *
 * This class represents the map of the SLAM system. It is composed by all MapPoints and KeyFrames and
 * a graph relating observations between them
 */

#ifndef MINI_SLAM_MAP_H
#define MINI_SLAM_MAP_H


#include "Map/MapPoint.h"
#include "Map/KeyFrame.h"

#include <unordered_map>
#include <unordered_set>
#include <memory>

typedef long unsigned int ID;

class Map {
public:
    Map();

    /*
     * Constructor defining the minimum number of common observations between 2 KeyFrames to be
     * considered covisibles
     */
    Map(float minCommonObs);

    /*
     * Inserts a new MapPoint into the map
     */
    void insertMapPoint(std::shared_ptr<MapPoint> pMP);

    /*
     * Inserts a new KeyFrame into the map
     */
    void insertKeyFrame(std::shared_ptr<KeyFrame> pKF);

    /*
     * Removes a MapPoint from the map
     */
    void removeMapPoint(ID id);

    /*
     * Gets the KeyFrame with the given id
     */
    std::shared_ptr<KeyFrame> getKeyFrame(ID id);

    /*
     * Gets the MapPoint with the given id
     */
    std::shared_ptr<MapPoint> getMapPoint(ID id);

    /*
     * Adds to the map the observation of a MapPoint with id mpId at the KeyPoint of index idx
     * in the KeyFrame with id kfId
     */
    void addObservation(ID kfId, ID mpId, size_t idx);

    /*
     * Removes the observation of the MapPoint with id mpId in the KeyFrame with id kfId
     */
    void removeObservation(ID kfId, ID mpId);

    /*
     * Gets the covisible KeyFrames of the KeyFrame with id kfId
     */
    std::vector<std::pair<ID,int>> getCovisibleKeyFrames(ID kfId);

    /*
     * Gets the number of commong observations between 2 KeyFrames
     */
    int numberOfCommonObservationsBetweenKeyFrames(ID kf1, ID kf2);

    /*
     * Gets all MapPoints of the map
     */
    std::unordered_map<ID,std::shared_ptr<MapPoint>>& getMapPoints();

    /*
     * Gets all KeyFrames of the map
     */
    std::unordered_map<ID,std::shared_ptr<KeyFrame>>& getKeyFrames();

    /*
     * Gets the local map of a given KeyFrame. The local map is composed by the 1-neighbour KeyFrames and its
     * observed MapPoints
     */
    void getLocalMapOfKeyFrame(ID kfId, std::set<ID>& sLocalMapPointsIds, std::set<ID>& sLocalKeyFramesIds,
                                std::set<ID>& sLocalFixedKeyFramesIds);

    /*
     * Fuses 2 MapPoints in one so the final MapPoints holds the observation from both
     */
    void fuseMapPoints(ID mp1, ID mp2);

    /*
     * Checks if a MapPoint is seen in a KeyFrame
     */
    int isMapPointInKeyFrame(ID mp, ID kf);

    /*
     * Gets the number of observations of a given MapPoint
     */
    int getNumberOfObservations(ID mp);

    /*
     * Checks that the KeyFrame with the given ID is consistent with the information
     * in the graph. FOR DEBUG PURPOSES ONLY
     */
    void checkKeyFrame(ID kfId){
        /*std::shared_ptr<KeyFrame> pKF = mKeyFrames_[kfId];
        std::vector<std::shared_ptr<MapPoint>>& vMps = pKF->getMapPoints();

        //Node to the KeyFrame
        GraphNode_ n = mKeyFrameGraph_[kfId];
        std::unordered_set<ID> graphIds;
        while(n){
            assert(graphIds.count(n->nMapPointId_) == 0);
            assert(mMapPoints_.count(n->nMapPointId_) != 0);
            graphIds.insert(n->nMapPointId_);
            n = n->pNextMapPoint_;
        }


        for(int i = 0; i < vMps.size(); i++){
            std::shared_ptr<MapPoint> pMP = vMps[i];
            if(pMP){
                assert(graphIds.count(pMP->getId()) == 1);
                assert(mMapPoints_.count(pMP->getId()) != 0);
                graphIds.erase(pMP->getId());
            }
        }

        assert(graphIds.size() == 0);*/
    }

private:
    /*
     * Updates the orientation and most distinctive descriptor of a MapPoint
     */
    void updateOrientationAndDescriptor(ID mpId);

    /*
     * Mapping of the KeyFrame/MapPoint ids and the KeyFrame/MapPoint itself
     */
    std::unordered_map<ID,std::shared_ptr<MapPoint>> mMapPoints_;
    std::unordered_map<ID,std::shared_ptr<KeyFrame>> mKeyFrames_;

    /*
     * Observation graph
     */
    std::unordered_map<ID,std::unordered_map<ID,size_t>> mKeyFrameObs_;
    std::unordered_map<ID,std::unordered_map<ID,size_t>> mMapPointObs_;

    /*
     * Covisibility graph
     */
    std::unordered_map<ID,std::unordered_map<ID,int>> mCovisibilityGraph_;

    /*
     * Observation graph
     */
    class GraphNode{
    public:
        GraphNode(ID kfId, ID mpId, size_t indexInKeyFrame){
            nKeyFrameId_ = kfId;
            nMapPointId_ = mpId;
            idxInKf_ = indexInKeyFrame;
        }

        ID nKeyFrameId_, nMapPointId_;
        int idxInKf_;
        std::shared_ptr<GraphNode> pPrevKeyFrame_, pNextKeyFrame_;
        std::shared_ptr<GraphNode> pPrevMapPoint_, pNextMapPoint_;
    };
    typedef std::shared_ptr<GraphNode> GraphNode_;

    std::unordered_map<ID,GraphNode_> mKeyFrameGraph_;
    std::unordered_map<ID,GraphNode_> mMapPointGraph_;

    float minCommonObs_;
};


#endif //MINI_SLAM_MAP_H
