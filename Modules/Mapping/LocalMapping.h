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
 * Implementation of the Mapping functionalities of Mini-SLAM. It takes KeyFrames and process them so
 * it triangulates new MapPoints, removes duplicated ones and runs a Local Bundl
 */

#ifndef MINI_SLAM_LOCALMAPPING_H
#define MINI_SLAM_LOCALMAPPING_H

#include "Map/KeyFrame.h"
#include "Map/Map.h"
#include "System/Settings.h"

#include "Visualization/FrameVisualizer.h"

#include <memory>

class LocalMapping {
public:
    LocalMapping();

    /*
     * Constructor with the SLAM map and settings
     */
    LocalMapping(Settings& settings, std::shared_ptr<Map> pMap);

    /*
     * Does the mapping operative: triangulation, duplication remove and Local Bundle Adjustment
     */
    void doMapping(std::shared_ptr<KeyFrame>& pCurrKeyFrame);

private:
    /*
     * Removes from the map redundant or wrongly triangulated points
     */
    void mapPointCulling();

    /*
     * Triangulates new MapPoints with the current KeyFrame
     */
    void triangulateNewMapPoints();

    /*
     * Matches MapPoints from the current KeyFrame with the previous ones and checks for duplicates
     */
    void checkDuplicatedMapPoints();

    std::shared_ptr<Map> pMap_;

    std::shared_ptr<KeyFrame> currKeyFrame_;

    Settings settings_;
};


#endif //MINI_SLAM_LOCALMAPPING_H
