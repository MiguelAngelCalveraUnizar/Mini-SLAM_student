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
 * Implementaiton of the Budle Adjustemn problem with the g2o optimization library
 */

#ifndef MINI_SLAM_G2OBUNDLEADJUSTMENT_H
#define MINI_SLAM_G2OBUNDLEADJUSTMENT_H

#include "Map/Map.h"

/*
 * Performs a full Bundle Adjustment (optimizes both camera poses and 3D points)
 */
void bundleAdjustment(Map* pMap);

/*
 * Performs an only pose optimization with the given Frame. It detects outliers and removes them
 * from the Frame. Returns the number of inliers (the number of MapPoints hold after the optimization)
 */
int poseOnlyOptimization(Frame& currFrame);

/*
 * Performs a Bundle Adjustmen using the local map around the given KeyFrame
 */
void localBundleAdjustment(Map* pMap, ID currKeyFrameId);

#endif //MINI_SLAM_G2OBUNDLEADJUSTMENT_H
