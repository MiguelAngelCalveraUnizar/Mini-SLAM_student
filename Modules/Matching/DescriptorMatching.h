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
 * Methods for matching binary descriptors (with Hamming distance) between Frames and KeyFrames
 */

#ifndef MINI_SLAM_DESCRIPTORMATCHING_H
#define MINI_SLAM_DESCRIPTORMATCHING_H

#include "Map/KeyFrame.h"
#include "Map/Map.h"
#include "Tracking/Frame.h"

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int HammingDistance(const cv::Mat &a, const cv::Mat &b);

/*
 * Searches matches for initialization in a window centered in the last position a KeyPoint was seen
 */
int searchForInitializaion(Frame& refFrame, Frame& currFrame, int th, std::vector<int>& vMatches, std::vector<cv::Point2f>& vPrevMatched);

/*
 * Searches matches between a reference Frame and a current Frame by reprojecting MapPoints into the current frame
 */
int guidedMatching(Frame& refFrame, Frame& currFrame, int th, std::vector<int>& vMatches, int windowSizeFactor);

/*
 * Searches matches with a given vector of MapPoints by prijecting them into the current frame
 */
int searchWithProjection(Frame& currFrame, int th, std::vector<std::shared_ptr<MapPoint>>& vMapPoints);

/*
 * Searches matches between KeyFrames that fulfill the epipolar constrain to later triangulate a MapPoint
 */
int searchForTriangulation(KeyFrame* kf1, KeyFrame* kf2, int th, float fEpipolarTh, Eigen::Matrix<float,3,3>& E, std::vector<int>& vMatches);

/*
 * Searches matches between the current KeyFrame and the previous one to get more observations or fuse duplicated MapPoints
 */
int fuse(std::shared_ptr<KeyFrame> pKF, int th, std::vector<std::shared_ptr<MapPoint>>& vMapPoints, Map* pMap);

#endif //MINI_SLAM_DESCRIPTORMATCHING_H

