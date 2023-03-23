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
 * This class implents functions to perform image visualization like tthe features extracted in a frame,
 * matches done or the state of the current Frame
 */

#ifndef MINI_SLAM_FRAMEVISUALIZER_H
#define MINI_SLAM_FRAMEVISUALIZER_H

#include "Tracking/Frame.h"

#include <opencv2/opencv.hpp>

class FrameVisualizer {
public:
    FrameVisualizer();

    /*
     * Sets a reference Frame for matchings visualization
     */
    void setReferenceFrame(std::vector<cv::KeyPoint>& vKeys, cv::Mat& im);

    /*
     * Draw matches between the reference frame and the given frame
     */
    void drawFrameMatches(std::vector<cv::KeyPoint>& vKeys, cv::Mat& im, std::vector<int>& vMatches);

    /*
     * Draws matches between 2 vectors of KeyPoints
     */
    void drawMatches(std::vector<cv::KeyPoint>& vKeys1, cv::Mat& im1,
                     std::vector<cv::KeyPoint>& vKeys2, cv::Mat& im2, std::vector<int>& vMatches);

    /*
     * Draws the extracd features in an image
     */
    void drawCurrentFeatures(std::vector<cv::KeyPoint>& vKeys, cv::Mat& im);

    /*
     * Draws the current matched MapPoints of a frame
     */
    void drawCurrentFrame(Frame& f);

    /*
     * Updates the windows (so they show the last drawn image)
     */
    void updateWindows();
private:

    std::vector<cv::KeyPoint> vRefKeys_;
    cv::Mat refIm_;
};


#endif //MINI_SLAM_FRAMEVISUALIZER_H
