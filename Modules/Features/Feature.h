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
 * This class represents an interface for Feature extractors. Derived classes must provide a method to compute
 * image features from a given image
 */

#ifndef JJSLAM_FEATURE_H
#define JJSLAM_FEATURE_H

#include <vector>

#include <opencv2/opencv.hpp>

class Feature{
public:
    Feature(){};

    /*
     * Computes KeyPoints in the given image. To be implemented by the children classes
     */
    virtual void extract(const cv::Mat& im, std::vector<cv::KeyPoint>& vKeys) = 0;
private:
};


#endif //JJSLAM_FEATURE_H
