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
 * This class loads a sequence of the TUM-RGBD dataset stored in a given directory downloaded
 * from https://vision.in.tum.de/data/datasets/rgbd-dataset/download.
 */

#ifndef MINI_SLAM_TUMRGBDLOADER_H
#define MINI_SLAM_TUMRGBDLOADER_H

#include <string>
#include <vector>

#include "sophus/se3.hpp"

#include <opencv2/opencv.hpp>

class TUMRGBDLoader {
public:
    /*
     * Loads the dataset stored at folderPath with the given timestamps at timesPath
     */
    TUMRGBDLoader(std::string folderPath, std::string timesPath);

    /*
     * Retrieves the i RGB image. Returns false if the image does not exit
     */
    bool getRGBImage(size_t idx, cv::Mat& im);


    /*
     * Retrieves the timestamp for the i image pair .Returns false if the timestamp does not exit
     */
    bool getTimeStamp(size_t idx, double& timestamp);

    /*
     * Returns the number of images in the sequence
     */
    int getLenght();

    cv::Size getImageSize();

private:
    std::vector<std::string> vRGBPaths;     //Image paths
    std::vector<double> vTimeStamps_;       //Vector with the timestamps

    cv::Size imSize_;                       //Size of the images
};


#endif //MINI_SLAM_TUMRGBDLOADER_H
