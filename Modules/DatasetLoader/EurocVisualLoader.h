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
 * This class loads a sequence of the EuRoC dataset stored in a given directory downloaded
 * from https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets.
 *
 * It also computes (or loads if already processed) the left camera ground truth (Twc).
 */

#ifndef JJSLAM_EUROCVISUALLOADER_H
#define JJSLAM_EUROCVISUALLOADER_H

#include <string>
#include <vector>

#include "sophus/se3.hpp"

#include <opencv2/opencv.hpp>

class EurocVisualLoader {
public:
    /*
     * Loads the dataset stored at folderPath with the given timestamps at timesPath.
     *
     * If groundTruthPath exists, this constuctor loads the ground truth stored in it. Otherwise, it computes
     * the left camera ground truth an stores in it.
     */
    EurocVisualLoader(std::string folderPath, std::string timesPath, std::string groundTruthPath);

    /*
     * Retrieves the i left image. Returns false if the image does not exit
     */
    bool getLeftImage(size_t idx, cv::Mat& im);

    /*
     * Retrieves the i right image. Returns false if the image does not exit
     */
    bool getRightImage(size_t idx, cv::Mat& im);

    /*
     * Retrieves the timestamp for the i image pair .Returns false if the timestamp does not exit
     */
    bool getTimeStamp(size_t idx, double& timestamp);

    /*
     * Retrieves the ground truth with the given timestamp. Returns false if the gt pose does not exit
     */
    bool getGroundTruth(double timestamp, Sophus::SE3f& gtPose);

    Sophus::SE3f getClosestGroundTruth(double timestamp);

    /*
     * Returns the number of images in the sequence
     */
    int getLenght();

    cv::Size getImageSize();

private:

    /*
     * Loads the ground truth of the drone body and stores at outFile the left camera ground truth
     */
    void loadBodyGroundTruth(std::string path, std::string outFile);

    /*
     * Loads from path the ground truth
     */
    void loadLeftGroundTruth(std::string path);

    std::vector<std::pair<std::string,std::string>> vImgsPairs_;    //Pair with the image paths of the left and right cameras
    std::vector<double> vTimeStamps_;                               //Vector with the timestamps
    std::unordered_map<double,Sophus::SE3f> groundTruth_;           //Loaded ground truth

    cv::Size imSize_;                                               //Size of the images
};


#endif //JJSLAM_EUROCVISUALLOADER_H
