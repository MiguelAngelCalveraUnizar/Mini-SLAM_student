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
 * This class represents a KeyFrame: a selected Frame that comes with high visual innovation to the map
 */

#ifndef MINI_SLAM_KEYFRAME_H
#define MINI_SLAM_KEYFRAME_H

#include "Tracking/Frame.h"

class KeyFrame {
public:
    /*
     * Constructor from a Frame
     */
    KeyFrame(Frame& f);

    /*
     * Gets the pose of the KeyFrame
     */
    Sophus::SE3f getPose();

    /*
     * Sets the pose of the KeyFrame
     */
    void setPose(Sophus::SE3f& Tcw);

    /*
     * Gets the KeyPoint at index idx in the KeyFrame
     */
    cv::KeyPoint getKeyPoint(size_t idx);

    /*
     * Gets all the MapPoint matches of the KeyFrame. They are associated with the KeyPoint at the same index
     */
    std::vector<std::shared_ptr<MapPoint>>& getMapPoints();

    /*
     * Gets the descriptor matrix of all KeyPoints of the KeyFrame
     */
    cv::Mat& getDescriptors();

    /*
     * Returns the number of matched MapPoints
     */
    int getNummberOfMapPoints();

    /*
     * Computes the median depth of the matched MapPoints
     */
    float computeSceneMedianDepth();

    /*
     * Sets a MapPoint at the index idx
     */
    void setMapPoint(size_t idx, std::shared_ptr<MapPoint> pMP);

    /*
     * Retrieves the calibration of the KeyFrame
     */
    std::shared_ptr<CameraModel> getCalibration();

    /*
     * Get the unique id of the KeyFrame
     */
    long unsigned int getId();

    /*
     * Gets the corresponding scale factor for a given level of the image pyramid
     */
    float getScaleFactor(int octave);

    /*
     * Gets the corresponding inverse scale factor for a given level of the image pyramid
     */
    float getInvScaleFactor(int octave);

    /*
     * Gets the corresponding KeyPoint observation uncertainty for an observation made in a given
     * level of the image pyramid
     */
    float getSigma2(int octave);

    /*
     * Gets the corresponding inverse KeyPoint observation uncertainty for an observation made in a given
     * level of the image pyramid
     */
    float getInvSigma2(int octave);

    /*
     * Gets the number of scales used in the image pyramid
     */
    int getNumberOfScales();

    /*
     * Retrieves a list of features close to a given (x,y) position that fulfill the following conditions:
     *  -Their distance to (x,y) is less than radius
     *  -They have been observed in an scale grater or equal than minLevel and less or equal than maxLevel
     */
    void getFeaturesInArea(const float x, const float y, const float radius, const int minLevel, const int maxLevel, std::vector<size_t> &vIndices);

    /*
     * Sets the timestamp fo the frame
     */
    void setTimestamp(double ts);

    /*
     * Gets the timestamp fo the frame
     */
    double getTimestamp();

private:
    //KeyPoints, descriptors and associated MapPoints
    std::vector<cv::KeyPoint> vKeys_;
    cv::Mat descriptors_;
    std::vector<std::shared_ptr<MapPoint>> vMapPoints_;

    Sophus::SE3f Tcw_;

    //------------------------
    //     Calibration
    //------------------------
    std::shared_ptr<CameraModel> calibration_;

    //------------------------
    // Point grid for faster feature matching
    //------------------------
    typedef std::vector<std::vector<std::vector<size_t>>> Grid;

    Grid grid_;
    float gridElementWidthInv_, gridElementHeightInv_;
    float minCol_, minRow_;                         //Undistorted image boundaries
    /*
    * Gets the grid coordinates of a point (x,y)in the feature grid
    */
    bool posInGrid(const float x, const float y, int &col, int &row);


    //Unique id
    long unsigned int nId_;
    static long unsigned int nNextId_;

    std::vector<float> vScaleFactor_, vInvScaleFactor_; //Scale and inverse scale factor for each pyramid level
    std::vector<float> vSigma2_, vInvSigma2_;

    double timestamp_;
};


#endif //MINI_SLAM_KEYFRAME_H
