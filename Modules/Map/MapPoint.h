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
 * This class represents a MapPoint: a 3D landmark
 */

#ifndef MINI_SLAM_MAPPOINT_H
#define MINI_SLAM_MAPPOINT_H

#include <opencv2/opencv.hpp>

#include <Eigen/Core>

class MapPoint {
public:
    /*
     * Constructor with a given 3D position
     */
    MapPoint(Eigen::Vector3f& p3d);

    /*
     * Gets the position of the MapPoint in the world reference
     */
    Eigen::Vector3f getWorldPosition();

    /*
     * Sets the new position of the MapPoint in the world reference
     */
    void setWorldPosition(Eigen::Vector3f& p3d);

    /*
     * Sets the most distinctive descriptor of the MapPoint
     */
    void setDescriptor(cv::Mat& desc);

    /*
     * Gets the mos distinctive descriptor of the MapPoint
     */
    cv::Mat& getDescriptor();

    /*
     * Sets the orientation of the MapPoint
     */
    void setNormalOrientation(Eigen::Vector3f& v);

    /*
     * Gets the orientation of the MapPoint
     */
    Eigen::Vector3f getNormalOrientation();

   /*
    * Setters and getter for the Scale invariance distances
    */
    void setMinDistanceInvariance(float minDistance);
    void setMaxDistanceInvariance(float maxDistance);
    float getMinDistanceInvariance();
    float getMaxDistanceInvariance();

    /*
     * Gets the unique if of the MapPoint
     */
    long unsigned int getId();

private:
    //3D position of the point in the world reference
    Eigen::Vector3f position3D_;

    //Normal orientation of all observations of the MapPoint
    Eigen::Vector3f normalOrientation_;

    //Most distinctive descriptor of the MapPoint for fast matching
    cv::Mat mDescriptor_;

    float fMinDistance_, fMaxDistance_;

    //Unique id
    long unsigned int nId_;
    static long unsigned int nNextId_;
};


#endif //MINI_SLAM_MAPPOINT_H
