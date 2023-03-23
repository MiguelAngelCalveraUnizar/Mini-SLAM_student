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
 * Implementation of the PinHole camera model with 4 parameters
 */

#ifndef JJSLAM_PINHOLE_H
#define JJSLAM_PINHOLE_H

#include "CameraModel.h"

#include <assert.h>
#include <vector>

#include <opencv2/opencv.hpp>

class PinHole : public CameraModel{
public:
    PinHole() {
        vParameters_.resize(4);
    }

    /*
     * Constructor with a vector of parameters that corresponds to:
     *      [fx, fy, cx, cy]
     */
    PinHole(const std::vector<float> _vParameters) : CameraModel(_vParameters) {
        assert(vParameters_.size() == 4);
    }

    /*
     * Implementation of the pinhole projection function
     */
    void project(const Eigen::Vector3f& p3D, Eigen::Vector2f& p2D);

    /*
     * Implementation of the pinhole unprojection function
     */
    void unproject(const Eigen::Vector2f& p2D, Eigen::Vector3f& p3D);

    /*
     * Implementation of the jacobian matrix of the pinhole projection function
     */
    void projectJac(const Eigen::Vector3f& p3D, Eigen::Matrix<float,2,3>& Jac);

    /*
     * Implementation of the jacobian matrix of the pinhole unprojection function
     */
    void unprojectJac(const Eigen::Vector2f& p2D, Eigen::Matrix<float,3,2>& Jac);
};


#endif //JJSLAM_PINHOLE_H