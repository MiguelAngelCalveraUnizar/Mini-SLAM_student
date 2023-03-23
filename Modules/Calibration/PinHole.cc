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

#include "PinHole.h"

#define fx vParameters_[0]
#define fy vParameters_[1]
#define cx vParameters_[2]
#define cy vParameters_[3]

void PinHole::project(const Eigen::Vector3f& p3D, Eigen::Vector2f& p2D){
    /*
     * Your code for task2 here!
     */
    float x = p3D[0];
    float y = p3D[1];
    float z = p3D[2];

    p2D[0] = cx + (fx*x/z);
    p2D[1] = cy + (fy*y/z);
}

void PinHole::unproject(const Eigen::Vector2f& p2D, Eigen::Vector3f& p3D) {
    /*
     * Your code for task2 here!
     */

    float X = p2D[0];
    float Y = p2D[1];

    p3D[0] = (X - cx)/fx;
    p3D[1] = (Y - cy)/fy;
    p3D[2] = 1;
}

void PinHole::projectJac(const Eigen::Vector3f& p3D, Eigen::Matrix<float,2,3>& Jac) {
    /*
     * Your code for task2 here!
     */
    auto x = p3D[0];
    auto y = p3D[1];
    auto z = p3D[2];
    Jac(0,0) = fx/z;
    Jac(0,1) =  0;
    Jac(0,2) = -(fx*x)/(z*z);
    Jac(1,0) = 0;
    Jac(1,1) = fy/z;
    Jac(1,2) = -(fy*y)/(z*z);

}

void PinHole::unprojectJac(const Eigen::Vector2f& p2D, Eigen::Matrix<float,3,2>& Jac) {
    Jac(0,0) = 1 / fx;
    Jac(0,1) = 0.f;

    Jac(1,0) = 0.f;
    Jac(1,1) = 1 / fy;

    Jac(2,0) = 0.f;
    Jac(2,1) = 0.f;
}