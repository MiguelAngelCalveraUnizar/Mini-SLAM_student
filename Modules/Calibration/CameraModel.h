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
 * An abstract base class for implementing specialized camera models. It provides 4 basic methods
 * that must be implemented in derived classes: project, unproject, projectJac and unprojectJac.
 *
 * Internally, camera intrinsic parameters are stored as floats inside a vector (vParameters_). This
 * base class also offers setter and getters for them.
 *
 * Finally, it also provides some overload functions to allow interaction with other libraries and
 * data types
 */

#ifndef JJSLAM_GEOMETRICCAMERA_H
#define JJSLAM_GEOMETRICCAMERA_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

class CameraModel {
public:
    CameraModel() {}

    /*
     * Constructor with calibration parameters
     */
    CameraModel(const std::vector<float> &_vParameters) : vParameters_(_vParameters) {}

    /*
     * Projects a given 3D point into the image
     */
    virtual void project(const Eigen::Vector3f& p3D, Eigen::Vector2f& p2D) = 0;

    /*
     * Unprojects a given image 2D point into it is bearing ray
     */
    virtual void unproject(const Eigen::Vector2f& p2D, Eigen::Vector3f& p3D) = 0;

    /*
     * Analytic Jacobian of the projection function (stored by rows)
     */
    virtual void projectJac(const Eigen::Vector3f& p3D, Eigen::Matrix<float,2,3>& Jac) = 0;

    /*
     * Analytic Jacobian of the unprojection function
     */
    virtual void unprojectJac(const Eigen::Vector2f& p2D, Eigen::Matrix<float,3,2>& Jac) = 0;

    /*
     * Gets the i calibration parameter
     */
    float getParameter(const int i){return vParameters_[i];}

    /*
     * Sets the i calibration parameter
     */
    void setParameter(const float p, const size_t i){vParameters_[i] = p;}

    /*
     * Return the number of calibration parameters
     */
    int getNumberOfParameters() {return vParameters_.size();}

    //-----------------------------------------------------------------
    //Useful overloadings to allow the use of different data structures
    //-----------------------------------------------------------------

    cv::Point2f project(Eigen::Vector3f & X){
        Eigen::Vector2f uv;

        this->project(X.cast<float>(),uv);

        cv::Point2f toReturn(uv(0),uv(1));
        return toReturn;
    }

    Eigen::Vector2d project(Eigen::Vector3d & X){
        Eigen::Vector2f uv;

        this->project(X.cast<float>(),uv);

        return uv.cast<double>();
    }

    Eigen::Matrix<float,1,3> unproject(const float u, const float v){
        Eigen::Vector2f uv(u,v);
        Eigen::Vector3f ray;

        this->unproject(uv,ray);

        return ray;
    }

    Eigen::Matrix<float,1,3> unproject(cv::Point2f puv){
        Eigen::Vector2f uv(puv.x,puv.y);
        Eigen::Vector3f ray;

        this->unproject(uv,ray);

        return ray;
    }

    Eigen::Matrix<double,2,3> projectJac(Eigen::Vector3d &p3D){
        Eigen::Matrix<float,2,3> jac;

        this->projectJac(p3D.cast<float>(),jac);

        return jac.cast<double>();
    }

protected:
    std::vector<float> vParameters_;    //Vector of calibration parametets
};


#endif //JJSLAM_GEOMETRICCAMERA_H