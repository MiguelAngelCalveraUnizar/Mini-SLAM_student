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
 * Implementation of the types needed by g2o to perform Budnle Adjustemnt
 */

#ifndef MINI_SLAM_G2OTYPES_H
#define MINI_SLAM_G2OTYPES_H

#include "Calibration/CameraModel.h"

#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Geometry>

#include <memory>

class  VertexSBAPointXYZ : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSBAPointXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
        _estimate.fill(0);
    }

    virtual void oplusImpl(const double * update)
    {
        Eigen::Map<const Eigen::Vector3d> v(update);
        _estimate += v;
    }
};

/*
 * Error function for geometric (reprojection error) Bundle Adjustment with analytic derivatives. Both
 * camera pose and 3D point are optimizable parameters
 */
class EdgeSE3ProjectXYZ: public  g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZ();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);      //Observed point in the image
        Eigen::Vector3d p3Dw = v2->estimate();  //Predicted 3D world position  of the point
        g2o::SE3Quat Tcw = v1->estimate();      //Predicted camera pose

        /*
         * Your code for task 3 here! Example:
         * _error = Eigen::Vector2d::Ones()*100;
         */
        Eigen::Vector3d p3Dc = Tcw.map(p3Dw) ; //* p3DW Tcw * p3Dw
        Eigen::Vector2d p2D = pCamera->project(p3Dc);
        _error = obs - p2D;
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        return ((v1->estimate().map(v2->estimate()))(2)>0.0);
    }

    virtual void linearizeOplus();

    std::shared_ptr<CameraModel> pCamera;
};

class EdgeSE3ProjectXYZOnlyPose: public g2o::BaseUnaryEdge<2,Eigen::Vector2d,g2o::VertexSE3Expmap>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3ProjectXYZOnlyPose();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector2d obs(_measurement);  //Observed point in the image
        g2o::SE3Quat Tcw = v1->estimate();  //Predicted camera pose

        /*
        * Your code for task 3 here! Example:
        * _error = Eigen::Vector2d::Ones()*100;
        */
        Eigen::Vector3d p3Dc = Tcw.map(Xworld) ;
        Eigen::Vector2d p2D = pCamera->project(p3Dc);
        _error = obs - p2D;
    }

    bool isDepthPositive() {
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d p3dCamera = v1->estimate().map(Xworld);
        return ((v1->estimate().map(Xworld))(2)>0.0);
    }

    virtual void linearizeOplus();

    Eigen::Vector3d Xworld;
    std::shared_ptr<CameraModel> pCamera;
};

#endif //MINI_SLAM_G2OTYPES_H
