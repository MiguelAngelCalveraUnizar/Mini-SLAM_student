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

#include "Optimization/g2oTypes.h"

VertexSBAPointXYZ::VertexSBAPointXYZ() : BaseVertex<3, Eigen::Vector3d>()
{
}

bool VertexSBAPointXYZ::read(std::istream& is)
{

    is >> _estimate(0 ), _estimate(1 ), _estimate(2 );
    return true;
}

bool VertexSBAPointXYZ::write(std::ostream& os) const
{
    os << _estimate;
    return true;
}

EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Eigen::Vector2d, VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {
}

bool EdgeSE3ProjectXYZ::read(std::istream& is){
    for (int i=0; i<2; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {

    for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
            os << " " <<  information()(i,j);
        }
    return os.good();
}


void EdgeSE3ProjectXYZ::linearizeOplus() {
    g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
    g2o::SE3Quat T(vj->estimate());
    VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
    Eigen::Vector3d xyz = vi->estimate();
    Eigen::Vector3d xyz_trans = T.map(xyz);

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    Eigen::Matrix<double,2,3> projectJac = -pCamera->projectJac(xyz_trans);

    _jacobianOplusXi =  projectJac * T.rotation().toRotationMatrix();

    Eigen::Matrix<double,3,6> SE3deriv;
    SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
                 -z , 0.f, x, 0.f, 1.f, 0.f,
                 y ,  -x , 0.f, 0.f, 0.f, 1.f;

    _jacobianOplusXj = projectJac * SE3deriv;
}

EdgeSE3ProjectXYZOnlyPose::EdgeSE3ProjectXYZOnlyPose(){}

bool EdgeSE3ProjectXYZOnlyPose::read(std::istream& is){
    for (int i=0; i<2; i++){
        is >> _measurement[i];
    }
    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++) {
            is >> information()(i,j);
            if (i!=j)
                information()(j,i)=information()(i,j);
        }
    return true;
}

bool EdgeSE3ProjectXYZOnlyPose::write(std::ostream& os) const {

    for (int i=0; i<2; i++){
        os << measurement()[i] << " ";
    }

    for (int i=0; i<2; i++)
        for (int j=i; j<2; j++){
            os << " " <<  information()(i,j);
        }
    return os.good();
}


void EdgeSE3ProjectXYZOnlyPose::linearizeOplus() {
    g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
    Eigen::Vector3d xyz_trans = vj->estimate().map(Xworld);

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];

    Eigen::Matrix<double,2,3> projectJac = -pCamera->projectJac(xyz_trans);

    Eigen::Matrix<double,3,6> SE3deriv;
    SE3deriv << 0.f, z,   -y, 1.f, 0.f, 0.f,
            -z , 0.f, x, 0.f, 1.f, 0.f,
            y ,  -x , 0.f, 0.f, 0.f, 1.f;

    _jacobianOplusXi = projectJac * SE3deriv;
}