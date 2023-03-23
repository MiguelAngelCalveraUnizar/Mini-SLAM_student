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
 * This class implements the algorithm to initialize a Map from 2 monocular views using RANSAC
 * to compute an Essential Matrix to reconstruct the camera poses and the environment
 */

#ifndef MINI_SLAM_MONOCULARMAPINITIALIZER_H
#define MINI_SLAM_MONOCULARMAPINITIALIZER_H

#include "Frame.h"

class MonocularMapInitializer {
public:
    MonocularMapInitializer();

    /*
     * Constructor with the number of expected features, the calibration and thresholds for reconstruction
     */
    MonocularMapInitializer(const int nFeatures, std::shared_ptr<CameraModel> calibration, const float fEpipolarTh, float fMinParallax);

    /*
     * Changes the reference view
     */
    void changeReference(std::vector<cv::KeyPoint>& vKeys);

    /*
     * Tries to initialize using the reference and the current views. Returns true on success with the triangulated points
     * and the estimated camera pose
     */
    bool initialize(const std::vector<cv::KeyPoint>& vCurrKeys, const std::vector<int>& vMatches, const int nMatches,
                    Sophus::SE3f& Tcw, std::vector<Eigen::Vector3f>& v3DPoints, std::vector<bool>& vTriangulated);

private:
    //Essential matrix computation with ransac
    Eigen::Matrix3f findEssentialWithRANSAC(const int nMatches, std::vector<bool>& vInliers, int& nInliers);

    //Computes an Essential Matrix from a minimum set of points
    Eigen::Matrix3f computeE(Eigen::Matrix<float,8,3>& refRays, Eigen::Matrix<float,8,3>& currRays);

    //Computes the number of inliers of a given Essential matrix with the current data
    int computeScoreAndInliers(const int nMatched, Eigen::Matrix<float,3,3>& E, std::vector<bool>& vInliers);

    //Environment reconstruction from a given Essential Matrix
    bool reconstructEnvironment(Eigen::Matrix3f& E, Sophus::SE3f& Tcw, std::vector<Eigen::Vector3f>& v3DPoints,
                                std::vector<bool>& vTriangulated, int& nInliers);

    //Reconstructs the camera pose from the Essential Matrix. Automatically sellects the correct rotaiton and translation
    void reconstructCameras(Eigen::Matrix3f& E ,Sophus::SE3f& Tcw, Eigen::MatrixXf& rays1, Eigen::MatrixXf& rays2);

    //Reconstructs the environment with te predicted camera pose
    bool reconstructPoints(const Sophus::SE3f& Tcw, std::vector<Eigen::Vector3f>& v3DPoints, std::vector<bool>& vTriangulated);

    //Decompose an Essential matrix into the 2 possible rotations and translations
    void decomposeE(Eigen::Matrix3f& E, Eigen::Matrix3f& R1, Eigen::Matrix3f& R2, Eigen::Vector3f& t);

    //Computes the max number of iterations for the RANSAC
    int computeMaxTries(const float fInlierFraction, const float fSuccessLikelihood);

    //Reference and current KeyPoints
    std::vector<cv::KeyPoint> refKeys_, currKeys_;
    std::vector<cv::Point2f> refKeysMatched_, currKeysMatched_;

    //Matches between the reference and current KeyPoints
    std::vector<int> vMatches_;

    //Bearing rays (unprojected points) of the reference and current KeyPoints
    Eigen::Matrix<float,Eigen::Dynamic,3,Eigen::RowMajor> refRays_, currRays_;

    //Calibration of the reference and current view
    std::shared_ptr<CameraModel> calibration_;

    float fEpipolarTh_;
    float fMinParallax_;
};


#endif //MINI_SLAM_MONOCULARMAPINITIALIZER_H
