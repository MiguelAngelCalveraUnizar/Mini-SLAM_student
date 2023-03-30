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

#include "Utils/Geometry.h"
#include "Tracking/MonocularMapInitializer.h"

using namespace std;

MonocularMapInitializer::MonocularMapInitializer(){}

MonocularMapInitializer::MonocularMapInitializer(const int nFeatures, shared_ptr<CameraModel> calibration, const float fEpipolarTh, float fMinParallax){
    //Reserve memory
    refKeys_.reserve(nFeatures);
    currKeys_.reserve(nFeatures);

    vMatches_.reserve(nFeatures);

    refRays_.resize(nFeatures,3);
    currRays_.resize(nFeatures,3);

    refKeysMatched_.resize(nFeatures);
    currKeysMatched_.resize(nFeatures);

    calibration_ = calibration;

    fEpipolarTh_ = fEpipolarTh;

    fMinParallax_ = fMinParallax;
}

void MonocularMapInitializer::changeReference(std::vector<cv::KeyPoint> &vKeys) {
    refKeys_ = vKeys;
}

bool MonocularMapInitializer::initialize(const std::vector<cv::KeyPoint>& vCurrKeys, const std::vector<int>& vMatches, const int nMatches,
                                         Sophus::SE3f& Tcw, std::vector<Eigen::Vector3f>& v3DPoints, std::vector<bool>& vTriangulated) {
    //Set up data
    currKeys_ = vCurrKeys;
    vMatches_ = vMatches;

    //Unproject matches points to its bearing rays
    std::vector<size_t> vRansacToFrameIndeces;
    vRansacToFrameIndeces.reserve(nMatches);
    size_t currMatIdx = 0;

    refKeysMatched_.clear();
    currKeysMatched_.clear();

    for(size_t i = 0; i < vMatches.size(); i++){
        if(vMatches_[i] != -1){
            refRays_.block(currMatIdx,0,1,3) = calibration_->unproject(refKeys_[i].pt.x,refKeys_[i].pt.y).normalized();
            currRays_.block(currMatIdx,0,1,3) = calibration_->unproject(currKeys_[vMatches[i]].pt.x,currKeys_[vMatches[i]].pt.y).normalized();

            refKeysMatched_.push_back(refKeys_[i].pt);
            currKeysMatched_.push_back(currKeys_[vMatches[i]].pt);

            vRansacToFrameIndeces[currMatIdx] = i;

            currMatIdx++;
        }
    }

    vector<bool> vInliersOfE(currMatIdx,false);

    //If enough matches found, find an Essential matrix with RANSAC
    int nInliersOfE;
    Eigen::Matrix3f E = findEssentialWithRANSAC(nMatches,vInliersOfE, nInliersOfE);

    fill(vTriangulated.begin(),vTriangulated.end(),false);
    for(size_t i = 0; i < vInliersOfE.size(); i++){
        if(vInliersOfE[i]){
            vTriangulated[vRansacToFrameIndeces[i]] = true;
        }
    }

    //Reconstruct the environment with the Essential matrix found
    return reconstructEnvironment(E,Tcw,v3DPoints,vTriangulated, nInliersOfE);

}

int MonocularMapInitializer::computeMaxTries(const float fInlierFraction, const float fSuccessLikelihood){
    return log(1 - fSuccessLikelihood) /
                 log(1 - pow(fInlierFraction,8));
}

Eigen::Matrix3f MonocularMapInitializer::findEssentialWithRANSAC(const int nMatches, vector<bool>& vInliers, int& nInliers) {
    int bestScore = 0;
    vector<bool> vBestInliers;
    vBestInliers.reserve(nMatches);
    Eigen::Matrix<float,3,3> bestE;

    srand(10);

    //Cluster data for random selection
    vector<int> vLabels;
    vector<cv::Point2f> vCenters;
    cv::kmeans(refKeysMatched_,8,vLabels,cv::TermCriteria( cv::TermCriteria::EPS, 10, 1.0),
               3, cv::KMEANS_PP_CENTERS, vCenters);

    vector<vector<size_t>> vvClusters = vector<vector<size_t>>(8);

    //Assign indices to clusters with the given labels
    for(size_t i = 0; i < vLabels.size(); i++){
        vvClusters[vLabels[i]].push_back(i);
    }

    int nMaxIters = computeMaxTries(0.8,0.95);
    int nCurrIt = 0;

    //Do all iterations
    while(nCurrIt < nMaxIters){
        nCurrIt++;

        //get minimum set of data
        Eigen::Matrix<float,8,3> refRays, currRays;
        for(int i = 0; i < 8; i++){
            random_shuffle(vvClusters[i].begin(),vvClusters[i].end());
            size_t idx = vvClusters[i][0];

            refRays.block<1,3>(i,0) = refRays_.block(idx,0,1,3);
            currRays.block<1,3>(i,0) = currRays_.block(idx,0,1,3);
        }

        //Compute model with the sample
        Eigen::Matrix<float,3,3> E = computeE(refRays,currRays);

        //Get score and inliers for the computed model
        int score = computeScoreAndInliers(nMatches,E,vInliers);
        if(score > bestScore){
            bestScore = score;
            vBestInliers = vInliers;
            bestE = E;
        }
    }

    vInliers = vBestInliers;
    nInliers = bestScore;

    return bestE;
}

Eigen::Matrix3f MonocularMapInitializer::computeE(Eigen::Matrix<float, 8, 3> &refRays,
                                                  Eigen::Matrix<float, 8, 3> &currRays) {
    Eigen::Matrix<float,8,9> A;
    for(size_t i = 0; i < 8; i++){
        A.block<1,3>(i,0) = refRays.row(i) * currRays(i,0);
        A.block<1,3>(i,3) = refRays.row(i) * currRays(i,1);
        A.block<1,3>(i,6) = refRays.row(i) * currRays(i,2);
    }

    Eigen::JacobiSVD<Eigen::Matrix<float,8,9>> svd(A,Eigen::ComputeFullV);
    svd.computeV();
    Eigen::Matrix<float,9,1> eigenVec = svd.matrixV().col(8);
    Eigen::Matrix3f E;
    E.row(0) = eigenVec.block<3,1>(0,0).transpose();
    E.row(1) = eigenVec.block<3,1>(3,0).transpose();
    E.row(2) = eigenVec.block<3,1>(6,0).transpose();

    Eigen::JacobiSVD<Eigen::Matrix<float,3,3>> svd_(E,Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3f v;
    v << 1, 1, 0;
    Eigen::Matrix<float,3,3> Ef = svd_.matrixU()*v.asDiagonal()*svd_.matrixV().transpose();

    return -Ef;
}


int MonocularMapInitializer::computeScoreAndInliers(const int nMatched, Eigen::Matrix<float,3,3>& E,vector<bool>& vInliers) {
    Eigen::MatrixXf r1_hat = (E*refRays_.block(0,0,nMatched,3).transpose()).transpose().rowwise().normalized();

    auto err = (M_PI/2 - (r1_hat.array() *
                currRays_.block(0,0,nMatched,3).rowwise().normalized().array())
                .rowwise().sum().acos()).abs() < fEpipolarTh_;

    int score = 0;
    fill(vInliers.begin(),vInliers.end(),false);
    for(int i = 0; i < nMatched; i++){
        if(err(i)){
            vInliers[i] = true;
            score++;
        }
    }

    return score;
}

bool MonocularMapInitializer::reconstructEnvironment(Eigen::Matrix3f& E, Sophus::SE3f& Tcw, std::vector<Eigen::Vector3f>& v3DPoints,
                                                     std::vector<bool>& vTriangulated, int& nInliers) {
    //Compute rays of the inliers points
    Eigen::MatrixXf refRays(nInliers,3), currRays(nInliers,3);
    size_t currMatIdx = 0;
    for(int i = 0; i < vTriangulated.size(); i++){
        if(vTriangulated[i]){
            refRays.row(currMatIdx) = calibration_->unproject(refKeys_[i].pt.x,refKeys_[i].pt.y).normalized();
            currRays.row(currMatIdx) = calibration_->unproject(currKeys_[vMatches_[i]].pt.x,currKeys_[vMatches_[i]].pt.y).normalized();

            currMatIdx++;
        }
    }

    //Reconstruct camera poses
    reconstructCameras(E,Tcw,refRays,currRays);

    //Reconstruct 3D points (try with the 2 possible translations)
    return reconstructPoints(Tcw,v3DPoints,vTriangulated);
}

void MonocularMapInitializer::reconstructCameras(Eigen::Matrix3f &E, Sophus::SE3f &Tcw, Eigen::MatrixXf& rays1, Eigen::MatrixXf& rays2) {
    //Decompose E into R1, R2 and t
    Eigen::Matrix3f R1,R2,Rgood;
    Eigen::Vector3f t;
    decomposeE(E,R1,R2,t);

    //Choose the smallest rotation
    Rgood = (R2.trace() > R1.trace()) ? R2 : R1;

    //Get correct translation
    float away = ((Rgood*rays1.transpose()-rays2.transpose()).array() *
                 (rays2.transpose().colwise()-t).array()).colwise().sum().sign().sum();

    t = (signbit(away)) ? -t : t;

    Tcw = Sophus::SE3f(Rgood,t);
}

void MonocularMapInitializer::decomposeE(Eigen::Matrix3f &E, Eigen::Matrix3f &R1, Eigen::Matrix3f &R2,
                                         Eigen::Vector3f &t) {
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(E,Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f W;
    W << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    R1 = svd.matrixU() * W.transpose() * svd.matrixV().transpose();
    if(R1.determinant() < 0)
        R1 = -R1;

    R2 = svd.matrixU() * W * svd.matrixV().transpose();
    if(R2.determinant() < 0)
        R2 = -R2;

    t = svd.matrixU().col(2).normalized();
}

bool MonocularMapInitializer::reconstructPoints(const Sophus::SE3f &Tcw, std::vector<Eigen::Vector3f> &v3DPoints,
                                                std::vector<bool>& vTriangulated) {
    vector<float> vParallax;
    int nTriangulated = 0, N = 0;

    Eigen::Vector3f O2 = Tcw.inverse().translation();

    int  err1 = 0, err2 = 0, depth1 = 0, depth2 = 0, parallax_ = 0;

    for(size_t i = 0; i < vTriangulated.size(); i++){
        if(vTriangulated[i]){
            N++;

            //Unproject KeyPoints to rays
            Eigen::Vector3f r1 = calibration_->unproject(refKeys_[i].pt.x,refKeys_[i].pt.y).normalized();
            Eigen::Vector3f r2 = calibration_->unproject(currKeys_[vMatches_[i]].pt.x,currKeys_[vMatches_[i]].pt.y).normalized();

            //Triangulate point
            Eigen::Vector3f p3D;
            triangulate(r1,r2,Sophus::SE3f(),Tcw,p3D);

            //Check the parallax of the triangulated point
            Eigen::Vector3f normal1 = p3D;
            Eigen::Vector3f normal2 = p3D - O2;
            float cosParallaxPoint = cosRayParallax(normal1,normal2);

            //Check that the point has been triangulated in front of the first camera (possitive depth)
            if(p3D(2) < 0.0f){
                vTriangulated[i] = false;
                depth1++;
                continue;
            }

            //Check Reprojection error
            cv::Point2f uv1 = calibration_->project(p3D);

            if(squaredReprojectionError(refKeys_[i].pt,uv1) > 5.991){
                vTriangulated[i] = false;
                err1++;
                continue;
            }

            Eigen::Vector3f p3D2 = Tcw * p3D;
            if(p3D2(2) < 0.0f){
                vTriangulated[i] = false;
                depth2++;
                continue;
            }

            cv::Point2f uv2 = calibration_->project(p3D2);

            if(squaredReprojectionError(currKeys_[vMatches_[i]].pt,uv2) > 5.991){
                vTriangulated[i] = false;
                err2++;
                continue;
            }

            v3DPoints[i] = p3D;

            nTriangulated++;
            vParallax.push_back(cosParallaxPoint);

        }
    }

    if(nTriangulated == 0)
        return false;

    sort(vParallax.begin(),vParallax.end());
    size_t idx = min(50,int(vParallax.size()-1));
    float _parallax = vParallax[idx];

    if(nTriangulated > 100 && _parallax > fMinParallax_) {
        return true;
    }
    else{
        return false;
    }
}
