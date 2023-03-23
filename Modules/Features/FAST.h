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
 * FAST feature extractor based in the one used in ORB-SLAM, ORB-SLAM2 and ORB-SLAM3. It internally uses an octree
 * to equally distribute features along the image
 */

#ifndef JJSLAM_FAST_H
#define JJSLAM_FAST_H

#include "Feature.h"

#include <opencv2/opencv.hpp>

//Octree nodes
class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class FAST : public Feature{
public:
    /*
     * Basic constructor: it takes the number of scales for the multi scale extracion, the desired number of features,
     * and 2 FAST thresholds
     */
    FAST(int nScales, float fScaleFactor, int nFeatures, int thFAST, int minThFAST) :
    nScales_(nScales), fScaleFactor_(fScaleFactor), nMaxFeatures_(nFeatures), thFast_(thFAST), minThFAST_(minThFAST) {
        vScaleFactor_.resize(nScales_);
        vInvScaleFactor_.resize(nScales_);

        vScaleFactor_[0]=1.0f;
        for(int i=1; i<nScales_; i++){
            vScaleFactor_[i] = vScaleFactor_[i-1] * fScaleFactor_;
        }

        for(int i=0; i<nScales_; i++){
            vInvScaleFactor_[i] = 1.0f / vScaleFactor_[i];
        }

        vImagePyramid_.resize(nScales_);
        vFeaturesPerLevel_.resize(nScales_);

        float factor = 1.0f / fScaleFactor_;
        float nDesiredFeaturesPerScale = nMaxFeatures_*(1 - factor)/(1 - (float)pow((double)factor, (double)nScales_));

        int sumFeatures = 0;
        for( int level = 0; level < nScales_-1; level++ ){
            vFeaturesPerLevel_[level] = cvRound(nDesiredFeaturesPerScale);
            sumFeatures += vFeaturesPerLevel_[level];
            nDesiredFeaturesPerScale *= factor;
        }
        vFeaturesPerLevel_[nScales_-1] = std::max(nMaxFeatures_ - sumFeatures, 0);

        //This is for orientation
        // pre-compute the end of a row in a circular patch
        umax_.resize(HALF_PATCH_SIZE + 1);

        int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
        int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
        const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
        for (v = 0; v <= vmax; ++v)
            umax_[v] = cvRound(sqrt(hp2 - v * v));

        // Make sure we are symmetric
        for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v){
            while (umax_[v0] == umax_[v0 + 1])
                ++v0;
            umax_[v] = v0;
            ++v0;
        }
    }

    void extract(const cv::Mat& im, std::vector<cv::KeyPoint>& vKeys) override;

private:
    /*
     * Computes the image pyramid for multi-scale processing
     */
    void computePyramid(const cv::Mat& im);

    /*
     * Computes KeyPoints and distribute them with an oct tree
     */
    void computeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint>>& allKeypoints);

    /*
     * Distributes a given set of KeyPoints with an octree
     */
    std::vector<cv::KeyPoint> distributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                                const int &maxX, const int &minY, const int &maxY, const int &N, const int &level);

    /*
     * Computes the oritentation of the given keypoints
     */
    void computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const std::vector<int>& umax);

    /*
     * Computes the orientation of a single KEYPOINT
     */
    float IC_Angle(const cv::Mat& image, cv::Point2f pt,  const std::vector<int> & u_max);

    int nScales_;               //Number of pyramid levels
    float fScaleFactor_;        //Scale factor for the downsampling in the image pyramid
    int nMaxFeatures_;          //Max number of desired features
    int thFast_, minThFAST_;    //FAST thresholds

    std::vector<float> vScaleFactor_, vInvScaleFactor_; //Scale and inverse scale factor for each pyramid level
    std::vector<cv::Mat> vImagePyramid_;                //Image pyramid
    std::vector<int> vFeaturesPerLevel_;                //Number of features to extract per level
    std::vector<int> umax_;

    const int EDGE_THRESHOLD = 19;
    const int PATCH_SIZE = 31;
    const int HALF_PATCH_SIZE = 15;
};


#endif //JJSLAM_FAST_H
