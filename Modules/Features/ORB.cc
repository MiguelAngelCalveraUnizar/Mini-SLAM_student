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

#include "ORB.h"

void ORB::describe(const cv::Mat &im, std::vector<cv::KeyPoint> &vKeys, cv::Mat &desc) {
    //Build blurred image pyramid
    computePyramid(im);

    for(size_t i = 0; i < vKeys.size(); i++){
        const cv::KeyPoint kp = vKeys[i];
        cv::KeyPoint kpscaled = kp;
        kpscaled.pt = kp.pt * vInvScaleFactor_[kp.octave];
        computeORBDescriptor(kpscaled,vImagePyramid_[kp.octave],&pattern[0],desc.ptr((int) i));
    }
}

void ORB::computePyramid(const cv::Mat& im){
    for (int level = 0; level < nScales_; ++level){
        float scale = vInvScaleFactor_[level];
        cv::Size sz(cvRound((float)im.cols*scale), cvRound((float)im.rows*scale));
        cv::Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
        cv::Mat temp(wholeSize, im.type()), masktemp;
        vImagePyramid_[level] = temp(cv::Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

        // Compute the resized image
        if( level != 0 ){
            resize(vImagePyramid_[level-1], vImagePyramid_[level], sz, 0, 0, cv::INTER_LINEAR);

            copyMakeBorder(vImagePyramid_[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           cv::BORDER_REFLECT_101+cv::BORDER_ISOLATED);}
        else{
            copyMakeBorder(im, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                           cv::BORDER_REFLECT_101);
        }
        GaussianBlur(vImagePyramid_[level], vImagePyramid_[level], cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);
    }
}

void ORB::computeORBDescriptor(const cv::KeyPoint& kpt, const cv::Mat& img, const cv::Point* pattern, uchar* desc){
    float angle = (float)kpt.angle*(float)(CV_PI/180.f);
    float a = (float)cos(angle), b = (float)sin(angle);

    const uchar* center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
    const int step = (int)img.step;

#define GET_VALUE(idx) \
        center[cvRound(pattern[idx].x*b + pattern[idx].y*a)*step + \
               cvRound(pattern[idx].x*a - pattern[idx].y*b)]


    for (int i = 0; i < 32; ++i, pattern += 16)
    {
        int t0, t1, val;
        t0 = GET_VALUE(0); t1 = GET_VALUE(1);
        val = t0 < t1;
        t0 = GET_VALUE(2); t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;
        t0 = GET_VALUE(4); t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;
        t0 = GET_VALUE(6); t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;
        t0 = GET_VALUE(8); t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;
        t0 = GET_VALUE(10); t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;
        t0 = GET_VALUE(12); t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;
        t0 = GET_VALUE(14); t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;

        desc[i] = (uchar)val;
    }

#undef GET_VALUE
}