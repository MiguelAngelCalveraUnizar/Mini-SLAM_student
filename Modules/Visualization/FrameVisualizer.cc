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



#include "FrameVisualizer.h"

using namespace std;

FrameVisualizer::FrameVisualizer() {}

void FrameVisualizer::setReferenceFrame(std::vector<cv::KeyPoint> &vKeys, cv::Mat &im) {
    vRefKeys_ = vKeys;
    refIm_ = im.clone();
}

void FrameVisualizer::drawFrameMatches(std::vector<cv::KeyPoint> &vKeys, cv::Mat &im, vector<int>& vMatches) {
    vector<cv::DMatch> vDMatches;
    vector<cv::KeyPoint> vGoodKeys1, vGoodKeys2;

    for(size_t i = 0, j = 0; i < vMatches.size(); i++){
        if(vMatches[i] != -1){
            vGoodKeys1.push_back(vRefKeys_[i]);
            vGoodKeys2.push_back(vKeys[vMatches[i]]);
            vDMatches.push_back(cv::DMatch(j,j,0));

            j++;
        }
    }

    cv::Mat imMatches;
    cv::drawMatches(refIm_,vGoodKeys1,im,vGoodKeys2,vDMatches,imMatches);
    cv::imshow("Matches between frames", imMatches);
}

void FrameVisualizer::drawMatches(std::vector<cv::KeyPoint> &vKeys1, cv::Mat &im1, std::vector<cv::KeyPoint> &vKeys2, cv::Mat &im2, std::vector<int> &vMatches) {
    vector<cv::DMatch> vDMatches;
    vector<cv::KeyPoint> vGoodKeys1, vGoodKeys2;

    for(size_t i = 0, j = 0; i < vMatches.size(); i++){
        if(vMatches[i] != -1){
            vGoodKeys1.push_back(vKeys1[i]);
            vGoodKeys2.push_back(vKeys2[vMatches[i]]);
            vDMatches.push_back(cv::DMatch(j,j,0));

            j++;
        }
    }

    cv::Mat imMatches;
    cv::drawMatches(im1,vGoodKeys1,im2,vGoodKeys2,vDMatches,imMatches);
    cv::imshow("Matches between KeyFrames", imMatches);
}

void FrameVisualizer::drawCurrentFeatures(std::vector<cv::KeyPoint> &vKeys, cv::Mat &im) {
    cv::Mat imWithKeys;
    cv::drawKeypoints(im,vKeys,imWithKeys);
    cv::imshow("KeyPoints extracted", imWithKeys);
}

void FrameVisualizer::drawCurrentFrame(Frame &f) {
    cv::Mat im = f.getIm().clone();
    cvtColor(im,im,cv::COLOR_GRAY2BGR);

    auto vKeys = f.getKeyPointsDistorted();
    auto vMapPoints = f.getMapPoints();

    int nMatches = 0;

    for(size_t i = 0; i < vKeys.size(); i++){
        if(vMapPoints[i]){
            cv::Point2f pt1, pt2;
            pt1.x=vKeys[i].pt.x-5;
            pt1.y=vKeys[i].pt.y-5;
            pt2.x=vKeys[i].pt.x+5;
            pt2.y=vKeys[i].pt.y+5;

            cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
            cv::circle(im,vKeys[i].pt,2,cv::Scalar(0,255,0),-1);

            nMatches++;
        }
    }

    stringstream s;
    s << "SLAM MODE | Matches: " << nMatches;

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);
    cv::Mat imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

    cv::imshow("Mini-SLAM: current Frame", imText);
}

void FrameVisualizer::updateWindows() {
    cv::waitKey(10);


}