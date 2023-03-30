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

#include "MiniSLAM.h"

#include "Optimization/g2oBundleAdjustment.h"

using namespace std;

MiniSLAM::MiniSLAM() : tracker_() {
}

MiniSLAM::MiniSLAM(const std::string &settingsFile) {
    //Load settings from file
    cout << "Loading system settings from: " << settingsFile << endl;
    settings_ = Settings(settingsFile);
    cout << settings_ << endl;

    //Create map
    pMap_ = shared_ptr<Map>(new Map(settings_.getMinCommonObs()));

    //Create visualizers
    mapVisualizer_ = shared_ptr<MapVisualizer>(new MapVisualizer(pMap_));
    visualizer_ = shared_ptr<FrameVisualizer>(new FrameVisualizer);

    //Initialize tracker
    tracker_ = Tracking(settings_, visualizer_, mapVisualizer_, pMap_);

    //Initialize mapper
    mapper_ = LocalMapping(settings_,pMap_);
}

bool MiniSLAM::processImage(const cv::Mat &im, Sophus::SE3f& Tcw) {
    //Convert image to grayscale if needed
    cv::Mat grayIm = convertImageToGrayScale(im);

    //Predic camera pose
    bool goodTracked = tracker_.doTracking(grayIm, Tcw);

    //Do mapping
    shared_ptr<KeyFrame> lastKeyFrame = tracker_.getLastKeyFrame();
    mapper_.doMapping(lastKeyFrame);

    //Update viewer windows
    visualizer_->updateWindows();

    //Uncomment for step by step execution (pressing esc key)
    /*while((cv::waitKey(10) & 0xEFFFFF) != 27){
        mapVisualizer_->update();
    }*/

    mapVisualizer_->update();
    cv::waitKey(1);

    return goodTracked;
}

cv::Mat MiniSLAM::convertImageToGrayScale(const cv::Mat &im) {
    cv::Mat grayScaled;

    if(im.type() == CV_8U)
        grayScaled = im;
    else if(im.channels()==3){
        cvtColor(im,grayScaled,cv::COLOR_RGB2GRAY);
    }
    else if(im.channels()==4){
        cvtColor(im,grayScaled,cv::COLOR_BGRA2GRAY);
    }

    return grayScaled;
}

void MiniSLAM::runGlobalBundleAdjustment() {
    cout << "Running a Global Bundle Adjustment" << endl;
    bundleAdjustment(pMap_.get());

    //Save optimized trajectory to file
    ofstream trajectoryFile ("KeyFrameTrajectory.txt");

    unordered_map<ID,std::shared_ptr<KeyFrame>> unorderedKeyFrames = pMap_->getKeyFrames();
    map<ID,std::shared_ptr<KeyFrame>> ordered(unorderedKeyFrames.begin(), unorderedKeyFrames.end());
    for(auto it = ordered.begin(); it != ordered.end(); ++it){
        shared_ptr<KeyFrame> pKF = it->second;
        Sophus::SE3f Twc = pKF->getPose().inverse();
        double ts = pKF->getTimestamp();
        //Save predicted pose to the file
        trajectoryFile << setprecision(17) << ts << "," << setprecision(7) << Twc.translation()(0) << ",";
        trajectoryFile << Twc.translation()(1) << "," << Twc.translation()(2) << ",";
        trajectoryFile << Twc.unit_quaternion().x() << "," << Twc.unit_quaternion().y() << ",";
        trajectoryFile << Twc.unit_quaternion().z() << "," << Twc.unit_quaternion().w() << endl;
    }

    trajectoryFile.close();
}
