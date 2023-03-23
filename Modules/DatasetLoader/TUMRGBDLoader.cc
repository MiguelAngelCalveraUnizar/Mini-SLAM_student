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

#include "TUMRGBDLoader.h"


#include <iostream>
#include <fstream>

#include <sys/stat.h>

using namespace std;

TUMRGBDLoader::TUMRGBDLoader(std::string folderPath, std::string timesPath) {
    ifstream fTimes;
    fTimes.open(timesPath.c_str());
    vTimeStamps_.reserve(5000);
    vRGBPaths.reserve(5000);

    if(!fTimes.is_open()){
        cerr << "[TUMRGBDLoader]: Could not load dataset at " << folderPath << endl;
        return;
    }

    //Skip first 3 lines
    string s;
    getline(fTimes,s);
    getline(fTimes,s);
    getline(fTimes,s);

    while(!fTimes.eof()){
        string s;
        getline(fTimes,s);
        if(!s.empty()){
            stringstream ss;
            ss << s;

            double t;
            ss >> t;
            vTimeStamps_.push_back(t);

            string sRGB;
            ss >> sRGB;
            vRGBPaths.push_back(folderPath + "/" + sRGB);

        }
    }

    cv::Mat im;
    getRGBImage(0,im);
    imSize_ = im.size();
}

bool TUMRGBDLoader::getRGBImage(size_t idx, cv::Mat& im) {
    if(idx >= vTimeStamps_.size()) return false;

    //cout << "[EurocVisualLoader]: loading image at " << vImgsPairs_[idx].first << endl;
    im = cv::imread(vRGBPaths[idx], cv::IMREAD_UNCHANGED);

    return true;
}

bool TUMRGBDLoader::getTimeStamp(size_t idx, double &timestamp) {
    if(idx >= vTimeStamps_.size()) return false;

    timestamp = vTimeStamps_[idx];

    return true;
}

int TUMRGBDLoader::getLenght() {
    return (int)vTimeStamps_.size();
}

cv::Size TUMRGBDLoader::getImageSize() {
    return imSize_;
}
