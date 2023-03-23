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
 * This class loads and stores the settings for the SLAM system
 */

#ifndef MINI_SLAM_SETTINGS_H
#define MINI_SLAM_SETTINGS_H

#include "Calibration/CameraModel.h"

#include <memory>

class Settings {
public:
    /*
     * Default constructor: sets everything to default values
     */
    Settings();

    /*
     * Constructor reading parameters from file
     */
    Settings(const std::string& configFile);

    /*
     * Ostream operator overloading to dump settings to the terminal
     */
    friend std::ostream& operator<<(std::ostream& output, const Settings& D);

    //Getter methods
    std::shared_ptr<CameraModel> getCalibration();
    std::vector<float> getDistortionParameters();
    int getImCols();
    int getImRows();

    int getFeaturesPerImage();
    int getNumberOfScales();
    float getScaleFactor();

    int getGridCols();
    int getGridRows();

    float getEpipolarTh();

    int getMatchingInitTh();
    int getMatchingGuidedTh();
    int getMatchingByProjectionTh();
    int getMatchingForTriangulationTh();
    int getMatchingFuseTh();

    int getMinCommonObs();

    float getMinCos();

private:
    //Camera parameters
    std::shared_ptr<CameraModel> calibration_;  //Geometric calibration with projection and unprojection functions
    std::vector<float> vDistortion_;                //[Optional] Image distortion parameters
    int imCols_, imRows_;                           //Image size

    //Feature extractor parameters
    int nFeatures_;                                 //Number of features to extract per image
    int nScales_;                                   //Number of scales in the image pyramid representation
    float fScaleFactor_;                            //Scale factor

    //Feature grid
    int nGridCols_, nGridRows_;                     //Feature grid size

    float fEpipolarTh_;                             //Epipolar threshold for the Essential matrix

    int nMatchingInitTh_;
    int nMatchingGuidedTh_;
    int nMatchingProjectionTh_;
    int nMatchingTriangulationTh_;
    int nMatchingFuseTh_;

    int nMinCommonObs_;

    float fMinCos_;
};


#endif //MINI_SLAM_SETTINGS_H
