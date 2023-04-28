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
 * A demo showing the Mini-SLAM library processing a sequence of the EuRoC dataset
 */

#include "DatasetLoader/EurocVisualLoader.h"
#include "System/MiniSLAM.h"

#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, char **argv){
    //Check program parameters are good
    if(argc != 3){
        cerr << "[Error]: you need to invoke the program with 2 parameters: " << endl;
        cerr << "\t./mono_euroc <dataset_path> <timestamps_file>" << endl;
        cerr << "Finishing execution..." << endl;
        return -1;
    }

    //Load dataset sequence
    string datasetPath = argv[1];
    string timestampsFile = argv[2];
    EurocVisualLoader sequence(datasetPath, timestampsFile, datasetPath + "/mav0/state_groundtruth_estimate0/data.csv");

    //Create SLAM system
    MiniSLAM SLAM("Data/EuRoC.yaml");

    //File to store the trajectory
    ofstream trajectoryFile ("trajectory.txt");
    if (!trajectoryFile.is_open()){
        cerr << "[Error]: could not open the trajectory file, aborting..." << endl;
        return -2;
    }

    //Process the sequence
    cv::Mat currIm;
    double currTs;
    for(int i = 150; i < sequence.getLenght(); i++){
        sequence.getLeftImage(i,currIm);
        sequence.getTimeStamp(i,currTs);

        Sophus::SE3f Tcw;
        if(SLAM.processImage(currIm, Tcw)){
            Sophus::SE3f Twc = Tcw.inverse();
            //Save predicted pose to the file
            trajectoryFile << setprecision(17) << currTs*1e9 << "," << setprecision(7) << Twc.translation()(0) << ",";
            trajectoryFile << Twc.translation()(1) << "," << Twc.translation()(2) << ",";
            trajectoryFile << Twc.unit_quaternion().x() << "," << Twc.unit_quaternion().y() << ",";
            trajectoryFile << Twc.unit_quaternion().z() << "," << Twc.unit_quaternion().w() << endl;
        }
    }

    trajectoryFile.close();

    return 0;
}