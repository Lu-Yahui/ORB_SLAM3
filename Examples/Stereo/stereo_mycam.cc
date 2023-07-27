/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

void GrabImages(cv::Mat& left, cv::Mat& right, double& timestamp_sec, cv::VideoCapture& cap)
{
    cv::Mat frame;
    cap >> frame;
    auto t = std::chrono::system_clock::now();
    timestamp_sec = 1E-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
}

int main(int argc, char **argv)
{  
    bool isLive = false;
    cv::VideoCapture cap;
    if (argc == 3)
    {
        isLive = true;
        
        int deviceID = 0;
        int apiID = cv::CAP_ANY;
        cap.open(deviceID);

        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        
        if (!cap.isOpened()) {
            cerr << "ERROR! Unable to open camera\n";
            return -1;
        }
    }

    if(!isLive && argc != 5)
    {
        cerr << endl << "Usage: ./stereo_mycam path_to_vocabulary path_to_settings path_to_images path_to_times_file" << endl;

        return 1;
    }

    // Load all sequences:
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestampsCam;
    int nImages;

    if (!isLive)
    {
        cout << "Loading images..." << std::endl;

        string pathCam0 = argv[3];
        string pathCam1 = argv[3];
        string pathTimeStamps = argv[4];

        LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft, vstrImageRight, vTimestampsCam);
        cout << "LOADED!" << endl;

        nImages = vstrImageLeft.size();
    }

    cout << endl << "-------" << endl;
    cout.precision(17);

    string vocabFile = argv[1];
    string settingYaml = argv[2];

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocabFile,settingYaml,ORB_SLAM3::System::STEREO, true);

    cv::Mat imLeft, imRight;

    double t_resize = 0;
    double t_rect = 0;
    double t_track = 0;
    int num_rect = 0;
    int proccIm = 0;
    int ni = 0;
    while (true)
    {
        if (!isLive && ni >= nImages)
        {
            break;
        }

        double tframe;
        if (isLive)
        {
            GrabImages(imLeft, imRight, tframe, cap);
        }
        else
        {
            std::cout << "[" << ni << "] Left: " << vstrImageLeft[ni] << ", Right: " << vstrImageRight[ni] << std::endl;

            // Read left and right images from file
            imLeft = cv::imread(vstrImageLeft[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
            tframe = vTimestampsCam[ni];
        }

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image" << endl;
            return 1;
        }

        if(imRight.empty())
        {
            cerr << endl << "Failed to load image" << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe, vector<ORB_SLAM3::IMU::Point>(), vstrImageLeft[ni]);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
        t_track = t_resize + t_rect + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        if (!isLive)
        {
            // Wait to load the next frame
            double T=0;
            if(ni<nImages-1)
                T = vTimestampsCam[ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6
            
            ni++;
            proccIm++;
        }
    }
    
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    SLAM.SaveMapPoints("map.txt");

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/cam0_" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/cam1_" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}
