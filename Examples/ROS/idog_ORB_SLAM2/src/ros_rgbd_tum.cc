/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/OccupancyGrid.h>

#include<opencv2/core/core.hpp>

#include"System.h"

#include "arrGrid.h"

using namespace std;


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void initGrid(nav_msgs::OccupancyGridPtr pOccuGrid, const string strSettingsFile);

void updateGrid(nav_msgs::OccupancyGridPtr pOccuGrid, arrGrid ArrGrid);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        ros::shutdown();
        return 1;
    }


    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<const nav_msgs::OccupancyGrid>("grid", 1);

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]); // argv[4] : path_to_association
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }


    nav_msgs::OccupancyGridPtr pOccuGrid( new nav_msgs::OccupancyGrid );
    initGrid(pOccuGrid, argv[2]);


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // argv[1] : path_to_vocabulary
    // argv[2] : path_to_settings
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD, pOccuGrid->data,false);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;



    cv::Mat imRGB, imD;

    for(int ni=0; ni<nImages; ni++)
    {
        if(ros::ok())
        {
            ros::spinOnce();

            // Read image and depthmap from file
            imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
            imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
            double tframe = vTimestamps[ni];

            if(imRGB.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
                return 1;
            }

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            // Pass the image to the SLAM system
            SLAM.TrackRGBD(imRGB,imD,tframe);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            if(SLAM.newGrid())
            {
                arrGrid arrayGrid = SLAM.getNav_array();
                updateGrid(pOccuGrid, arrayGrid);
                pub.publish(pOccuGrid);
                std::cout << "publish new grid!" << std::endl;
            }

            // Wait to load the next frame
            double T=0;
            if(ni<nImages-1)
                T = vTimestamps[ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestamps[ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6);



        }

    }

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}


void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            // UNKNOWN=> cannot understand the >> operator
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}


void initGrid(nav_msgs::OccupancyGridPtr pOccuGrid, const string strSettingsFile)
{
    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }
    pOccuGrid->header.seq = 1;
    pOccuGrid->header.frame_id = "map";
    pOccuGrid->info.origin.position.z = 0;
    pOccuGrid->info.origin.orientation.w = 1;
    pOccuGrid->info.origin.orientation.x = 0;
    pOccuGrid->info.origin.orientation.y = 0;
    pOccuGrid->info.origin.orientation.z = 0;

    int xCells, yCells;
    xCells = fsSettings["OccupiedGrid.xCells"];
    yCells = fsSettings["OccupiedGrid.yCells"];

    pOccuGrid->data.resize( xCells * yCells );
    std::fill_n(pOccuGrid->data.begin(), pOccuGrid->data.size(), -1);

    return;
}


void updateGrid(nav_msgs::OccupancyGridPtr pOccuGrid, arrGrid ArrGrid)
{
    pOccuGrid->header.seq++;
    pOccuGrid->header.stamp.sec = ros::Time::now().sec;
    pOccuGrid->header.stamp.nsec = ros::Time::now().nsec;
    pOccuGrid->info.map_load_time = ros::Time::now();
    pOccuGrid->info.resolution = ArrGrid.mCellResolution;
    pOccuGrid->info.width = ArrGrid.xCells;
    pOccuGrid->info.height = ArrGrid.yCells;
    pOccuGrid->info.origin.position.x = 0;
    pOccuGrid->info.origin.position.y = 0;
}
