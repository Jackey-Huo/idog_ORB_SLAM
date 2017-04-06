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

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/OccupancyGrid.h>

#include<opencv2/core/core.hpp>

#include"System.h"

#include "arrGrid.h"

using namespace std;

void initGrid(nav_msgs::OccupancyGridPtr pOccuGrid, const string strSettingsFile);

void updateGrid(nav_msgs::OccupancyGridPtr pOccuGrid, arrGrid ArrGrid);


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // construct navigation vector, used by ORB_SLAM system
    nav_msgs::OccupancyGridPtr pOccuGrid( new nav_msgs::OccupancyGrid );
    initGrid(pOccuGrid, argv[2]);



    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD, pOccuGrid->data, true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::Publisher pub = nh.advertise<const nav_msgs::OccupancyGrid>("grid", 1);

    //ros::spin();
    while(true)
    {
        if(ros::ok())
        {
            ros::spinOnce();

            if(SLAM.newGrid())
            {
                arrGrid arrayGrid = SLAM.getNav_array();
                updateGrid(pOccuGrid, arrayGrid);
                pub.publish(pOccuGrid);
                std::cout << "publish new grid!" << std::endl;
            }
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
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
