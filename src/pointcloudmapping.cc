/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/common/projection_matrix.h>
#include "Converter.h"
#include "MapPoint.h"

#include <boost/make_shared.hpp>

PointCloudMapping::PointCloudMapping(double resolution_, vector<int8_t> & vDataGrid,
        int xCells, int yCells, double x_bias, double y_bias, double cell_resolution):
    mArrGrid(xCells, yCells, x_bias, y_bias, cell_resolution, vDataGrid)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);

    mpCoefficients = boost::make_shared<pcl::ModelCoefficients>(  );
    globalMap = boost::make_shared< PointCloud >( );
    bNewGrid = false;

    bCoefficientDown = false;
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );

    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=3 )
    {
        for ( int n=0; n<depth.cols; n+=3 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;

            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(p);
        }
    }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;

    if(bCoefficientDown)
    {
        PointCloud::Ptr tmp_ground_cloud( new PointCloud );

        // filter ground
        pcl::ModelOutlierRemoval<PointT> ground_filter;
        ground_filter.setModelCoefficients (*mpCoefficients);
        ground_filter.setThreshold (0.05);
        ground_filter.setModelType (pcl::SACMODEL_PLANE);
        ground_filter.setInputCloud (cloud);
        ground_filter.filter (*tmp_ground_cloud);

        Eigen::Vector3d normal(mpCoefficients->values[0], mpCoefficients->values[1], mpCoefficients->values[2]);

        Eigen::Matrix3d R = Eigen::Quaterniond::FromTwoVectors( normal, Eigen::Vector3d::UnitZ() ).toRotationMatrix();

        for(const auto &i : tmp_ground_cloud->points)
        {
            Eigen::Vector3d pos(i.x, i.y, i.z);
            Eigen::Vector3d res = R*pos;
            int x = ( res[0] + mArrGrid.x_bias ) / mArrGrid.mCellResolution, y = ( res[1] + mArrGrid.y_bias ) / mArrGrid.mCellResolution;
            if( !(0<=x && x<mArrGrid.xCells && 0<=y && y<mArrGrid.yCells) )
                {
                std::cerr << "\n" << "\033[1;31m"
                    << "At PointCloudMapping.generatePointCloud: invalid x, y value: x = "
                    << x << ", y = " << y << "\n"
                    << "bias x,y may need to be adjusted"
                    << "\033[0m" << std::endl;

                exit(-1);
            }
            //std::cout << "At PointCloudMapping.generatePointCloud: x, y value: x = "
                //<< x << ", y = " << y << ", z = " << res[2] << std::endl;

            mArrGrid.GridDataVector[y*mArrGrid.xCells + x] = 100;
        }

        bNewGrid = true;

    }

    cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}

// find ground pos and update the mpCoefficients param
void PointCloudMapping::segmentGround()
{
    if(!mpCoefficients->values.empty())
    {
        mpCoefficients->values.clear();
    }

    pcl::PointIndices::Ptr groud_inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.05);

    seg.setInputCloud (globalMap);
    seg.segment (*groud_inliers, *mpCoefficients);

    bCoefficientDown = true;
}

void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }

        static int count = 0;
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            count++;
            if(count == 7)
            {
                segmentGround();
            }
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
            *globalMap += *p;
        }
        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( globalMap );
        voxel.filter( *tmp );
        globalMap->swap( *tmp );
        viewer.showCloud( globalMap );
        cout << "show global map, size=" << globalMap->points.size() << endl;
        lastKeyframeSize = N;
    }
}


void PointCloudMapping::saveCurrentPointCloud()
{
    pcl::io::savePCDFileBinary( "current_bin_pcd.pcd", *globalMap );

    std::cerr << "Saved " << globalMap->size() << " data points to current_pcd.pcd." << std::endl;

    return;
}

bool PointCloudMapping::newGrid()
{
    bool res = bNewGrid;
    bNewGrid = false;
    return res;
}
