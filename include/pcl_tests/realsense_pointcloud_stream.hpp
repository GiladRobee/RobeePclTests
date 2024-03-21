#ifndef REALSENSE_POINTCLOUD_STREAM_HPP
#define REALSENSE_POINTCLOUD_STREAM_HPP

#include <iostream>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class realsense_pointcloud_stream
{
    public:
    realsense_pointcloud_stream()
    {
        cloud_ =std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        // pipe_ = rs2::pipeline();
        pipe_.start();
        is_streaming_ = true;
    }
    ~realsense_pointcloud_stream()
    {   
        // if(is_streaming_)
        //     pipe_.stop();
    }
    realsense_pointcloud_stream start()
    {
        pipe_.start();
        is_streaming_ = true;
        return *this;
    }
    // realsense_pointcloud_stream stop()
    // {
    //     if(is_streaming_)
    //         pipe_.stop();
    //     return *this;
    // }
    realsense_pointcloud_stream pollFrame()
    {
        std::cout << "polling frame" << std::endl;
        frames_ = pipe_.wait_for_frames();
        std::cout << "got frame, calculate depth" << std::endl;
        depth_ = frames_.get_depth_frame();
        return *this;
    }
    realsense_pointcloud_stream depthToPc()
    {
        std::cout << "depth pointcloud" << std::endl;
        points_ = pc_.calculate(depth_);
        return *this;
    }
    realsense_pointcloud_stream pcToPoints()
    {
        std::cout << "pointcloud to points" << std::endl;
        auto sp = points_.get_profile().as<rs2::video_stream_profile>();
        cloud_->width = sp.width();
        cloud_->height = sp.height();
        cloud_->is_dense = false;
        cloud_->points.resize(points_.size());
        auto ptr = points_.get_vertices();
        std::cout << "setting pcl points" << std::endl;
        for(auto& p : cloud_->points)
        {
            p.x = ptr->x;
            p.y = ptr->y;
            p.z = ptr->z;
            p.r = 255;
            ptr++;
        }
        return *this;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud()
    {
        return cloud_;
    }
    private:
    bool is_streaming_;
    rs2::pointcloud pc_;
    rs2::points points_;
    rs2::pipeline pipe_;
    rs2::frameset frames_;
    rs2::frame depth_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
};

#endif // REALSENSE_POINTCLOUD_STREAM_HPP