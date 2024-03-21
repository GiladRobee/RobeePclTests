#ifndef REALSENSE_POINTCLOUD_STREAM_HPP
#define REALSENSE_POINTCLOUD_STREAM_HPP

#include <iostream>
#include <librealsense2/rs.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
class realsense_pointcloud_stream
{
    public:
    realsense_pointcloud_stream()
    {
        cloud_ =std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        // pipe_ = rs2::pipeline();
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
        pipe_.start(cfg);
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
        color_frame_ = frames_.get_color_frame();
        return *this;
    }
    realsense_pointcloud_stream depthToPc()
    {
        std::cout << "depth pointcloud" << std::endl;
        pc_.map_to(color_frame_);
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
        auto color_ptr = points_.get_texture_coordinates();

        std::cout << "setting pcl points" << std::endl;
        for(auto& p : cloud_->points)
        {
            if(ptr->z ){
                p.x = ptr->x;
                p.y = ptr->y;
                p.z = ptr->z;
                auto xy_raw = *color_ptr;
                int x = std::min(sp.width() - 1, std::max(0, static_cast<int>(xy_raw.u * sp.width() + .5f)));
                int y = std::min(sp.height() - 1, std::max(0, static_cast<int>(xy_raw.v * sp.height() + .5f)));
                std::cout<<"computing color at "<<x<<", "<<y<<"\n";
                cv::Mat mat_col = cv::Mat(cv::Size(sp.width(), sp.width()),CV_8UC3, (void*)color_frame_.get_data());
                p.r = mat_col.at<cv::Vec3b>(y,x)[0];
                p.g = mat_col.at<cv::Vec3b>(y,x)[1];
                p.b = mat_col.at<cv::Vec3b>(y,x)[2];
            }
            color_ptr++;
            ptr++;
        }
        return *this;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud()
    {
        return cloud_;
    }

    bool isRunning()
    {
        return is_streaming_;
    }
    private:
    bool is_streaming_;
    rs2::pointcloud pc_;
    rs2::points points_;
    rs2::pipeline pipe_;
    rs2::frameset frames_;
    rs2::frame depth_;
    rs2::frame color_frame_ ;
    rs2::config cfg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
};

#endif // REALSENSE_POINTCLOUD_STREAM_HPP