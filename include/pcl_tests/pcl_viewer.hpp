#ifndef PCL_VIEWER_HPP
#define PCL_VIEWER_HPP

#include <iostream>
#include <thread>

#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

template<typename PointT>
class PCLViewer
{
    public:
    PCLViewer(int size_p) : size_p_(size_p),cloud_(nullptr),viewer_ (new pcl::visualization::PCLVisualizer ("3D viewer_"))
    {
        
    }
    ~PCLViewer() 
    {
        viewer_ = nullptr;
        cloud_ = nullptr;
    }

    void setCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        cloud_ = cloud;
    }

    void setSize(int size_p)
    {
        size_p_ = size_p;
    }

    bool wasStopped()
    {
        return viewer_->wasStopped();
    }

    void spinOnce()
    {
        viewer_->spin();
    }

    void simpleVis ()
    {
    // --------------------------------------------
    // -----Open 3D viewer_ and add point cloud_-----
    // --------------------------------------------
    if(size_p_ != 1)
    {
        std::cerr << "Size of point in point cloud_ is not 1" << std::endl;
        return;
    }
    viewer_->setBackgroundColor (0, 0, 0);
    viewer_->addPointCloud<PointT> (cloud_, "sample cloud_");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_p_, "sample cloud_");
    // viewer_->addCoordinateSystem (1.0);
    viewer_->initCameraParameters ();
    }


    void rgbVis ()
    {
    // --------------------------------------------
    // -----Open 3D viewer_ and add point cloud_-----
    // --------------------------------------------
    if(size_p_ != 3)
    {
        std::cerr << "Size of point in point cloud_ is not 3" << std::endl;
        return;
    }
    viewer_->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud_);
    viewer_->addPointCloud<PointT> (cloud_, rgb, "sample cloud_");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_p_, "sample cloud_");
    // viewer_->addCoordinateSystem ();
    viewer_->initCameraParameters ();
    }


    void customColourVis ()
    {
    // --------------------------------------------
    // -----Open 3D viewer_ and add point cloud_-----
    // --------------------------------------------
    if(size_p_ != 3)
    {
        std::cerr << "Size of point in point cloud_ is not 3" << std::endl;
        return;
    }
    viewer_->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color(cloud_, 0, 255, 0);
    viewer_->addPointCloud<PointT> (cloud_, single_color, "sample cloud_");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_p_, "sample cloud_");
    // viewer_->addCoordinateSystem (1.0);
    viewer_->initCameraParameters ();
    }


    void normalsVis (pcl::PointCloud<pcl::Normal>::ConstPtr normals)
    {
    // --------------------------------------------------------
    // -----Open 3D viewer_ and add point cloud_ and normals-----
    // --------------------------------------------------------
    if(size_p_ != 3)
    {
        std::cerr << "Size of point in point cloud_ is not 3" << std::endl;
        return;
    }
    viewer_->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud_);
    viewer_->addPointCloud<PointT> (cloud_, rgb, "sample cloud_");
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_p_, "sample cloud_");
    viewer_->addPointCloudNormals<PointT, pcl::Normal> (cloud_, normals, 10, 0.05, "normals");
    // viewer_->addCoordinateSystem (1.0);
    viewer_->initCameraParameters ();
    }

    private:
    int size_p_;
    typename pcl::PointCloud<PointT>::Ptr cloud_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;
};
#endif // PCL_VIEWER_HPP