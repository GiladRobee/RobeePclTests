#ifndef PCL_VIEWER_HPP
#define PCL_VIEWER_HPP


#include <pcl_tests/commons.hpp>

class PCLViewer
{
using PointT = pcl::PointXYZRGB;
using cloud_t = pcl::PointCloud<PointT>;
    public:
    PCLViewer(int size_p) : size_p_(size_p),offset_(0),cloud_(nullptr),viewer_ (new pcl::visualization::PCLVisualizer ("3D viewer_"))
    {
        
    }
    ~PCLViewer() 
    {
        viewer_ = nullptr;
        cloud_ = nullptr;
    }

    void setCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
    {
        auto in_cloud = cloud;
        if(cloud_ == nullptr)
            cloud_ = in_cloud;
        else
            concentratePointCloudsWithOffset(in_cloud,offset_);
        ++offset_;
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

    void drawLines(std::deque<pcl::ModelCoefficients> lines)
    {           
        auto line_0_rev = lines[0];
        auto line_1_rev = lines[1];
        auto line_0     = lines[0];
        auto line_1     = lines[1];
        for(int j = 3; j < lines[0].values.size(); j++)
        {
            line_0_rev.values[j] *= -1;
            line_1_rev.values[j] *= -1;
        }
        for(int i = offset_ -1 ; i >= 0; i--)
        {
            
            line_0_rev.values[2] = lines[0].values[2]+i;
            line_1_rev.values[2] = lines[1].values[2]+i;
            line_0.values[2]=lines[0].values[2]+i;
            line_1.values[2]=lines[1].values[2]+i;
            viewer_->addLine(line_0,"line"+std::to_string(0)+std::to_string(i));
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "line"+std::to_string(0)+std::to_string(i));
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.1, "line"+std::to_string(0)+std::to_string(i));
            viewer_->addLine(line_0_rev,"line"+std::to_string(10)+std::to_string(i));
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "line"+std::to_string(10)+std::to_string(i));
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.1, "line"+std::to_string(10)+std::to_string(i));
            viewer_->addLine(line_1_rev,"line"+std::to_string(1)+std::to_string(i));
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line"+std::to_string(1)+std::to_string(i));
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.1, "line"+std::to_string(1)+std::to_string(i));
            viewer_->addLine(line_1,"line"+std::to_string(11)+std::to_string(i));
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "line"+std::to_string(11)+std::to_string(i));
            viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.1, "line"+std::to_string(11)+std::to_string(i));

        }
    }

    void viewTwoClouds(cloud_t::Ptr cloud_1,cloud_t::Ptr cloud_2)
    {
        for(auto& point: cloud_2->points)
        {
            point.g= 0;
            point.r =0;
            point.b = 255;
            point.a = 255;
        }
        cloud_t::Ptr out_cloud(new cloud_t());
        out_cloud->points.resize(cloud_1->points.size() + cloud_2->points.size());
        out_cloud->points.insert(out_cloud->points.end(), cloud_1->points.begin(), cloud_1->points.end());
        out_cloud->points.insert(out_cloud->points.end(), cloud_2->points.begin(), cloud_2->points.end());

        viewer_->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(out_cloud);
        viewer_->addPointCloud<PointT> (out_cloud, rgb, "sample cloud_");
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud_");

    // viewer_->addCoordinateSystem ();
    viewer_->initCameraParameters ();

    }

    void concentratePointClouds(cloud_t::Ptr cloud)
    {
        cloud_->points.resize(cloud_->points.size() + cloud->points.size());
        cloud_->points.insert(cloud_->points.end(), cloud->points.begin(), cloud->points.end());
    }

    void concentratePointCloudsWithOffset(cloud_t::Ptr cloud, int offset)
    {
        auto in_cloud = cloud;
        for(auto& point : in_cloud->points)
        {
            point.z += offset_;
        }
        concentratePointClouds(in_cloud);
    }

    private:
    int size_p_,offset_;
    cloud_t::Ptr  cloud_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;
};
#endif // PCL_VIEWER_HPP