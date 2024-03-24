#include <iostream>
#include "pcl_tests/pcl_algo.hpp"
#include "pcl_tests/pcd_loader.hpp"
#include <pcl_tests/pcl_viewer.hpp>
#include <pcl_tests/argparse.hpp>
#include <pcl/io/pcd_io.h>
#define Debug(x) std::cout << "debug: " << x << std::endl;
enum output_cloud{
filtered_cloud,
};

struct main_args : public argparse::Args
{
    output_cloud &filter            = kwarg("f,filter","show filtered point cloud");
    bool &help              = flag("h,help", "Print this help message").set_default(false);
    bool &verbose           = flag("v,verbose", "A flag to toggle verbose").set_default(false);
};

int main()
{
    pcl_algo algo;
    pcl_loader<pcl::PointXYZRGB> loader;
    loader.setFilename("../2024-03-24.16:54:39.pcd");
    loader.load();
    auto loaded_cloud = loader.getCloud();
    algo.setCloud(loaded_cloud);
    algo.filter_p();
    auto filtered_cloud = algo.getFilteredCloud();
    algo.segment_p();
    auto cluster_0 = algo.getCluster(0);
    auto model_coefficients = algo.getPlaneCoefficient(0);
    auto plane_inliers = algo.getPlaneInlier(0);
    algo.hull_p(plane_inliers,model_coefficients,cluster_0);
    auto hull = algo.getHullCloud();
    // algo.filter_camera_edges_p();
    algo.functor_filter_camera_edges_p();
    auto filtered_camera_edges_cloud = algo.getEdgeCutCloud();
    algo.segment_lines_p();
    auto lines = algo.getLines();
    auto lines_coefs = algo.getLineCoefficients();
    //-----------------------------------output-----------------------------------//
    
    PCLViewer<point_t> viewer(0.05);
    // viewer.setCloud(filtered_cloud);
    viewer.setCloud(loaded_cloud);
    // viewer.setCloud(filtered_cloud);
    // viewer.setCloud(cluster_0);
    // viewer.setCloud(hull);
    // viewer.setCloud(filtered_camera_edges_cloud);
    // viewer.setCloud(lines[0]);
    // viewer.setCloud(lines[1]);

    viewer.drawLines(lines_coefs);
    viewer.rgbVis();
    while (!viewer.wasStopped ())
    {
        std::cout << "spinning" << std::endl;
        viewer.spinOnce ();
        std::cout << "spun" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "slept" << std::endl;
    }
    return 0;
}