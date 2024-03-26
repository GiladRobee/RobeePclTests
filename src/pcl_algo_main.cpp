#include <pcl_tests/commons.hpp>
#include "pcl_tests/pcl_algo.hpp"
#include "pcl_tests/pcd_loader.hpp"
#include <pcl_tests/pcl_viewer.hpp>
#include <pcl_tests/argparse.hpp>
#include <pcl_tests/realsense_pointcloud_stream.hpp>



struct main_args : public argparse::Args
{
    output_cloud &filter            = kwarg("f,filter","show filtered point cloud").set_default(output_cloud::lines);
    bool &help              = flag("h,help", "Print this help message").set_default(false);
    bool &verbose           = flag("v,verbose", "A flag to toggle verbose").set_default(false);
    bool &load              = flag("l,load","Load a pcd file").set_default(true);
    std::string &filename   = kwarg("p,path","path of pcd file").set_default("2024-03-25.11:25:29.pcd");
    bool &stream            = flag("s,stream","Stream from realsense").set_default(false);
    bool &use_color         = flag("c,color","Use color in point cloud").set_default(false);
    bool &save_cloud        = flag("sc,save_cloud","Save the cloud to a pcd file").set_default(false);
    int &debug            = kwarg("d,debug","Debug mode").set_default(spdlog::level::info);
};

int main(int argc, char** argv)
{
    //-- parse command line arguments
    main_args args = argparse::parse<main_args>(argc,argv);
    if(args.verbose){
      args.print();
    }
    spdlog::set_level(static_cast<spdlog::level::level_enum>(args.debug));
    spdlog::set_pattern("[%H:%M:%S %z] [%^%L%$] [thread %t] %v");
    spdlog::info("starting point cloud viewer, with log level: {}",spdlog::level::to_string_view(spdlog::get_level()));
    cloud_t::Ptr loaded_cloud;
    pcl_algo algo;
    if(args.load && !args.stream)
    {
        pcl_loader<pcl::PointXYZRGB> loader;
        loader.setFilename("../pcd_files/"+args.filename);
        loader.load();
         loaded_cloud = loader.getCloud();
        algo.setCloud(loaded_cloud);
    }else{
        realsense_pointcloud_stream stream(args.use_color);
        if(stream.isRunning()){
         loaded_cloud =  stream.pollFrame().depthToPc().pcToPoints().getCloud();
        }else{
            std::cerr << "Stream not running" << std::endl;
            return 1;
        }
    }
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
    
    PCLViewer viewer(0.05);
    // viewer.setCloud(filtered_cloud);
    viewer.setCloud(loaded_cloud);
    viewer.setCloud(filtered_cloud);
    viewer.setCloud(cluster_0);
    viewer.setCloud(hull);
    // viewer.setCloud(filtered_camera_edges_cloud);
    // viewer.setCloud(lines[0]);
    // viewer.setCloud(lines[1]);

    // viewer.drawLines(lines_coefs);
    viewer.rgbVis();
    // viewer.viewTwoClouds(filtered_cloud,hull);

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