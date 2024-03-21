#include <thread>
#include <iostream>
#include <chrono>
#include "pcl_tests/pcl_viewer.hpp"
#include "pcl_tests/pcd_loader.hpp"
#include "pcl_tests/realsense_pointcloud_stream.hpp"
#include "pcl_tests/argparse.hpp"

struct main_args : public argparse::Args
{
  bool &load = flag("l,load","Load a pcd file can't be set with realsense stream").set_default(false);
  std::string &filename = kwarg("f,filename","Filename of pcd file").set_default("Robee_tile_test_1.pcd");
  bool &stream = flag("s,stream","Stream from realsense, can't be set with load from pcd").set_default(false);
  bool &use_color = flag("c,color","Use color in point cloud").set_default(false);
  bool &verbose           = flag("v,verbose", "A flag to toggle verbose").set_default(false);
  bool &help              = flag("h,help", "Print this help message").set_default(false);
};


int main (int argc, char** argv)try
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
    main_args args = argparse::parse<main_args>(argc,argv);
    if(args.verbose){
      args.print();
    }
    std::cout << "starting point cloud viewer" << std::endl;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
    if(args.load && !args.stream)
    {
      std::cout << "loading" << std::endl;
      pcl_loader<pcl::PointXYZRGB> loader;
      loader.setFilename("../"+args.filename);
      loader.load();
      cloud = loader.getCloud();
    }else if(args.stream && !args.load)
    {
      std::cout << "from stream" << std::endl;
      realsense_pointcloud_stream stream(args.use_color);
      if(stream.isRunning())
        cloud =  stream.pollFrame().depthToPc().pcToPoints().getCloud();
    }
    else{
      std::cerr << "Invalid arguments" << std::endl;
      return 1;
    }
    std::cout << "setting up viewer with cloud size: "<<cloud->size() << std::endl;
    PCLViewer<pcl::PointXYZRGB> viewer(3);
    std::cout << "setting cloud" << std::endl;
    viewer.setCloud(cloud);
    std::cout << "viewing" << std::endl;
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
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
