#include "pcl_tests/pcl_viewer.hpp"
#include "pcl_tests/pcd_loader.hpp"
#include <thread>
#include <iostream>
#include <chrono>
void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "\n\n";
}

int main (int argc, char** argv)
{
     // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return 0;
  }
    pcl_loader<pcl::PointXYZRGB> loader;
    loader.setFilename("/home/gilad/dev_ws/pcl_tests/Robee_tile_test_1.pcd");
    loader.load();
    auto  cloud = loader.getCloud();
    PCLViewer<pcl::PointXYZRGB> viewer(3);
    viewer.setCloud(cloud);
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
