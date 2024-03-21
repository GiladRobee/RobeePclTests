#ifndef PCL_LOADER_HPP
#define PCL_LOADER_HPP
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
template<typename PointT>
class pcl_loader
{
public:
pcl_loader() : cloud_(new pcl::PointCloud<PointT>())
{
}
~pcl_loader()
{
    cloud_ = nullptr;
}
void setFilename(std::string filename)
{
    filename_ = filename;
}
bool load()
{
    
    if ( pcl::io::loadPCDFile(filename_, *cloud_) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file  \n");
        return false;
    }
    std::cout << "Loaded "
                << cloud_->width * cloud_->height
                << " data points from "<< filename_<<"  with the following fields: "
                << std::endl;
    return true;
}
typename pcl::PointCloud<PointT>::Ptr getCloud()
{
    return cloud_;
}
private:
typename pcl::PointCloud<PointT>::Ptr cloud_;
std::string filename_;
};

#endif // PCL_LOADER_HPP