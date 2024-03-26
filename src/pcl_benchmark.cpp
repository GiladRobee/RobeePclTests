#include "pcl_tests/pcl_algo.hpp"
#include "benchmark/cppbenchmark.h"
#include "pcl_tests/pcd_loader.hpp"

class pcl_BM 
{
    public:
    pcl_BM()
    {
        algo_ = std::make_shared<pcl_algo>();
        for(int i = 1; i < 11; i++)
        {
            loader.setFilename("../pcd_files/Robee_tile_test_"+std::to_string(i)+".pcd");
            loader.load();
            cloud_v_.push_back(loader.getCloud());
        }

    }
    protected:    
    std::vector<cloud_t::Ptr> cloud_v_;
    std::shared_ptr<pcl_algo> algo_;
    pcl_loader<pcl::PointXYZRGB> loader;

};

BENCHMARK_FIXTURE(pcl_BM,"filter",Settings().Attempts(5).Operations(5))
{

        algo_->setCloud(cloud_v_[0]);
        algo_->filter_p();

}

BENCHMARK_MAIN()

