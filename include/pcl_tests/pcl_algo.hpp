#ifndef PCL_ALGO_HPP_
#define PCL_ALGO_HPP_
#include <iostream>
#include <chrono>
#include <deque>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/experimental/functor_filter.h>

using point_t = pcl::PointXYZRGB;
using cloud_t = pcl::PointCloud<point_t>;

using time_point_t = std::chrono::time_point<std::chrono::system_clock>;

class pcl_algo
{
    
    public:
    pcl_algo()
    {}
    pcl_algo(std::shared_ptr<cloud_t>& cloud) : cloud_(cloud)
    {}
    std::shared_ptr<cloud_t> getCloud() const
    {
        return cloud_;
    }

    void setCloud(std::shared_ptr<cloud_t>& cloud)
    {
        cloud_ = cloud;
    }
    void filter_p()
    {
        filterCloud();
    }
    
    void segment_p()
    {
        plane_segmentation();
    }
    void hull_p(pcl::PointIndices::Ptr plane_inliers_, pcl::ModelCoefficients::Ptr plane_coeffs_, cloud_t::Ptr plane_cloud_)
    {
        compute_hull(plane_inliers_,plane_coeffs_,plane_cloud_);
    }

    void filter_camera_edges_p()
    {
        filter_camera_edges();
    }

    void segment_lines_p()
    {
        segment_lines();
    }

    void functor_filter_camera_edges_p()
    {
        functor_filter_camera_edges();
    }

    std::shared_ptr<cloud_t> getFilteredCloud() const
    {
        return filtered_cloud_;
    }
   
    std::deque<cloud_t::Ptr> getClusters() const
    {
        return clusters_;
    }
    cloud_t::Ptr getCluster(size_t i) const
    {
        if(i < clusters_.size())
            return clusters_[i];
        else
            throw std::out_of_range("Index out of range");
    }
    std::deque<pcl::ModelCoefficients::Ptr> getPlaneCoefficients() const
    {
        return plane_coefficients_;
    }
    pcl::ModelCoefficients::Ptr getPlaneCoefficient(size_t i) const
    {
        if(i < plane_coefficients_.size())
            return plane_coefficients_[i];
        else
            throw std::out_of_range("Index out of range");
    }
    std::deque<pcl::PointIndices::Ptr> getPlaneInliers() const
    {
        return plane_inliers_;
    }
    pcl::PointIndices::Ptr getPlaneInlier(size_t i) const
    {
        if(i < plane_inliers_.size())
            return plane_inliers_[i];
        else
            throw std::out_of_range("Index out of range");
    }
    cloud_t::Ptr getHullCloud() const
    {
        if(hull_cloud_ == nullptr)
            throw std::runtime_error("Hull cloud not computed");
        return hull_cloud_;
    }

    cloud_t::Ptr getEdgeCutCloud() const
    {
        if(edge_cut_cloud_ == nullptr)
            throw std::runtime_error("Edge cut cloud not computed");
        return edge_cut_cloud_;
    }

    std::deque<cloud_t::Ptr> getLines() const
    {
        return lines_;
    }
    cloud_t::Ptr getLine(size_t i) const
    {
        if(i < lines_.size())
            return lines_[i];
        else
            throw std::out_of_range("Index out of range");
    }
    std::deque<pcl::ModelCoefficients> getLineCoefficients() const
    {
        std::cout << "Number of line coefficients: " << line_coefficients_.size() << std::endl;
        return line_coefficients_;
    }
    pcl::ModelCoefficients getLineCoefficient(size_t i) const
    {
        if(i < line_coefficients_.size())
            return line_coefficients_[i];
        else
            throw std::out_of_range("Index out of range");
    }
    protected:
    void filterCloud()
    {
        cloud_t filtered_cloud_stage1;
        cloud_t filtered_cloud_stage_11;
        cloud_t filtered_cloud_stage_12;
        cloud_t filtered_cloud_stage2;
        pcl::PassThrough<point_t> pass,passX;
        pcl::StatisticalOutlierRemoval<point_t> sor;
        tick = std::chrono::system_clock::now();
        auto min_height = [this]()->double{
                double ret =DBL_MAX;
                std::for_each(cloud_->points.begin(), cloud_->points.end(), [&ret](const point_t& p){
                        if(p.z < 0.001)
                            return;
                        if(p.z < ret)
                            ret = p.z;
                        });
                return ret;
            }();
        tock = std::chrono::system_clock::now();
        std::cout << "min height: " << min_height << " took: " << comapreTime(tick, tock) << "[ms]" << std::endl;
        min_height += min_height_offset; // 4cm delta
        //-- filter stage 1
        tick = std::chrono::system_clock::now();
        pass.setInputCloud(cloud_);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.001, min_height);
        pass.filter(filtered_cloud_stage1);
        tock = std::chrono::system_clock::now();
        std::cout << "filter stage 1 took: " << comapreTime(tick, tock) << "[ms]" << std::endl;


        std::function<bool(const cloud_t&, pcl::index_t)> filter = [=](const cloud_t& fil_cloud,pcl::index_t i){
            return abs(atan2(fil_cloud.points[i].y,fil_cloud.points[i].z))<(camera_y_span-1*threshhold_filter_y) &&
                    abs(atan2(fil_cloud.points[i].x,fil_cloud.points[i].z))<(camera_x_span-1*threshhold_filter_x);
        };

        std::cout << "filtered cloud 1 size: " << filtered_cloud_stage1.size() << std::endl;
        //-- filter stage 1.1
        pcl::experimental::FilterFunction<point_t> filter_fun(filter);
        pcl::experimental::FunctionFilter<point_t> functor_filter(filter_fun);
        functor_filter.setInputCloud(std::make_shared<cloud_t>(filtered_cloud_stage1));
        functor_filter.filter(filtered_cloud_stage_11);
        std::cout << "filtered cloud 11 size: " << filtered_cloud_stage_11.size() << std::endl; 
        
        auto minmax = std:: minmax_element(filtered_cloud_stage_11.points.begin(),filtered_cloud_stage_11.points.end(),[](const point_t& p1, const point_t& p2){
            return p1.x < p2.x;
        });
        std::cout << "min x: " << minmax.first->x << " max x: " << minmax.second->x << std::endl;

        passX.setInputCloud(std::make_shared<cloud_t>(filtered_cloud_stage_11));
        passX.setFilterFieldName("x");
        passX.setFilterLimits(minmax.first->x , minmax.second->x );
        passX.filter(filtered_cloud_stage_12);
        std::cout << "filtered cloud 12 size: " << filtered_cloud_stage_12.size() << std::endl;
           

        //-- filter stage 2
        tick = std::chrono::system_clock::now();
        sor.setInputCloud(std::make_shared<cloud_t>(filtered_cloud_stage_12));
        sor.setMeanK(50);//test correct value
        sor.setStddevMulThresh(1.0);//test correct value
        sor.filter(filtered_cloud_stage2);
        tock = std::chrono::system_clock::now();
        std::cout << "filter stage 2 took: " << comapreTime(tick, tock) << "[ms]" << std::endl;
        filtered_cloud_ = std::make_shared<cloud_t>(filtered_cloud_stage2);
    }

    void plane_segmentation()
    {
        cloud_t::Ptr cloud_f(new cloud_t);
        cloud_t::Ptr cloud_s(new cloud_t);
        cloud_f->height = 1;
        cloud_f->is_dense = false;
        cloud_f->points.resize(filtered_cloud_->points.size());
        std::memcpy(cloud_f->points.data(), filtered_cloud_->points.data(), filtered_cloud_->points.size()*sizeof(point_t));
        clusters_.clear();


        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<point_t> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.005);

        auto nr_points = cloud_f->points.size();
        while(cloud_f->points.size() > 0.3*nr_points )
        {
            seg.setInputCloud(cloud_f);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
                return ;
            }
            cloud_t::Ptr cloud_p (new cloud_t);
            
            // Extract the inliers
            pcl::ExtractIndices<point_t> extract;
            extract.setInputCloud (cloud_f);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_p);
            clusters_.push_back(cloud_p);
            plane_coefficients_.push_back(coefficients);
            plane_inliers_.push_back(inliers);
            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_s);
            cloud_f.swap(cloud_s);
        }

        std::cout << "Number of clusters: " << clusters_.size() << std::endl;
        std::cout << "Number of points in clusters: " << std::accumulate(
            clusters_.begin(), clusters_.end(), 0, [](int sum, cloud_t::Ptr& c){return sum + c->points.size();}) 
            << "\nNumber of points in filtered cloud: "<<filtered_cloud_->points.size() <<std::endl;
        std::cout << "Plane Coefficients: " << coefficients->values[0] << " "
            << coefficients->values[1] << " "
            << coefficients->values[2] << " "
            << coefficients->values[3] << "\n";
        
    }

    void compute_hull(pcl::PointIndices::Ptr plane_inliers_, pcl::ModelCoefficients::Ptr plane_coeffs_, cloud_t::Ptr plane_cloud_)
    {
        cloud_t projected_cloud,concave_cloud,convex_cloud;
        pcl::ProjectInliers<pcl::PointXYZRGB> projection;
        pcl::ConcaveHull<pcl::PointXYZRGB> concave_hull;
        pcl::ConvexHull<pcl::PointXYZRGB> convex_hull;
    
        //*projection
        projection.setModelType (pcl::SACMODEL_PLANE);
        projection.setIndices (plane_inliers_);
        projection.setInputCloud (plane_cloud_);
        projection.setModelCoefficients (plane_coeffs_);
        projection.filter (projected_cloud);

        //concave
        concave_hull.setInputCloud(std::make_shared<cloud_t>(projected_cloud));
        concave_hull.setAlpha(0.01);
        concave_hull.reconstruct(concave_cloud);
        
        //convex
        convex_hull.setInputCloud (std::make_shared<cloud_t>(projected_cloud));
        convex_hull.reconstruct (convex_cloud);
        //reconstruct hull cloud
        hull_cloud_=std::make_shared<cloud_t>();
        hull_cloud_->reserve(convex_cloud.size()+concave_cloud.size());
        hull_cloud_->insert(hull_cloud_->begin(),convex_cloud.begin(),convex_cloud.end());
        hull_cloud_->insert(hull_cloud_->end(),concave_cloud.begin(),concave_cloud.end());

        std::cout << "Number of points in hull cloud: " << hull_cloud_->points.size() << std::endl;
        std::for_each(hull_cloud_->points.begin(), hull_cloud_->points.end(), []( point_t& p){
            p.g = 255;
            p.r = 0;
            p.b = 0;
        });

    }

    void filter_camera_edges()
    {
        pcl::PointIndices::Ptr ind_to_filter = std::make_shared<pcl::PointIndices>();
        edge_cut_cloud_ = std::make_shared<cloud_t>();
            if(!hull_cloud_->size())
                throw std::runtime_error("Hull cloud not computed");
            for(size_t i  = 0; i < (*hull_cloud_).size(); i++)
            {
                //  auto pr = projected_z(filt->points[i]);
                auto prx = atan2(hull_cloud_->points[i].x,hull_cloud_->points[i].z);
                auto pry = atan2(hull_cloud_->points[i].y, hull_cloud_->points[i].z);
            
                if(std::abs(prx)>(camera_x_span-threshhold_filter_x) || std::abs(pry)>(camera_y_span-threshhold_filter_y))
                    ind_to_filter->indices.push_back(i);
            }

            pcl::ExtractIndices<pcl::PointXYZRGB> ext_edg;
            ext_edg.setInputCloud(hull_cloud_);
            ext_edg.setIndices(ind_to_filter);
            ext_edg.setNegative(true);

            ext_edg.filter(*edge_cut_cloud_);
      
    }

    void functor_filter_camera_edges()
    {
        edge_cut_cloud_ = std::make_shared<cloud_t>();
        std::function<bool(const cloud_t&, pcl::index_t)> filter_f = [=](const cloud_t& fil_cloud,pcl::index_t i){
            // std::cout << "angle_y: " << atan2(fil_cloud.points[i].y,fil_cloud.points[i].z) <<std::endl;
            return abs(atan2(fil_cloud.points[i].y,fil_cloud.points[i].z))<(camera_y_span-1.32*threshhold_filter_y) && 
                    abs(atan2(fil_cloud.points[i].x,fil_cloud.points[i].z))<(camera_x_span-1.32*threshhold_filter_x);
        };

        //-- filter stage 1.1
        pcl::experimental::FilterFunction<point_t> filter_fun(filter_f);
        pcl::experimental::FunctionFilter<point_t> functor_filter(filter_fun);
        functor_filter.setInputCloud(hull_cloud_);
        functor_filter.filter(*edge_cut_cloud_);
    }

    void segment_lines()
    {
        cloud_t::Ptr cloud_f(new cloud_t);
        cloud_t::Ptr cloud_s(new cloud_t);
        cloud_f->height = 1;
        cloud_f->is_dense = false;
        cloud_f->points.resize(edge_cut_cloud_->points.size());
        std::memcpy(cloud_f->points.data(), edge_cut_cloud_->points.data(), edge_cut_cloud_->points.size()*sizeof(point_t));
        lines_.clear();


        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<point_t> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_LINE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.005);

        auto nr_points = cloud_f->points.size();
        while(cloud_f->points.size() > 0.3*nr_points )
        {
            seg.setInputCloud(cloud_f);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
                return ;
            }
            cloud_t::Ptr cloud_p (new cloud_t);
            
            // Extract the inliers
            pcl::ExtractIndices<point_t> extract;
            extract.setInputCloud (cloud_f);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_p);
            lines_.push_back(cloud_p);
            line_coefficients_.push_back(*coefficients);
            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_s);
            cloud_f.swap(cloud_s);
        }

        std::cout << "Number of clusters: " << lines_.size() << std::endl;
        std::cout << "Number of points in clusters: " << std::accumulate(
            lines_.begin(), lines_.end(), 0, [](int sum, cloud_t::Ptr& c){return sum + c->points.size();}) 
            << "\nNumber of points in filtered cloud: "<<filtered_cloud_->points.size() <<std::endl;
        std::cout << "line Coefficients: " << coefficients->values[0] << " "
            << coefficients->values[1] << " "
            << coefficients->values[2] << " "
            << coefficients->values[3] << " "
            << coefficients->values[4] << " "
            << coefficients->values[5] <<
            "\n";
    }

    

    double comapreTime(time_point_t tick, time_point_t tock)
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(tock - tick).count();
    }
    cloud_t::Ptr cloud_;
    cloud_t::Ptr filtered_cloud_;
    cloud_t::Ptr hull_cloud_;
    cloud_t::Ptr edge_cut_cloud_;
    time_point_t tick;
    time_point_t tock;
    const double min_height_offset = 0.01;
    const double threshhold_filter_y = 0.07;
    const double threshhold_filter_x = 3*threshhold_filter_y;
    const double camera_x_span = 0.7592182;
    const double camera_y_span = 0.506145;
    std::deque<cloud_t::Ptr> clusters_;
    std::deque<cloud_t::Ptr> lines_;
    std::deque<pcl::ModelCoefficients::Ptr> plane_coefficients_;
    std::deque<pcl::ModelCoefficients> line_coefficients_;
    std::deque<pcl::PointIndices::Ptr> plane_inliers_;  
};

#endif //PCL_ALGO_HPP_