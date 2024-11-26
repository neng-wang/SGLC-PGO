

#pragma once

#include <vector>
#include <limits>
#include <iostream>
#include <Eigen/Core>
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Coreutils.hpp" 


namespace SGLC
{
    // for view
    template<typename T> 
    std::string toString(const T& t) {
        std::ostringstream oss;
        oss << t;
        return oss.str();
    }

    struct PointAPR{
        double azimuth;
        double polar_angle;
        double range;
    };

    struct Voxel{
        bool haspoint = false;
        int cluster = -1;
        std::vector<int> index;
    };

    class SemanticCluster
    {
        private:
            double deltaA_ = 2; //z轴夹角分辨率,单位：度
            double deltaR_ = 0.5; //xoy平面的半径，单位:m
            double deltaP_ = 1.2; //与x轴正方向的夹角分辨率，单位:度

            double min_range_ = std::numeric_limits<double>::max();
            double max_range_ = std::numeric_limits<double>::min();
            double min_azimuth_ = -24.8 * M_PI/180;
            double max_azimuth_ = 2 * M_PI/180;
            int length_ = 0;
            int width_  = 0;
            int height_ = 0;

    public:
        SemanticCluster(std::vector<double>& param)
        {
            if(param.size() != 3){
                printf("Param number is not correct!");
                std::abort();		
            }
            for(size_t i=0; i<param.size(); ++i){
                deltaA_ = param[0];
                deltaR_ = param[1];
                deltaP_ = param[2];
            }
        }
    
        ~SemanticCluster(){}

        std::vector<PointAPR> calculateAPR(const std::vector<Eigen::Vector3d>& cloud_IN);
        std::unordered_map<int, Voxel> build_hash_table(const std::vector<PointAPR>& vapr);
        std::vector<int> cluster(std::unordered_map<int, Voxel>& map_in,
                                 const std::vector<PointAPR>& vapr);
        void find_neighbors(int polar,
                                     int range,
                                     int azimuth,
                                     std::vector<int>& neighborindex);
        void mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2);

        std::vector<int> remove_noise(std::vector<int> values);
    };

    std::vector<Bbox> Cluster(const std::vector<Eigen::Vector3d>& frame,
                                  const std::vector<int>& frame_label,
                                  double delta_azimuth,
                                  double delta_range,
                                  double delta_polar,
                                  bool view);
    std::vector<Bbox> ClusterPoints(const std::vector<Eigen::Vector3d>& frame,
                          const std::vector<int>& frame_label,
                          std::pair<std::vector<Eigen::Vector3d>,std::vector<int>>& background_points,
                          std::pair<std::vector<Eigen::Vector3d>,std::vector<int>>& ClusterSemPoints,
                          double delta_azimuth,
                          double delta_range,
                          double delta_polar,
                          bool view);

} // namespace SGLC
