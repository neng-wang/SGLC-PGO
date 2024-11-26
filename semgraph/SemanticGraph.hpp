


#pragma once

#include <vector>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>
#include <tbb/parallel_for.h>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tsl/robin_map.h>

#include "Coreutils.hpp" 
#include "SemanticCluster.hpp"
#include "Hungarian.hpp"
#include "FastIcp.hpp"
#include "PlaneIcp.hpp"

#include "KDTreeVectorOfVectorsAdaptor.h"
#include "nanoflann.hpp"
namespace SGLC
{
    class SemanticGraph
    {
    private:
        struct VoxelHash{
            size_t operator()(const Eigen::Vector3i &voxel) const {
                const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
                return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
                }
            }; 
        struct VoxelBlock {
        // buffer of points with a max limit of n_points
            std::vector<Eigen::Vector3d> points;
            inline void AddPoint(const Eigen::Vector3d &point) {
                points.push_back(point);
            }
        };
        using InvDesTree = KDTreeVectorOfVectorsAdaptor< std::vector<std::vector<float>>, float >;
        bool show=false;
        bool remap=true;
        bool cluster_view = false;

 
        double deltaA = 2; //z轴夹角分辨率,单位：度 
        double deltaR = 0.35; //xoy平面的半径，单位:m 
        double deltaP = 1.2; //与x轴正方向的夹角分辨率，单位:度

        // graph
        double edge_th = 5;
        double sub_interval = 30;

        int graph_node_dimension = 60;
        int max_icp_iteration = 500;
        double estimation_threshold = 0.0001;
        double max_distance_for_loop = 0.07;

        double max_dis=50;
        double min_dis=5;
        int rings=24;
        int sectors=360;
        int sectors_range=360;
        int global_des_dim = 231;

        
        std::vector<int> order_vec = {0, 22, 0, 0, 21, 20, 0, 0, 0, 10, 11, 12, 13, 15, 16, 14, 17, 9, 18, 19, 0, 0, 0, 0, 0, 0};
        YAML::Node learning_map;
        std::vector<int> label_map;
        typedef std::tuple<u_char, u_char, u_char> Color;
        std::map<uint32_t, Color> _color_map, _argmax_to_rgb;
        std::shared_ptr<pcl::visualization::CloudViewer> viewer;
    public:
        int key_frame_idx = 0;
        double total_time = 0;
        double time_gen_des = 0.0;
        double time_search = 0.0;
        std::vector<std::vector<float>>  global_des_vec;
        std::vector<V3d_i> cloud_foreground;
        std::vector<V3d_i> cloud_background;
        std::vector<V3d_i> current_nodes_vec;
        std::vector<Graph> global_graph;
        

        std::vector<std::pair<int,int>> loop_pair_vec;
        std::vector<Eigen::Matrix4d> loop_trans_vec;
        std::vector<std::pair<V3d_i,V3d_i>> correspondence_node;
        int search_results_num = 5;
        int num_exclude_curr = 30;
        SemanticGraph(std::string conf_file);
        ~SemanticGraph();


        V3d_i loadCloud(std::string file_cloud, std::string file_label);
        
        SGLC::Graph build_graph(std::vector<SGLC::Bbox> cluster_boxes, double edge_dis_th,double subinterval);
        Eigen::MatrixXf MatrixDecomposing(Eigen::MatrixXf MatrixInput,int Dimension);
        std::vector<float> gen_graph_descriptors(SGLC::Graph graph,float edge_th,int subinterval);
        
        std::vector<float> gen_scan_descriptors(V3d_i cloud);

        bool geometry_poses_estimation(
                SGLC::Graph graph_q,
                SGLC::Graph graph_c,
                V3d_i cloud_fore_q,
                V3d_i cloud_fore_c,
                V3d_i cloud_back_q,
                V3d_i cloud_back_c,
                Eigen::Matrix4d& transform
        );

        bool search_loop();

        double get_cos_simscore(const std::vector<float> vec1, const std::vector<float> vec2);
        double getMold(const std::vector<float> vec);
        double get_cos_simscore(const std::vector<int> vec1, const std::vector<int> vec2);
        double getMold(const std::vector<int> vec);
        double get_alignment_score(SGLC::Graph graph1, SGLC::Graph graph2, Eigen::Isometry3d trans);
        std::tuple<V3d_i,V3d_i>  find_correspondences_withidx(SGLC::Graph graph1,SGLC::Graph graph2);
        std::tuple<std::vector<Eigen::Vector3d>,std::vector<Eigen::Vector3d>>  findCorrespondences(SGLC::Graph graph1,SGLC::Graph graph2);
        std::tuple<V3d_i,V3d_i> outlier_pruning(SGLC::Graph graph1,SGLC::Graph graph2,V3d_i match_node1, V3d_i match_node2);

        std::tuple<Eigen::Isometry3d,double>ransac_alignment(std::vector<Eigen::Vector3d> match_node1,
                                                            std::vector<Eigen::Vector3d> match_node2,
                                                            std::vector<Eigen::Vector3d> node1,
                                                            std::vector<Eigen::Vector3d> node2,
                                                            int max_inter,int& best_inlier_num);
        std::tuple<Eigen::Isometry3d,double> ransac_alignment(std::vector<Eigen::Vector3d> match_node1,
                                                            std::vector<Eigen::Vector3d> match_node2,
                                                                int max_inter, int& best_inlier_num);
        Eigen::Isometry3d solveSVD(std::vector<Eigen::Vector3d> match_node1,std::vector<Eigen::Vector3d> match_node2);

        // background
        Eigen::MatrixXf gen_background_descriptors(V3d_i filtered_pointcloud);
        std::vector<float> gen_back_descriptors_global(V3d_i filtered_pointcloud);
        double calculateSim(Eigen::MatrixXf &desc1, Eigen::MatrixXf &desc2);
        std::pair<std::vector<Eigen::Vector3d>, std::vector<int>> pcl2eigen(pcl::PointCloud<pcl::PointXYZL>::Ptr pointcloud);
        V3d_i VoxelDownsample(const V3d_i &frame, double voxel_size);
    };
    

    

    
    

}