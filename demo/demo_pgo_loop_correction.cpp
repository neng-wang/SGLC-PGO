#include <iostream>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>
#include <mutex>

#include "include/utilis.h"
#include "SemanticGraph.hpp"


bool runGtsamFlag = false;
bool runPublishFlag = false;
gtsam::Values initial;
gtsam::NonlinearFactorGraph gtSAMgraph;
std::mutex mtxPosegraph;
std::mutex mtxpath;
gtsam::ISAM2 *isam;
gtsam::Values resultsIsam;
gtsam::noiseModel::Diagonal::shared_ptr odometryNoise;
gtsam::noiseModel::Base::shared_ptr robustLoopNoise;

ros::Publisher pubOdomAftMapped;
ros::Publisher pubCurrentCloud;
ros::Publisher pubMatchedCloud;
ros::Publisher pubCurrentNode;
ros::Publisher pubMatchedNode;
ros::Publisher pubSGLC;
ros::Publisher pubPath;
std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_vec;
std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> poses_vec_raw;
std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> key_poses_vec;
std::vector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> cloud_vec;
nav_msgs::Path path;

int recentIdxUpdated = 0;

std::unordered_map<int,Eigen::Vector3i> color_map={
			{0,{0,0,0}},
			{0,{255,0,0}},
			{1,{100,150,245}},
			{2,{100,230,245}},
			{5,{100,80,250}},
			{3,{30,60,150}},
			{5,{0,0,255}},
			{4,{80,30,180}},
			{5,{0,0,255}},
			{6,{255,30,30}},
			{7,{255,40,200}},
			{8,{150,30,90}},
			{9,{255,0,255}},
			{10,{255,150,255}},
			{11,{75,0,75}},
			{12,{175,0,75}},
			{13,{255,200,0}},
			{14,{255,120,50}},
			{0,{255,150,0}},
			{9,{150,255,170}},
			{15,{0,175,0}},
			{16,{135,60,0}},
			{17,{150,240,80}},
			{18,{255,240,150}},
			{19,{255,0,0}},
			{0,{50,255,255}},
			{1,{100,150,245}},
			{7,{255,40,200}},
			{6,{255,30,30}},
			{8,{150,30,90}},
			{5,{0,0,255}},
			{5,{100,80,250}},
			{4,{80,30,180}},
			{5,{255,0,0}}
		};

void zfill(std::string& in_str,int len){
        while (in_str.size() < len)
        {
            in_str = "0" + in_str;
        }
}

void gtsamGraphpProcess(void){

    ros::Rate rate(1);
    while (ros::ok()){
        rate.sleep();
        if(runGtsamFlag){
            std::cout<<"[run isam]"<<std::endl;
            mtxPosegraph.lock();
            isam->update(gtSAMgraph,initial);
            isam->update();
            gtSAMgraph.resize(0);
            initial.clear();
            resultsIsam = isam->calculateEstimate();
            mtxPosegraph.unlock();
            recentIdxUpdated = int(resultsIsam.size()) - 1;
            std::cout<<"recentIdxUpdated:"<<recentIdxUpdated<<std::endl;
            std::cout<<"poses_vec size:"<<poses_vec.size()<<std::endl;
            std::cout<<"path.poses size:"<<path.poses.size()<<std::endl;

            Eigen::Vector3d raw_translation_receup = poses_vec_raw[recentIdxUpdated].first;
            Eigen::Matrix3d raw_rotation_receup = poses_vec_raw[recentIdxUpdated].second;
 
            for(size_t node_idx=0;node_idx<resultsIsam.size();node_idx++){
                 gtsam::Pose3 pose = resultsIsam.at(node_idx).cast<gtsam::Pose3>();
                 poses_vec[node_idx].first = pose.translation();
                 poses_vec[node_idx].second = pose.rotation().matrix();
                 Eigen::Quaterniond q(pose.rotation().matrix());
                 mtxpath.lock();
                 path.poses[node_idx].pose.position.x = poses_vec[node_idx].first[0];
                 path.poses[node_idx].pose.position.y = poses_vec[node_idx].first[1];
                 path.poses[node_idx].pose.position.z = poses_vec[node_idx].first[2];
                 path.poses[node_idx].pose.orientation.w = q.w();
                 path.poses[node_idx].pose.orientation.x = q.x();
                 path.poses[node_idx].pose.orientation.y = q.y();
                 path.poses[node_idx].pose.orientation.z = q.z();
                 mtxpath.unlock();
            }
            
            Eigen::Vector3d new_translation_receup = poses_vec[recentIdxUpdated].first;
            Eigen::Matrix3d new_rotation_receup = poses_vec[recentIdxUpdated].second;

            for(int node_idx=recentIdxUpdated+1;node_idx<recentIdxUpdated+50;node_idx++){
                if(node_idx>=poses_vec.size()) break;

                poses_vec[node_idx].first = new_rotation_receup * raw_rotation_receup.transpose() * (poses_vec_raw[node_idx].first-raw_translation_receup) + new_translation_receup;
                poses_vec[node_idx].second = new_rotation_receup * raw_rotation_receup.transpose() * poses_vec_raw[node_idx].second;
            }
        }
    }
}

void publish(void){
    ros::Rate rate(20);
    while(ros::ok()){
        if(runPublishFlag){
            pubPath.publish(path);
            rate.sleep();
        }
        
    }

}

int main(int argc, char **argv){
    ros::init(argc, argv, "pgo_demo");
    ros::NodeHandle nh;
    std::string lidar_path = "/home/wangneng/DataFast/kitti/sequences/00/velodyne/";
    std::string pose_path = "/home/wangneng/SLAM/loop_closure/graph_loop_pgo/catkin_ws/src/poses/kitti00.txt";
    std::string label_path = "/home/wangneng/SLAM/loop_closure/SegNet4D_predicitions/kitti/sequences/00/predictions/";
    std::string config_path = "/home/wangneng/SLAM/loop_closure/graph_loop_pgo/catkin_ws/src/config/config_kitti_graph.yaml";
    double map_voxel_size = 0.5;
    int file_name_length = 6;
    int sub_frame_num = 5;
    nh.param<std::string>("lidar_path", lidar_path, "");
    nh.param<std::string>("pose_path", pose_path, "");
    nh.param<std::string>("label_path", label_path, "");
    nh.param<std::string>("config_path", config_path, "");
    nh.param<int>("file_name_length", file_name_length, 6);
    nh.param<double>("map_voxel_size", map_voxel_size, 0.5);
    nh.param<int>("sub_frame_num", sub_frame_num, 10);

    pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
    pubCurrentCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
    pubMatchedCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
    pubCurrentNode =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_node", 100);
    pubMatchedNode =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_node_matched", 100);
    pubSGLC =
        nh.advertise<visualization_msgs::MarkerArray>("node_correspondences", 10);
    pubPath = 
        nh.advertise<nav_msgs::Path>("/Path",100);

    
    ros::Rate loop(20);
    ros::Rate slow_loop(0.1);

    

    std::vector<double> times_vec;

    // replace here in odomtry
    load_pose_with_time(pose_path, poses_vec, times_vec);

    poses_vec_raw.assign(poses_vec.begin(),poses_vec.end()); 
    std::cout << "Sucessfully load pose with number: " << poses_vec.size()
                << std::endl;
    

    //等待系统初始化
    std::cout<<"Waiting system init!"<<std::endl;
    slow_loop.sleep();
    std::cout<<"System init success!"<<std::endl;

    SGLC::SemanticGraph SemGraph(config_path);

    //gtsam
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    gtsam::Vector Vector6(6);
    Vector6 << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    
    double loopNoiseScore = 0.1;
    gtsam::Vector robustNoiseVector6(
        6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
        loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

    std::thread isam_update {gtsamGraphpProcess};


    size_t cloudInd = 0;
    size_t keyCloudInd = 0;
    std::vector<int> keyidx2idx;
    while (ros::ok()){
        bool loop_flag = false;
        std::stringstream lidar_data_path;
        std::stringstream lidar_label_path;
        lidar_data_path << lidar_path << std::setfill('0') << std::setw(6)
                    << cloudInd << ".bin";
        lidar_label_path << label_path << std::setfill('0') << std::setw(6)
                    << cloudInd << ".label";

        auto lidar_data = SemGraph.loadCloud(lidar_data_path.str(),lidar_label_path.str());
        if(lidar_data.first.size()==0) {
            break;
        }
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>());

        Eigen::Vector3d translation = poses_vec[cloudInd].first;
        Eigen::Matrix3d rotation = poses_vec[cloudInd].second;

        for (std::size_t i = 0; i < lidar_data.first.size(); i++) {
            if(lidar_data.second[i]==0) continue;
            
            pcl::PointXYZRGBL point(color_map[lidar_data.second[i]][0],color_map[lidar_data.second[i]][1],\
                                    color_map[lidar_data.second[i]][2],lidar_data.second[i]);
            Eigen::Vector3d pv = rotation * lidar_data.first[i] + translation;
            point.x = pv[0];
            point.y = pv[1];
            point.z = pv[2];
            current_cloud->push_back(point);
        }
        pcl::VoxelGrid<pcl::PointXYZRGBL> sor;
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr current_cloud_downsmample(new pcl::PointCloud<pcl::PointXYZRGBL>());
        sor.setInputCloud(current_cloud);
        sor.setLeafSize(map_voxel_size, map_voxel_size, map_voxel_size);
        sor.filter(*current_cloud_downsmample);
        cloud_vec.push_back(current_cloud_downsmample);

        if(cloudInd % sub_frame_num == 0 && cloudInd != 0){

            keyidx2idx.emplace_back(cloudInd);
            auto global_des = SemGraph.gen_scan_descriptors(lidar_data);
            if(SemGraph.key_frame_idx>(SemGraph.num_exclude_curr+2*SemGraph.search_results_num)){
                
                loop_flag = SemGraph.search_loop();
                if(loop_flag){
                    int src_frame_idx = keyidx2idx[SemGraph.loop_pair_vec.back().first];
                    int tar_frame_idx = keyidx2idx[SemGraph.loop_pair_vec.back().second];
                    Eigen::Matrix4d trans = SemGraph.loop_trans_vec.back();
                    // trans = trans.inverse();
                    std::cout << "[Loop Detection]  between:" << src_frame_idx<< " ------ " <<tar_frame_idx << \
                                ",  Time [gen des]:"<<SemGraph.time_gen_des<<"ms, [loop search]:"<<SemGraph.time_search<<"ms"<<std::endl;
                    std::cout << "[Loop Pose]" << SemGraph.loop_trans_vec.back() << std::endl;

                    gtsam::Point3 ttem(trans.block<3,1>(1,3));
                    gtsam::Rot3 Rtem(trans.block<3,3>(0,0));

                    mtxPosegraph.lock();  
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(tar_frame_idx, src_frame_idx,gtsam::Pose3(Rtem, ttem), robustLoopNoise));
                    mtxPosegraph.unlock();

                    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr matched_cloud(new pcl::PointCloud<pcl::PointXYZRGBL>());
                    sensor_msgs::PointCloud2 pub_cloud_matched;
                    for(int i=0;i<cloud_vec[tar_frame_idx]->size();i++){
                        pcl::PointXYZRGBL point = (*cloud_vec[tar_frame_idx])[i];
                        point.z = point.z + 20;
                        matched_cloud->push_back(point);
                    }
                    pcl::toROSMsg(*matched_cloud, pub_cloud_matched);
                    pub_cloud_matched.header.frame_id = "velodyne";
                    pub_cloud_matched.header.stamp = ros::Time::now();
                    pubMatchedCloud.publish(pub_cloud_matched);

                }
                else{
                    std::cout << "[No Loop ]  between:" << keyidx2idx[SemGraph.key_frame_idx-1]<< \
                                ",  Time [gen des]:"<<SemGraph.time_gen_des<<"ms, [loop search]:"<<SemGraph.time_search<<"ms"<<std::endl;
                }
            }
            else{
                std::cout << "[No Loop ]  between:" << keyidx2idx[SemGraph.key_frame_idx-1]<< \
                                ",  Time [gen des]:"<<SemGraph.time_gen_des<<"ms"<<std::endl;
            }

             
                Eigen::Vector3d t_ab = poses_vec[cloudInd - 1].first;
                Eigen::Matrix3d R_ab = poses_vec[cloudInd - 1].second;

                t_ab = R_ab.transpose() * (poses_vec[cloudInd].first - t_ab);
                R_ab = R_ab.transpose() * poses_vec[cloudInd].second;

                gtsam::Rot3 R_sam(R_ab);
                gtsam::Point3 t_sam(t_ab);

                mtxPosegraph.lock();  
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudInd - 1, cloudInd, gtsam::Pose3(R_sam, t_sam), odometryNoise));
                initial.insert(cloudInd,gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),gtsam::Point3(poses_vec[cloudInd].first)));
                mtxPosegraph.unlock();
        }
        else{
            if(cloudInd==0){
                mtxPosegraph.lock();  
    
                gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0,gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second), gtsam::Point3(poses_vec[cloudInd].first)),odometryNoise));
                initial.insert(0,gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),gtsam::Point3(poses_vec[cloudInd].first)));
                mtxPosegraph.unlock();
                if(!runGtsamFlag) runGtsamFlag = true;
            }
            else{
                Eigen::Vector3d t_ab = poses_vec[cloudInd - 1].first;
                Eigen::Matrix3d R_ab = poses_vec[cloudInd - 1].second;

                t_ab = R_ab.transpose() * (poses_vec[cloudInd].first - t_ab);
                R_ab = R_ab.transpose() * poses_vec[cloudInd].second;

                gtsam::Rot3 R_sam(R_ab);
                gtsam::Point3 t_sam(t_ab);

                mtxPosegraph.lock();    
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudInd - 1, cloudInd, gtsam::Pose3(R_sam, t_sam), odometryNoise));
                initial.insert(cloudInd, gtsam::Pose3(gtsam::Rot3(poses_vec[cloudInd].second),gtsam::Point3(poses_vec[cloudInd].first)));
                mtxPosegraph.unlock();
            }
        }

        sensor_msgs::PointCloud2 pub_cloud;
        pcl::toROSMsg(*current_cloud_downsmample, pub_cloud);
        pub_cloud.header.frame_id = "velodyne";
        pub_cloud.header.stamp = ros::Time::now();
        pubCurrentCloud.publish(pub_cloud);


        // odom
        nav_msgs::Odometry odom;
        odom.header.frame_id = "velodyne";
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = translation[0];
        odom.pose.pose.position.y = translation[1];
        odom.pose.pose.position.z = translation[2];
        Eigen::Quaterniond q(rotation);
        odom.pose.pose.orientation.w = q.w();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        pubOdomAftMapped.publish(odom);

        // path
        geometry_msgs::PoseStamped poseStamp;
        poseStamp.header = odom.header;
        poseStamp.pose = odom.pose.pose;
        path.header.stamp = poseStamp.header.stamp;
        path.header.frame_id = "velodyne";
        mtxpath.lock();
        path.poses.push_back(poseStamp);
        mtxpath.unlock();
        pubPath.publish(path);

        loop.sleep();
        cloudInd++;
        if(!runPublishFlag) runPublishFlag=true;
    }
    return 0;
}