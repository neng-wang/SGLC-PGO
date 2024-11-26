#pragma once

#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <fstream>
#include <unordered_map>



void load_pose_with_time(
    const std::string &pose_file,
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> &poses_vec,
    std::vector<double> &times_vec);