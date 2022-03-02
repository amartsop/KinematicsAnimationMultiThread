#pragma once 
#include <iostream>

#include <eigen3/Eigen/Core>

namespace dm
{
    struct JointState{

        // Position vector 
        Eigen::Vector3d position = {0.0, 0.0, 0.0};

        // Orientation vector (euler angles) 
        Eigen::Vector3d euler = {0.0, 0.0, 0.0};
    };
} 

