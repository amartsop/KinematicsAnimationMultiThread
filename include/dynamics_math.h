#pragma once 
#include <iostream>

#include <eigen3/Eigen/Core>

/// Namespace dm
/**
 * This namespace includes a custom structs and functions
 * that are used in various places inside the project.
*/

namespace dm
{
    /// The JointState struct defines both the position and the orientation
    /// of a body/frame. The Euler angles follow the post multiply sequence zyx.
    /// Rotate "psi" around Z (yaw), "theta" around y (pitch) and "phi"
    /// around x (roll).

    struct JointState{
        // Position vector 
        Eigen::Vector3d position = {0.0, 0.0, 0.0};

        // Orientation vector (euler angles) 
        Eigen::Vector3d euler = {0.0, 0.0, 0.0};
    };
} 

