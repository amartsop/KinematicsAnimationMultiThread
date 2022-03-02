#include "../include/animated_hand.h"

// Generate hand angles 
std::vector<Eigen::Vector3d> AnimatedHand::get_hand_angles(const
    std::vector<double>& joint_angles)
{
    // Initialize euler container
    std::vector<Eigen::Vector3d> euler_vec;

    // Zero all euler angles
    for (size_t i = 0; i < m_hand_frames_num; i++)
    {
        euler_vec.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
    }
    
    // Set euler angles
    for(size_t i = 0; i < m_hand_map.size(); i++)
    {
        // Get hand map i
        HandMap config_i = m_hand_map.at(i);
        
        // Define euler vector
        euler_vec.at(config_i.frame_id)(config_i.rot_type) = 
            config_i.rot_dir * joint_angles.at(i);
    }

    return euler_vec;
}