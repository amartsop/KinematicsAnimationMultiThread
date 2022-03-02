#pragma once 

#include <iostream>
#include <vector>
#include <filesystem>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include "./nlohmann/json.hpp"
#include <igl/opengl/glfw/Viewer.h>

#include "dynamics_math.h"
#include "exoskeleton.h"
#include "animated_hand.h"
#include "finger.h"

class Hand
{
public:
    Hand(){};

    // Initialize
    void initialize(igl::opengl::glfw::Viewer* viewer,
        Exoskeleton* exo_handler, AnimatedHand* anim_hand,
        bool type, const Eigen::Vector3d& origin);

    // Update hand
    void update(const std::vector<Eigen::Vector3d>& euler_id, 
        igl::opengl::glfw::Viewer& viewer);

private:
    // Config file
    std::string m_config_rel_path = "share/hand_config.json";

    // Parse hand configuration file
    std::filesystem::path m_config_abs_path;

    // Parse hand configuration file
    void parse_config_file(const std::string& path);

    // Hand configuration
    std::vector<std::string> m_hand_config = {"Thumb", "Index", "Middle"};

    // Fingers
    std::vector<Finger> m_fingers;

    // Hand rotation matrix
    Eigen::Matrix3d m_hand_rot;

    // Hand origin
    Eigen::Vector3d m_hand_origin;

    // Hand index iterator
    std::vector<int> m_hand_idx_iter;

private:

    // Concatenate vertex or face data to a std vector 
    template <typename T>
    std::vector<T> concatenate_data(const std::vector<std::vector<T>>& data);

    // Vertex data
    std::vector<Eigen::MatrixXd> m_vertex_data;

    // Concatenated hand vertex data     
    Eigen::MatrixXd m_concatenated_hand_vertex_data;
    
    // Viewer data lower and upper idx
    int m_viewer_data_lower_idx, m_viewer_data_upper_idx;

    // Data list size 
    int m_data_list_size;    

    // Translation matrix
    Eigen::MatrixXd translation_matrix(const Eigen::Vector3d& offset, size_t vert_num);
};

// Concatenate vertex or face data to a std vector 
template <typename T>
std::vector<T> Hand::concatenate_data(const std::vector<std::vector<T>>& data)
{
    // Vertex data concatenated
    std::vector<T> data_conc;

    // Concatenate data to a single vector
    for (auto data_j : data)
    {
        for (size_t i = 0; i < data_j.size(); i++)
        {
            data_conc.push_back(data_j.at(i));
        }
    }

    return data_conc;
}