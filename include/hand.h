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

/// Class Hand
/**
 * This class defines the hand stucture. It calls the finger class (see Finger::)
 * to compose the three fingers and develop the full kinematic model of
 * the hand.
*/
class Hand
{
public:
    /// Empty constructor.
    Hand(){};

    /// Initialize the hand.
    void initialize(igl::opengl::glfw::Viewer* viewer,
        Exoskeleton* exo_handler, AnimatedHand* anim_hand,
        bool type, const Eigen::Vector3d& origin);

    // Update the hand.
    void update(const std::vector<Eigen::Vector3d>& euler_id, 
        igl::opengl::glfw::Viewer& viewer);

private:
    /// Relative name of hand's configuration file. This is a json file
    /// that contains the 
    /// geometric characteristics of each finger (link lengths of the finger,
    /// ordered as [proximal, middle, distal]), the position and orientation 
    /// of the finger's frame with respect to the local hand frame
    /// \f$ T_{f_{O_{i}}}^{f_{W_{0}}} \f$ with \f$ i = 0, 3, 6 \f$ and 
    /// the finger's frame names as illustrated in the figure below.
    /// \image html animated_hand_kinematic_model.png width=600px
    std::string m_config_rel_path = "share/hand_config.json";

    /// Absolute name of hand's configuration file.
    std::filesystem::path m_config_abs_path;

    /// Hand configuration.
    std::vector<std::string> m_hand_config = {"Thumb", "Index", "Middle"};

    /// Vector of finger handles.
    std::vector<Finger> m_fingers;

    /// The total rotation matrix of the hand \f$ T_{f_{W_{0}}}^{F} \f$, with
    /// respect to the  inertial frame of reference \f$ F \f$.
    Eigen::Matrix3d m_hand_rot;

    /// Hand origin with respect to the inertial frame of reference \f$ F \f$.
    Eigen::Vector3d m_hand_origin;

private:

    /// Concatenates vertex or face data to a std vector.
    template <typename T>
    std::vector<T> concatenate_data(const std::vector<std::vector<T>>& data);

    /// Vertex data container.
    std::vector<Eigen::MatrixXd> m_vertex_data;

    /// Concatenated hand vertex data.
    Eigen::MatrixXd m_concatenated_hand_vertex_data;
    
    /// Viewer data lower and upper idx. The viewer object of libigl stores 
    /// the vertex data into a container. This includes ALL the bodies that 
    /// are rendered on the screen. For setting the vertices when they 
    /// are updated it is important to know where the vertices for each object 
    /// are located inside this container. These two variables define the 
    /// lower and upper index for these specific instance of the hand. 
    /// This way when we set the data for this hand to the
    /// viewer.data_list we know what vertices to update.
    /// For more info on how libigl handles multiple meshes at
    /// https://github.com/libigl/libigl/blob/main/tutorial/107_MultipleMeshes/main.cpp.
    int m_viewer_data_lower_idx, m_viewer_data_upper_idx;

    /// The total size of the vertices of this instance of the hand on the 
    /// data_list container.
    int m_data_list_size;    

    /// This function offset a set of vertices by a given offset.
    Eigen::MatrixXd translation_matrix(const Eigen::Vector3d& offset, size_t vert_num);
};

/**
 * @brief This function is used to concatenate Eigen matrices. More specifically 
 * each link of the finger is imported as an Eigen::Matrix. The total finger 
 * is then a vector of Eigen matrices. The whole hand is a vector of 
 * fingers or a vector of a vector of links. This function takes this vector 
 * of vectors of links and converts to a vector of fingers.
 * @tparam T The type of Eigen::Matrix to concatenate (MatrixXd, MatrixXi etc). 
 * @param data The vector of vectors of matrices.
 * @return std::vector<T> The vector of matrices.
 */
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