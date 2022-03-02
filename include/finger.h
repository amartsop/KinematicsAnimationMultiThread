#pragma once

#include <iostream>
#include <vector>
#include <filesystem>
#include <fstream>

#include <igl/opengl/glfw/Viewer.h>
#include "dynamics_math.h"
#include "euler_rotations.h"

#include "./nlohmann/json.hpp"

class Finger
{
public:
    Finger() {};

    // Initialize finger
    void initialize(const std::string& name_id, const nlohmann::json& json_file, 
        igl::opengl::glfw::Viewer *viewer, int mesh_idx);

    // Update state
    void update(const std::vector<dm::JointState>& state);

    // Get vertices
    std::vector<Eigen::MatrixXd> get_vertices(void) { return m_vertices_data; }

    // Get frame ids
    std::vector<int> get_frame_ids(void) { return m_frame_ids; };

    // Load mesh files
    void load_mesh_files(igl::opengl::glfw::Viewer *viewer);

    // Get current state 
    std::vector<dm::JointState> get_state(void) { return m_state_vec; }

private: 

    // Finger name id 
    std::string m_name_id;

    // Finger origin
    dm::JointState m_origin;

    // Finger link lengths
    std::vector<double> m_link_lengths;

    // Finger frames ids
    std::vector<int> m_frame_ids;

    // Parse the json file
    void parse_json_file(const nlohmann::json& json_file);

private:

    // Mesh files for joints and bones
    std::string m_joint_rel_filename = "share/joint.obj";
    std::string m_bone_rel_filename = "share/bone.obj";

    // Meshes filenames
    std::vector<std::string> m_meshes_filenames;

    // Initialize mesh files
    void initialize_mesh_containers(void);

    // Joint scales
    double m_joint_scale = 0.05;

    // Finger scales
    std::vector<double> m_geom_scales;

    // Viewer data lower and upper idx
    int m_viewer_data_lower_idx, m_viewer_data_upper_idx;

private:

    // Get mesh data
    void get_mesh_data(igl::opengl::glfw::Viewer *viewer);

    // Vertices data (original)
    std::vector<Eigen::MatrixXd> m_vertices_data_o;

    // Vertices data (original homogeneous)
    std::vector<Eigen::MatrixXd> m_vertices_data_oh;

    // Vertices data
    std::vector<Eigen::MatrixXd> m_vertices_data;

    // Faces data
    std::vector<Eigen::MatrixXi> m_faces_data;

    // Postproccess meshes
    void postprocess_meshes(void);

private:

    // Initialize state
    void initialize_state(const std::vector<double>& link_lengths, 
        const dm::JointState& origin);

    // Finger state
    std::vector<dm::JointState> m_state_vec;

    // State size
    int m_state_size;

    // Local transforamation matrix 
    std::vector<Eigen::Matrix4d> m_local_transform;

    // Global transforamation matrix 
    std::vector<Eigen::Matrix4d> m_global_transform;
};