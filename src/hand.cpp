#include "../include/hand.h"

void Hand::initialize(igl::opengl::glfw::Viewer* viewer,
    Exoskeleton* exo_handler, AnimatedHand* anim_hand,
    bool type, const Eigen::Vector3d& origin)
{
    // Define absolute path of hand configuration file
    m_config_abs_path = std::filesystem::current_path() / m_config_rel_path;

    // Parse json file
    std::ifstream file(m_config_abs_path);
    nlohmann::json json_file = nlohmann::json::parse(file);

    // Define hand pose 
    m_hand_origin = origin;
    m_hand_rot = Eigen::Matrix3d::Identity();
    if (type) // Check if it's the right hand and apply mirror transformation
    {
        m_hand_rot(1, 1) = -1.0;
    }

    // Resize fingers vector
    m_fingers.resize(m_hand_config.size());
    
    // Intitialize vertex data 
    std::vector<std::vector<Eigen::MatrixXd>> vertex_data;

    // Get lower viewer data idx 
    m_viewer_data_lower_idx = (viewer->data_list.size() == 1) ? 0 :
        viewer->data_list.size();

    // Mesh idx initialization
    int mesh_idx = m_viewer_data_lower_idx;

    // Initialize fingers
    for (size_t i = 0; i < m_fingers.size(); i++)
    {
        m_fingers.at(i).initialize(m_hand_config.at(i), json_file, viewer,
            mesh_idx);

        // Update mesh idx
        mesh_idx = viewer->data_list.size();
    }

    // Get upper viewer data idx 
    m_viewer_data_upper_idx = viewer->data_list.size();

    // Data list size    
    m_data_list_size = m_viewer_data_upper_idx - m_viewer_data_lower_idx;
}


// Update hand
void Hand::update(const std::vector<Eigen::Vector3d>& euler_id, 
    igl::opengl::glfw::Viewer& viewer)
{
    // Intitialize vertex data 
    std::vector<std::vector<Eigen::MatrixXd>> vertex_data;

    // Update fingers
    for (size_t i = 0; i < m_fingers.size(); i++)
    {
        // Get finger frame ids
        std::vector<int> frame_ids = m_fingers.at(i).get_frame_ids();
        
        // Get current state of finger 
        std::vector<dm::JointState> state_vec = m_fingers.at(i).get_state();

        for (size_t j = 0; j < frame_ids.size(); j++)
        {
            state_vec.at(j).euler = euler_id.at(frame_ids.at(j));
        }

        // Update finger            
        m_fingers.at(i).update(state_vec);

        // Get vertices for finger i
        std::vector<Eigen::MatrixXd> finger_i_vertices =
            m_fingers.at(i).get_vertices();

        // Transform the data to match the hand position        
        for (size_t j = 0; j < finger_i_vertices.size(); j++)
        {
            // Get the translation matrix
            Eigen::MatrixXd t_mat = translation_matrix(m_hand_origin,
                finger_i_vertices.at(j).rows());
            
            // Perform transformation
            finger_i_vertices.at(j) = t_mat + (m_hand_rot *
                finger_i_vertices.at(j).transpose()).transpose();
        }

        // Push back vertex data
        vertex_data.push_back(finger_i_vertices);
    }

    // Set vertex data
    m_vertex_data = concatenate_data<Eigen::MatrixXd>(vertex_data);

    // Send vertex data to viewer
    for (size_t i = 0; i < m_data_list_size; i++)
    {
        // Send vertex data to viewer
        viewer.data_list.at(m_viewer_data_lower_idx +
            i).set_vertices(m_vertex_data.at(i));
    }
}

// Translation matrix
Eigen::MatrixXd Hand::translation_matrix(const Eigen::Vector3d& offset,
    size_t vert_num)
{
    // Initialize matrix 
    Eigen::MatrixXd t_mat = Eigen::MatrixXd(vert_num, offset.rows());

    // Ones vector
    Eigen::VectorXd ones_vec = Eigen::VectorXd::Ones(vert_num);

    for(size_t i = 0; i < offset.rows(); i++)
    {
        t_mat.col(i) = offset(i) * ones_vec;
    }

    return t_mat;
}