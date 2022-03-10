#include "../include/finger.h"

/**
 * @brief This initialization function first parses the finger configuration 
 * file (see Hand::m_config_rel_path) and sets up the properties of the finger.
 * It also loads and processes the meshes for the links and the joints and 
 * initializes the finger state.
 * @param name_id The name id of the finger.
 * @param json_file The json finger configuration file.
 * @param viewer Pointer to the viewer handle.
 * @param mesh_idx The mesh index.
 */
void Finger::initialize(const std::string& name_id,
    const nlohmann::json& json_file, igl::opengl::glfw::Viewer *viewer,
    int mesh_idx)
{
    // Get finger id
    m_name_id = name_id;

    // Parse the json file
    parse_json_file(json_file);

    // Initialize mesh files
    initialize_mesh_containers();

    // Load mesh files
    load_mesh_files(viewer);

    // Set viewer data indices
    m_viewer_data_lower_idx = mesh_idx;
    m_viewer_data_upper_idx = m_viewer_data_lower_idx +
        (m_meshes_filenames.size() - 1);
    
    // Get mesh data
    get_mesh_data(viewer);
    
    // Postprocess meshes
    postprocess_meshes();
    
    // Initialize state
    initialize_state(m_link_lengths, m_origin);

    // Initialize meshes
    update(m_state_vec);
}

/**
 * @brief This function updates the finger's state (position and orientation 
 * of its links and joints). It performs the forward kinematics for each link 
 * of the finger. It first calculates the local transform of each links with 
 * respect to each previous link. Then it calculates the global transforms 
 * (with respect to the hand's base frame \f$ f_{{W}_{0}}\f$) by performing 
 * and iterative compound transormation (post-miltiply rules, see 
 * Forward Kinematics Spong * Robot Modeling and Control). The frame conventions 
 * and the definitions of the rotation and translation matrices used are 
 * given in the handover document.
 * @param state The vector of joint euler angles and postions as
 * defined in dm::JointStateu.
 */
void Finger::update(const std::vector<dm::JointState>& state)
{
    // Update state vector
    m_state_vec =  state;

    // Resize states
    m_local_transform.resize(m_state_size);
    m_global_transform.resize(m_state_size);

    /* Loop through state vector components and
    generate transformation matrices */
    for (size_t i = 0; i < m_state_vec.size(); i++)
    {
        /*********** Local transformation ***********/
        // Initialize transformation matrices
        Eigen::Matrix4d local_transform = Eigen::Matrix4d::Identity();

        // Position vector
        local_transform.block<3, 1>(0, 3) = m_state_vec.at(i).position;

        // Rotation matrix
        local_transform.block<3, 3>(0, 0) =
            EulerRotations::rotation(m_state_vec.at(i).euler);

        // Push back locak transform
        m_local_transform.at(i) = local_transform;

        /*********** Global transformation ***********/
        if (i == 0) {
            // Global transform
            m_global_transform.at(i) = m_local_transform.at(i);
        }
        else {
            // Global transform
            m_global_transform.at(i) = m_global_transform.at(i-1) * 
                    m_local_transform.at(i);
        }
    }

    // Resize vertices data
    m_vertices_data.resize(2 * m_state_size);

    // Loop through global transformation matrices
    for (size_t i = 0; i < m_global_transform.size(); i++)
    {
        // Get global transformation matrix
        Eigen::Matrix4d t_mat = m_global_transform.at(i);

        /****************** Transform vertices ********************/
        // Joint vertices
        Eigen::MatrixXd joint_vert =
            (t_mat * m_vertices_data_oh.at(2*i).transpose()).transpose();

        // Link vertices
        Eigen::MatrixXd link_vert =
            (t_mat * m_vertices_data_oh.at(2*i+1).transpose()).transpose();

        // Push back vertices
        m_vertices_data.at(2*i) = joint_vert.leftCols<3>();
        m_vertices_data.at(2*i+1) = link_vert.leftCols<3>();
    }
}

/**
 * @brief It initializes the state of the finger based on the link_lengths and 
 * their origins as defined from the configuration file
 * (Hand::m_config_rel_path)
 * @param link_lengths The lengths of the links.
 * @param origin The origins of the links.
 */
void Finger::initialize_state(const std::vector<double>& link_lengths, 
    const dm::JointState& origin)
{
    // State size 
    m_state_size = link_lengths.size();

    // Initialize state vector
    m_state_vec.resize(m_state_size);

    // Set origin
    m_state_vec.at(0) = origin;

    // Generate initial state vector states  
    int idx = 0;

    for (size_t i = 1; i < m_state_size; i++)
    {
        // Set link lengths
        m_state_vec.at(i).position(0) = link_lengths.at(idx);

        // Update idx
        idx++;
    }
}

/**
 * @brief It initializes the mesh containers for its link and joint 
 * based on their properties defined in the configuration file
 * Hand::m_config_rel_path.
 */
void Finger::initialize_mesh_containers(void)
{
    // Get mesh files absolute filenames
    auto joint_mesh_abs = std::filesystem::current_path() / m_joint_rel_filename;
    auto bone_mesh_abs = std::filesystem::current_path() / m_bone_rel_filename;

   // Generate meshses filenames
    for (size_t i = 0; i < m_link_lengths.size(); i++)
    {
       // Meshes filenames configuration
       m_meshes_filenames.push_back(joint_mesh_abs.string());
       m_meshes_filenames.push_back(bone_mesh_abs.string());

       // Geometry scales configurations
       m_geom_scales.push_back(m_joint_scale);
       m_geom_scales.push_back(m_link_lengths.at(i));
    }
}

/**
 * @brief Simply loads the meshes to the viewer.
 * 
 * @param viewer Pointer to the viewer object.
 */
void Finger::load_mesh_files(igl::opengl::glfw::Viewer *viewer)
{
    for (size_t i = 0; i < m_meshes_filenames.size(); i++)
    {
        viewer->load_mesh_from_file(m_meshes_filenames.at(i));
    }
}

/**
 * @brief It passes a copy of the vertex data from the viewer to the local 
 * member variables of the finger instance.
 * @param viewer Pointer to the viewer object.
 */
void Finger::get_mesh_data(igl::opengl::glfw::Viewer *viewer)
{
    for (size_t i = m_viewer_data_lower_idx; i <= m_viewer_data_upper_idx; i++)
    {
        // Push back vertices data
        m_vertices_data_o.push_back(viewer->data_list.at(i).V);

        // Push back faces data
        m_faces_data.push_back(viewer->data_list.at(i).F);
    }
}

/**
 * @brief It sets the scale of the meshes based on the links length and the
 *  joints desired size.
 */
void Finger::postprocess_meshes(void)
{
    // Scale meshes
    for (size_t i = 0; i < m_vertices_data_o.size(); i++)
    {
        /************** Scale data *******************/
        Eigen::Matrix3d scale_matrix;
        scale_matrix << m_geom_scales.at(i), 0.0, 0.0,
            0.0, m_geom_scales.at(i), 0.0, 0.0, 0.0, m_geom_scales.at(i);

        m_vertices_data_o.at(i) = (scale_matrix *
            m_vertices_data_o.at(i).transpose()).transpose();

        /************** Generate homogeneous vertices data *******************/
        // Get mesh i
        Eigen::MatrixXd mesh_i = m_vertices_data_o.at(i);

        // Initialize homogeneous coordinates
        Eigen::MatrixXd mesh_i_oh = Eigen::MatrixXd::Ones(mesh_i.rows(),
            mesh_i.cols() + 1);

        // Generate matrix of homogeneous coordinates
        mesh_i_oh.col(0) = mesh_i.col(0);
        mesh_i_oh.col(1) = mesh_i.col(1);
        mesh_i_oh.col(2) = mesh_i.col(2);

        m_vertices_data_oh.push_back(mesh_i_oh);
    }
}

/**
 * @brief Simply parses the finger configuration file.
 * 
 * @param json_file The json file namek
 */
void Finger::parse_json_file(const nlohmann::json& json_file)
{
    // Get lengths
    auto lengths_json = json_file[m_name_id]["Lengths"];

    // Get frame ids
    auto frames_json = json_file[m_name_id]["Frames"];

    // Get origin position
    auto origin_position_json = json_file[m_name_id]["Origin"]["Position"];
    
    // Get origin orientation
    auto origin_euler_json = json_file[m_name_id]["Origin"]["Euler"];
    
    for(size_t i = 0; i < lengths_json.size(); i++)
    {
        // Push back link lengths
        m_link_lengths.push_back(lengths_json.at(i));

        // Set origin position
        m_origin.position(0) = origin_position_json.at(0);
        m_origin.position(1) = origin_position_json.at(1);
        m_origin.position(2) = origin_position_json.at(2);

        // Set origin orientation
        m_origin.euler(0) = origin_euler_json.at(0);
        m_origin.euler(1) = origin_euler_json.at(1);
        m_origin.euler(2) = origin_euler_json.at(2);
    }

    for(size_t i = 0; i < frames_json.size(); i++)
    {
        m_frame_ids.push_back(frames_json.at(i));
    }
}