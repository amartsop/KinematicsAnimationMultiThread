#include "../include/kinematic_animation.h"

/**
 * @brief Initializes kinematics animation by copying the input arguments
 * to the member variables.
 * @param viewer Pointer to the igl Viewer object.
 * @param left_exo Pointer to the left exoskeleton object.
 * @param anim_hand Pointer to the animated hand object.
 * @param menu_handler Pointer to menu handler object.
 */
void KinematicAnimation::initialize(igl::opengl::glfw::Viewer* viewer, 
    Exoskeleton* left_exo, AnimatedHand* anim_hand, MenuHandler* menu_handler)
{
    // Get exoskeleton handler pointer
    m_left_exo = left_exo;

    // Get animation hand pointer
    m_anim_hand = anim_hand;

    // Get menu handler pointer
    m_menu_handler = menu_handler;

    // Set camera center
    m_camera_center << -0.1, -0.1, 0.0, 0.1, -0.1, 0.0, 0.0, 0.1, 0.0;
}

/**
 * @brief 
 *  This is the main animation callback function. This is where the rendering is 
 * happening. The function is passed as a lambda function to
 * the Viewer handler (see main.cpp).
 * @param viewer Reference to the viewer handle.
 * @return true Animation should stop.
 * @return false Animations keeps playing.
 */
bool KinematicAnimation::animation_loop(igl::opengl::glfw::Viewer& viewer)
{
    if (viewer.core().is_animating)
    {
        viewer.core().align_camera_center(m_camera_center);

        // Initialize if ports are set
        if (m_menu_handler->are_ports_set() && m_initialize_animation)
        {
            // Setup exoskeleton
            setup_exoskeletons(viewer);

            // Dont initialize animations again
            m_initialize_animation = 0;
        }

        if(m_menu_handler->are_ports_set())
        {
            // Get euler angles
            auto euler_id =
                m_anim_hand->get_hand_angles(m_left_exo->get_joint_angles());
            
            // Update left hand
            m_left_hand.update(euler_id, viewer);
            
            // Update right hand
            m_right_hand.update(euler_id, viewer);
        }
    } 
    return false;
}

/**
 * @brief This function setups the exoskeletons. It initializes the 
 * serial communications, the exoskeleton and the hand objects.
 * @param viewer A reference to the viewer handle.
 */
void KinematicAnimation::setup_exoskeletons(igl::opengl::glfw::Viewer& viewer)
{
   // Define serial COM for left exoskeleton
    std::string serial_com_left = m_menu_handler->get_left_exoskeleton_port();

   // Define serial COM for right exoskeleton
    std::string serial_com_right = m_menu_handler->get_right_exoskeleton_port();

    // Define baudrate
    unsigned int baud_rate = 115200;
    
    // Initialize left exoskeleton
    m_left_exo->initialize(serial_com_left, baud_rate);

    // Initialize right exoskeleton (to be done)

    // Initialize left hand 
    m_left_hand.initialize(&viewer, m_left_exo, m_anim_hand, 0, m_left_origin);

    // Initialize right hand
    m_right_hand.initialize(&viewer, m_left_exo, m_anim_hand, 1, m_right_origin);
}