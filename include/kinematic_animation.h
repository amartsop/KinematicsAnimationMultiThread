#pragma once 

#include <iostream>
#include <igl/opengl/glfw/Viewer.h>

#include "animated_hand.h"
#include "exoskeleton.h"
#include "menu_handler.h"
#include "hand.h"

/// Class KinematicAnimation
/**
 * This class implements the kinematic animation for the hand.
*/
class KinematicAnimation
{
public:
    /// Constructor.
    KinematicAnimation(){};

    /// Initialize animation.
    void initialize(igl::opengl::glfw::Viewer* viewer, 
        Exoskeleton* left_exo, AnimatedHand* anim_hand, 
        MenuHandler* menu_handler);

    /// Animation loop callback.
    bool animation_loop(igl::opengl::glfw::Viewer & viewer);


private:

    /// Exoskeleton handler pointer.
    Exoskeleton* m_left_exo;

    /// Animated hand pointer.
    AnimatedHand* m_anim_hand;

    /// Menu handler pointer.
    MenuHandler* m_menu_handler;

    /// Left and right hand.
    Hand m_left_hand, m_right_hand;

    /// Left hand origin.
    Eigen::Vector3d m_left_origin = Eigen::Vector3d(0.0, 0.2, 0.0);

    /// Right hand origin.
    Eigen::Vector3d m_right_origin = Eigen::Vector3d(0.0, -0.2, 0.0);

    /// Bool start animation.
    bool m_initialize_animation = 1;

    /// Setup exoskeletons.
    void setup_exoskeletons(igl::opengl::glfw::Viewer& viewer);

    /// Camera matrix.
    Eigen::Matrix3d m_camera_center;
};