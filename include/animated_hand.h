#pragma once 

#include <iostream>
#include <vector>
#include <map>
#include "euler_rotations.h"


class AnimatedHand
{

public:
    AnimatedHand() {};

    struct HandMap
    {
        // Frame's unique index
        int frame_idx;

        // Frames name/id
        int frame_id;

        //Rotation type sequence of rotations (roll=0, pitch=1, yaw=2)
        int rot_type;

        // Rotation direction (1: positive, -1:negative)
        double rot_dir;
    };
    
public:
    
    // Generate hand angles
    std::vector<Eigen::Vector3d>
        get_hand_angles(const std::vector<double>& joint_angles);


private:

    std::vector<HandMap> m_hand_map {
        HandMap{0, 3, 2, 1},
        HandMap{0, 3, 1, 1},
        HandMap{1, 4, 1, 1},
        HandMap{2, 5, 1, 1},
        HandMap{3, 6, 2, 1},
        HandMap{3, 6, 1, 1},
        HandMap{4, 7, 1, 1},
        HandMap{5, 8, 1, 1},
        HandMap{6, 0, 0, 1},
        HandMap{6, 0, 1, 1},
        HandMap{6, 0, 2, 1},
        HandMap{7, 1, 1, 1},
        HandMap{8, 2, 1, 1},
    };

    // Number of hand frames
    int m_hand_frames_num = 9;

    // Hand index iterator
    std::vector<int> m_hand_idx_iter = {3, 4, 5, 6, 7, 8, 0, 1, 2};
};
