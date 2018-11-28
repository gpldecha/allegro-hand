//
// Created by guillaume on 26/11/18.
//

#ifndef AHAND_TYPES_H
#define AHAND_TYPES_H

#include <Eigen/Dense>

typedef Eigen::Matrix<double, 16, 1> Vector16d;

struct hand_info{
    Vector16d measured_joint_positions;
    Vector16d estimated_joint_velocities;
};

enum CONTROL_TYPE { NONE, JOINT_IMPEDANCE, BHAND };


#endif //AHAND_TYPES_H
