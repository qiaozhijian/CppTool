#pragma once
#include "testHead.h"


void Rotate(void);

void EulerToRoMat(Vector3d euler,Eigen::Matrix3d& rotation_matrix);

void EulerToQuat(Vector3d euler,Eigen::Quaterniond& quaternion);

