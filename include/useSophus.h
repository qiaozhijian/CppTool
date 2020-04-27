#pragma once
#include "testHead.h"

void UseSophus( void );
void se3andSE3(void);

Sophus::SE3d getSE3 (Vector3d euler,Eigen::Vector3d t);
