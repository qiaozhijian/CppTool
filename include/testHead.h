#pragma once
// #include <mrpt/opengl.h>
// #include <mrpt/opengl/CPointCloudColoured.h>
// #include <mrpt/gui.h>
// #include <mrpt/utils/CConfigFile.h>
// #include <mrpt/utils/CConfigFileBase.h>
// using namespace mrpt;
// using namespace mrpt::gui;
// using namespace mrpt::poses;
// using namespace mrpt::utils;
// using namespace mrpt::math;
// using namespace mrpt::opengl;

#include <iostream>
#include <fstream>
#include<iomanip>
#include <cv.h>
#include <opencv2/features2d/features2d.hpp>
// using namespace cv;
// #include "auxiliar.h"
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"
#include <vector>
#include <list>
#include <map>
using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <cmath>
using namespace Eigen;


using Eigen::Dynamic; 
using Eigen::Matrix; 
using Eigen::RowMajor; 

typedef Matrix<double, Dynamic, Dynamic, RowMajor> MatXf; 
typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;


typedef std::pair<int, int> point_2d;

#include <thread>
#include <mutex>
#include "glog/logging.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>


