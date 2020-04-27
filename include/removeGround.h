#ifndef _UTILITY_LIDAR_ODOMETGJGTTRY_H_
#define _UTILITY_LIDAR_ODOMETGJGTTRY_H_



#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <pcl/segmentation/progressive_morphological_filter.h>  //LYT
#include <pcl/sample_consensus/method_types.h>          //LYT
#include <pcl/sample_consensus/model_types.h>           //LYT
#include <pcl/segmentation/sac_segmentation.h>          //LYT
#include <pcl/filters/extract_indices.h>                //LYT
#include <pcl/filters/radius_outlier_removal.h>         //LYT
#include <pcl/filters/random_sample.h>                  //LYT
#include <pcl/filters/statistical_outlier_removal.h>    //LYT
#include <pcl/filters/passthrough.h>                    //LYT


#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#include <glog/logging.h>

#define ORIGIN_DETECT_CL
#define KITTI
// #define OUSTER
// #define VLP16
// #define PUBLISH_TO_PY

#define PI 3.14159265
#define OUT_POINTS 4096

int rmGround(string seq);
using namespace std;

typedef pcl::PointXYZI  PointType;

extern const string imageTopic = "/spinnaker/main0/image_raw";//"/spinnaker/frontview/image_raw";


#ifdef OUSTER
extern const string pointCloudTopic = "/os1_node/points";
#else
extern const string pointCloudTopic = "/velodyne_points";
#endif

extern const string imuTopic = "/imu";
// extern const string imuTopic = "/os1_node/imu";
// extern const string imuTopic = "/mti/sensor/imu";
// extern const string imuTopic = "/imu/data";

// Save pcd
extern const string fileDirectory = "/tmp/";
#ifdef VLP16
// VLP-16
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0+0.1;
extern const int groundScanInd = 7;
#endif
#ifdef KITTI
#define CORRECT_FREE
// // HDL-64
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 0.427;
extern const float ang_bottom = 24.9;
extern const int groundScanInd = 50;
#endif
// // HDL-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 0.427;
// extern const float ang_bottom = 24.9;
// extern const int groundScanInd = 50;

// HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet, please just publish point cloud data

#ifdef OUSTER
// Ouster OS1-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 7;

// Ouster OS1-64
extern const int N_SCAN = 64;
extern const int Horizon_SCAN = 1024;
extern const float ang_res_x = 360.0/float(Horizon_SCAN);
extern const float ang_res_y = 33.2/float(N_SCAN-1);
extern const float ang_bottom = 16.6+0.1;
extern const int groundScanInd = 15;
#endif

extern const bool loopClosureEnableFlag = true;
extern const double mappingProcessInterval = 0.3;

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 200;

extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy

extern const int segmentValidNum = 30;
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;

typedef PointXYZIRPYT  PointTypePose;

#endif
