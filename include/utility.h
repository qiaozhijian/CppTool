#ifndef _UTILITY_LIDAR_ODOMETRY_Hooo
#define _UTILITY_LIDAR_ODOMETRY_Hooo

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
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

using namespace std;

typedef pcl::PointXYZI  PointType;


struct smoothness_t {
    float value;
    size_t ind;
};

struct by_value {
    bool operator()(smoothness_t const &left, smoothness_t const &right) {
        return left.value < right.value;
    }
};

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
                                  )

typedef PointXYZIRPYT  PointTypePose;

int32_t createDirectory(const std::string &directoryPath);
void writeKittiPclBinData(pcl::PointCloud<PointType>::Ptr input_pointcloud,string dir);
int getTimeStamp(string seq);
void replace_str(std::string& str, const std::string& before, const std::string& after);
void loadBin(string binfile, pcl::PointCloud<PointType>::Ptr laserCloudIn);
// int rmGround(string seq);
#endif
