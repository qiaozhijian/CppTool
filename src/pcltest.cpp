#include "pcltest.h"
#include "testHead.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZI  PointType;
void txt2pcd(void)
{
    string path="/media/qzj/Software/code/catkin_ws/src/LPD-SLAM/experiment/kitti/00/trajectory.txt";
    std::ifstream infile(path);
    float x, y, z;
    int r, g, b;
    //cout<<"12"<<endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ pt;
    while(infile >> x >> y >> z )
    {
        pt.x = x;
        pt.y = y;
        pt.z = z;

        cloud->points.push_back(pt);
    }
    cloud->width = 1;
    cloud->height = cloud->points.size();
    
    
    std::string output_filename(path);
    pcl::io::savePCDFileBinary(output_filename.substr(0, output_filename.length()-4)+"try.pcd", *cloud);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PassThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr input_pointcloud,const std::string &field_name,float down_threshold, float up_threshold)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr PassT_cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
// 	PassT_cloud_filtered->points.resize(100);
// 	PointType nanPoint;
// 	nanPoint.x = std::numeric_limits<float>::quiet_NaN();
// 	nanPoint.y = std::numeric_limits<float>::quiet_NaN();
// 	nanPoint.z = std::numeric_limits<float>::quiet_NaN();
// 	nanPoint.intensity = -1;
// 	std::fill(PassT_cloud_filtered->points.begin(), PassT_cloud_filtered->points.end(), nanPoint);

    cout<<PassT_cloud_filtered<<endl;
    
    cout<<input_pointcloud<<endl;
    
	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud (input_pointcloud);                //设置输入点云
	pass.setFilterFieldName (field_name);             //设置过滤时所需要点云类型的XYZ字段
	pass.setFilterLimits (down_threshold, up_threshold);           //设置在过滤字段的范围

    cout<<PassT_cloud_filtered<<endl;
	pass.filter (*PassT_cloud_filtered);       //执行滤波，保存过滤结果在cloud_filtered
    cout<<PassT_cloud_filtered->points.size()<<endl;

	return PassT_cloud_filtered;
}


void shareTest(void)
{
    shared_ptr<int> sp(new int(10));                //一个指向整数的shared_ptr
    assert(sp.unique());                            //现在shared_ptr是指针的唯一持有者
    shared_ptr<int> sp2 = sp;                       //第二个shared_ptr,拷贝构造函数
    assert(sp == sp2 && sp.use_count() == 2);       //两个shared_ptr相等,指向同一个对象,引用计数为2
    *sp2 = 100;                                     //使用解引用操作符修改被指对象
    assert(*sp == 100);                             //另一个shared_ptr也同时被修改
    sp.reset();                                     //停止shared_ptr的使用
    assert(!sp);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr PassT_cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    cout<<PassT_cloud_filtered<<endl;
	PassT_cloud_filtered->points.resize(100);
	PointType nanPoint;
	nanPoint.x = 10.0;
	nanPoint.y = 1.0;
	nanPoint.z = 0.0;
	nanPoint.intensity = -1;
	std::fill(PassT_cloud_filtered->points.begin(), PassT_cloud_filtered->points.end(), nanPoint);
    
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_pointcloud (new pcl::PointCloud<pcl::PointXYZI>);
    cout<<input_pointcloud<<endl;
	input_pointcloud->points.resize(200);
	std::fill(input_pointcloud->points.begin(), input_pointcloud->points.end(), nanPoint);
    
    
    
	PassT_cloud_filtered = PassThroughFilter(input_pointcloud, "y", -100, 100); 
    cout<<PassT_cloud_filtered->points.size()<<endl;
    cout<<PassT_cloud_filtered->at(0).x<<endl;
    
        ////随机采样至固定数量
    pcl::PointCloud<PointType>::Ptr RandomS_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::RandomSample<PointType> rs;
    rs.setInputCloud(PassT_cloud_filtered);
    rs.setSample(100);                        //设置输出点的数量
    cout<<RandomS_cloud_filtered<<endl;
    rs.filter(*RandomS_cloud_filtered);        //下采样并输出到cloud_out	
    cout<<RandomS_cloud_filtered<<endl;
    
}
