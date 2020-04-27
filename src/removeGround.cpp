// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "utility.h"
#include "removeGround.h"
#include <algorithm>
#include <pcl/io/pcd_io.h>

class Preprocess {
private:

    pcl::PointCloud<PointType>::Ptr laserCloudIn;

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr rmGroundCloud;
    pcl::PointCloud<PointType>::Ptr outlierCloud;
    pcl::PointCloud<PointType>::Ptr StatisticalOR_cloud_filtered;
    //pcl::PointCloud<PointType>::Ptr VoxelG_cloud_filtered;
    pcl::PointCloud<PointType>::Ptr PassT_cloud_filtered1;
    pcl::PointCloud<PointType>::Ptr PassT_cloud_filtered2;
    pcl::PointCloud<PointType>::Ptr RandomS_cloud_filtered;

    PointType nanPoint; // fill in fullCloud at each iteration

    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    float startOrientation;
    float endOrientation;

    int pcl_num;

    std::vector<std::pair<uint8_t, uint8_t> > neighborIterator; // neighbor iterator for segmentaiton process

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation
    uint16_t *queueIndY;

public:
    Preprocess(string seq) {

        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;

        allocateMemory();
        resetParameters();
        cloudHandler(seq);
    }


    // 初始化各类参数以及分配内存
    void allocateMemory() {
        pcl_num = 0;

        laserCloudIn.reset(new pcl::PointCloud<PointType>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        rmGroundCloud.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());
        StatisticalOR_cloud_filtered.reset(new pcl::PointCloud<PointType>());
        //VoxelG_cloud_filtered.reset(new pcl::PointCloud<PointType>());
        PassT_cloud_filtered1.reset(new pcl::PointCloud<PointType>());
        PassT_cloud_filtered2.reset(new pcl::PointCloud<PointType>());
        RandomS_cloud_filtered.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        // labelComponents函数中用到了这个矩阵
        // 该矩阵用于求某个点的上下左右4个邻接点
        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1;
        neighbor.second =  0;
        neighborIterator.push_back(neighbor);
        neighbor.first =  0;
        neighbor.second =  1;
        neighborIterator.push_back(neighbor);
        neighbor.first =  0;
        neighbor.second = -1;
        neighborIterator.push_back(neighbor);
        neighbor.first =  1;
        neighbor.second =  0;
        neighborIterator.push_back(neighbor);

        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

    // 初始化/重置各类参数内容
    void resetParameters() {
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        rmGroundCloud->clear();
        outlierCloud->clear();
        StatisticalOR_cloud_filtered->clear();
        //VoxelG_cloud_filtered->clear();
        PassT_cloud_filtered1->clear();
        PassT_cloud_filtered2->clear();
        RandomS_cloud_filtered->clear();
// 这些填充点，在后面判断是否初始化，很有效
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;

        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
    }

    ~Preprocess() {
        //delete allPushedIndX; // array for tracking points of a segmented object
        //delete allPushedIndY;

        //delete queueIndX; // array for breadth-first search process of segmentation
        //delete queueIndY;
    }

    void cloudHandler(const string seq) {

        int nTimes = getTimeStamp(seq);
        stringstream ss;
        ss << setfill('0') << setw(2) << seq;
        string strPathToSequence = "/media/qzj/Document/grow/research/slamDataSet/kitti/data_odometry_velodyne/dataset/sequences/" + ss.str() + "/velodyne/";
//         string strPathToSave = "/media/qzj/Document/grow/research/slamDataSet/kitti/data_odometry_velodyne/dataset/downSample/bin/" + ss.str() + "/velodyne/";
        string strPathToSave = "/media/qzj/Document/grow/research/slamDataSet/kitti/data_odometry_velodyne/dataset/downSample/pcd2/" + ss.str() + "/velodyne/";
        cout<<"strPathToSequence: "<<strPathToSequence.c_str()<<endl;
        cout<<"strPathToSave: "<<strPathToSave.c_str()<<endl;
        cout<<"bin to save: "<<nTimes<<endl;
        cout<<"x y z intensity"<<endl;

        createDirectory(strPathToSave);

        for(pcl_num=0; pcl_num<nTimes; pcl_num++)
        {
            //allocateMemory();
            //ROS_INFO("%s to save",binfileToSave.c_str());
            stringstream ss;
            ss << setfill('0') << setw(6) << pcl_num;
            string binfile = strPathToSequence + ss.str() + ".bin";
            string binfileToSave = strPathToSave + ss.str() + ".bin";
            // 1. Convert ros message to pcl point cloud
// 			cout<<"load1"<<endl;
            loadBin(binfile, laserCloudIn);

            // 3. Range image projection
// 			cout<<"load3"<<endl;
            projectPointCloud();
            
            // 4. Mark ground points
// 			cout<<"load4"<<endl;
            groundRemoval();
            // 5. Point cloud segmentation
// 			cout<<"load5"<<endl;
//             cloudSegmentation();
            // 6. Publish all clouds
// 			cout<<"load6"<<endl;
            saveCloud(binfileToSave);
            // 7. Reset parameters for next iteration
// 			cout<<"load7"<<endl;
            resetParameters();
// 			cout<<"load8"<<endl;
        }
        cout<<" preprocess finish."<<endl;
    }


    void PreprocessPCL(pcl::PointCloud<PointType>::Ptr input_pointcloud)
    {
        float threshold = 30.0f;

        pcl::StatisticalOutlierRemoval<PointType> statistical;
        statistical.setInputCloud(input_pointcloud);
        statistical.setMeanK(30);                                  //取平均值的临近点数
        statistical.setStddevMulThresh(3);                         //设置判断是否为离群点的阀值
        statistical.filter(*StatisticalOR_cloud_filtered);

// 		cout<<pcl_num<<"th"<<" "<<StatisticalOR_cloud_filtered->points.size();

        pcl::PointXYZI point;
        point.x = 1000.f;
        point.y = 1000.f;
        point.z = 1000.f;

        pcl::PassThrough<PointType> pass;
        StatisticalOR_cloud_filtered->push_back(point);
        pass.setInputCloud (StatisticalOR_cloud_filtered);                //设置输入点云
        pass.setFilterFieldName ("y");             //设置过滤时所需要点云类型的XYZ字段
        pass.setFilterLimits (-threshold, threshold);           //设置在过滤字段的范围
        //pass.setFilterLimitsNegative (true);     //设置保留范围内还是过滤掉范围内
        pass.filter (*PassT_cloud_filtered1);       //执行滤波，保存过滤结果在cloud_filtered

// 		cout<<" "<<PassT_cloud_filtered1->points.size();

        PassT_cloud_filtered1->push_back(point);
        pass.setInputCloud (PassT_cloud_filtered1);                //设置输入点云
        pass.setFilterFieldName ("x");             //设置过滤时所需要点云类型的XYZ字段
        pass.setFilterLimits (-threshold, threshold);           //设置在过滤字段的范围
        //pass.setFilterLimitsNegative (true);     //设置保留范围内还是过滤掉范围内
        pass.filter (*PassT_cloud_filtered2);       //执行滤波，保存过滤结果在cloud_filtered

// 		cout<<" "<<PassT_cloud_filtered2->points.size();
        if(PassT_cloud_filtered2->points.size()>OUT_POINTS)
        {
            pcl::RandomSample<PointType> rs;
            rs.setInputCloud(PassT_cloud_filtered2);
            rs.setSample(OUT_POINTS);                        //设置输出点的数量
            rs.filter(*RandomS_cloud_filtered);        //下采样并输出到cloud_out
        }
        else
            pcl::copyPointCloud(*PassT_cloud_filtered2,  *RandomS_cloud_filtered);


// 		cout<<" "<<RandomS_cloud_filtered->points.size()<<endl;
        if(RandomS_cloud_filtered->points.size()<2048)
            cout<<pcl_num<<"th"<<" "<<StatisticalOR_cloud_filtered->points.size()<<" "<<PassT_cloud_filtered1->points.size()<<" "<<PassT_cloud_filtered2->points.size()<<" "<<RandomS_cloud_filtered->points.size()<<endl;
        //ROS_INFO("%dth %d %d %d %d ",pcl_num,,PassT_cloud_filtered1->points.size(),PassT_cloud_filtered2->points.size(),RandomS_cloud_filtered->points.size());

        pcl::copyPointCloud(*RandomS_cloud_filtered,  *input_pointcloud);
// 		for(int itr = 0; itr!=input_pointcloud->points.size(); itr++) {
// 			point = input_pointcloud->at(itr);
// 			input_pointcloud->at(itr).x = point.x / threshold;
// 			input_pointcloud->at(itr).y = point.y / threshold;
// 			input_pointcloud->at(itr).z = point.z / threshold;
// 		}
    }


    void projectPointCloud() {
        // range image projection
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize;
        PointType thisPoint;

        cloudSize = laserCloudIn->points.size();
//         double isEmpty[N_SCAN*Horizon_SCAN]={DBL_MAX};
//         for(int i=0;i<N_SCAN*Horizon_SCAN;i++)
//             isEmpty[i]=DBL_MAX;
        for (size_t i = 0; i < cloudSize; ++i) {
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;

            // 计算竖直方向上的角度（雷达的第几线）
            verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;

            // rowIdn计算出该点激光雷达是竖直方向上第几线的
            // 从下往上计数，-5度记为初始线，第0线，一共16线(N_SCAN=16)
            rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            // atan2(y,x)函数的返回值范围(-PI,PI],表示与复数x+yi的幅角
            // 下方角度atan2(..)交换了x和y的位置，计算的是与y轴正方向的夹角大小(关于y=x做对称变换)
            // 这里是在雷达坐标系，所以是与正前方的夹角大小
            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            // round函数进行四舍五入取整
            // 这边确定不是减去180度???  不是
            // 雷达水平方向上某个角度和水平第几线的关联关系???关系如下：
            // horizonAngle:(-PI,PI],columnIdn:[H/4,5H/4]-->[0,H] (H:Horizon_SCAN)
            // 下面是把坐标系绕z轴旋转,对columnIdn进行线性变换
            // x+==>Horizon_SCAN/2,x-==>Horizon_SCAN
            // y+==>Horizon_SCAN*3/4,y-==>Horizon_SCAN*5/4,Horizon_SCAN/4
            //
            //          3/4*H
            //          | y+
            //          |
            // (x-)H---------->H/2 (x+)
            //          |
            //          | y-
            //    5/4*H   H/4
            //
            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;
            // 经过上面columnIdn -= Horizon_SCAN的变换后的columnIdn分布：
            //          3/4*H
            //          | y+
            //     H    |
            // (x-)---------->H/2 (x+)
            //     0    |
            //          | y-
            //         H/4
            //
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            if (range < 0.1)
                continue;

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;
//             if(range<isEmpty[index])
//             {
            fullCloud->points[index] = thisPoint;
            fullInfoCloud->points[index] = thisPoint;
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
//                 isEmpty[index]=range;
//             }
        }
    }


    void groundRemoval() {
        size_t lowerInd, upperInd;
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < Horizon_SCAN; ++j) {
//             说明这7个线束，都是朝下的，会扫到地面
            // groundScanInd 是在 utility.h 文件中声明的线数，groundScanInd=7
            for (size_t i = 0; i < groundScanInd; ++i) {

                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                // 初始化的时候用nanPoint.intensity = -1 填充
                // 都是-1 证明是空点nanPoint
//                 有些点是没有填充的
//                 fullCloud是已经映射到二维图上的
                if (fullCloud->points[lowerInd].intensity == -1 ||
                        fullCloud->points[upperInd].intensity == -1) {
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }

                // 由上下两线之间点的XYZ位置得到两线之间的俯仰角
                // 如果俯仰角在10度以内，则判定(i,j)为地面点,groundMat[i][j]=1
                // 否则，则不是地面点，进行后续操作
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;
// 由两点构成的向量，与水平面的夹角，如果夹角小于十度，则认为是地面
                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
                if (abs(angle - sensorMountAngle) <= 10.0) {
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
                //else
                //ROS_INFO("%f",angle);
            }
        }

        // 找到所有点中的地面点或者距离为FLT_MAX(rangeMat的初始值)的点，并将他们标记为-1
        // rangeMat[i][j]==FLT_MAX，代表的含义是什么？ 无效点
        for (size_t i = 0; i < N_SCAN; ++i) {
            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX) {
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }
    }

    void cloudSegmentation() {
//         存到 labelMat里
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                // 如果labelMat[i][j]=0,表示没有对该点进行过分类
                // 需要对该点进行聚类
                if (labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        for (size_t i = 0; i < N_SCAN; ++i) {


            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                // 找到可用的特征点或者地面点(不选择labelMat[i][j]=0的点)
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1) {
                    // labelMat数值为999999表示这个点是因为聚类数量不够5而被舍弃的点,要跳过
                    if (labelMat.at<int>(i,j) == 999999) {
                        // 当列数为5的倍数，并且行数较大，可以认为非地面点的，将它保存进异常点云(界外点云)中
                        if (i > groundScanInd && j % 5 == 0) {
//                             该操作不会使fullcloud发生变化
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        } else {
                            continue;
                        }
                    }

                    // 如果是地面点,对于列数不为5的倍数的，直接跳过不处理
                    if (groundMat.at<int8_t>(i,j) == 1) {
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue;
                    }

                    segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    ++sizeOfSegCloud;
                }
            }

        }

        // 如果有节点订阅SegmentedCloudPure,
        // 那么把点云数据保存到segmentedCloudPure中去
        //if (pubSegmentedCloudPure.getNumSubscribers() != 0)
        {
            for (size_t i = 0; i < N_SCAN; ++i) {
                for (size_t j = 0; j < Horizon_SCAN; ++j) {
                    // 需要选择不是地面点(labelMat[i][j]!=-1)和没被舍弃的点
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999) {
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                    if (labelMat.at<int>(i,j) > 0 ) {
                        rmGroundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        rmGroundCloud->points.back().intensity = labelMat.at<int>(i,j);
                    }

                }
            }
        }
    }

    void labelComponents(int row, int col) {
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY;
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;

        // 标准的BFS
        // BFS的作用是以(row，col)为中心向外面扩散，
        // 判断(row,col)是否是这个平面中一点
        while(queueSize > 0) {
//             初始为函数输入的坐标
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
//             后面queueSize还会再加，最多加四次，也就是如果加四次的话，这里至少减四次，才能跳出循环，也就是以一个十字形的方式像一个圆一样去传播
            --queueSize;
            ++queueStartInd;
            // labelCount的初始值为1，后面会递增（他是个类变量，当聚类一次后，就聚下此类）
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;

            // neighbor=[[-1,0];[0,1];[0,-1];[1,0]]
            // 遍历点[fromIndX,fromIndY]边上的四个邻点
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter) {

                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;

                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;

                // 是个环状的图片，左右连通
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;

                // 如果点[thisIndX,thisIndY]已经标记过
                // labelMat中，999999需要舍弃的点，0代表未进行标记过，其余为其他的标记
                // 如果当前的邻点已经标记过，则跳过该点。
                // 如果labelMat已经标记为正整数，则已经聚类完成，不需要再次对该点聚类
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY),
                              rangeMat.at<float>(thisIndX, thisIndY));

                // alpha代表角度分辨率，
                // X方向上角度分辨率是segmentAlphaX(rad)
                // Y方向上角度分辨率是segmentAlphaY(rad)
                if ((*iter).first == 0)
//                     ang_res_x / 180.0 * M_PI;
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                // 通过下面的公式计算这两点之间是否有平面特征
                // atan2(y,x)的值越大，d1，d2之间的差距越小,越平坦
                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

//                 这两个点近似在一个平面上
                if (angle > segmentTheta) {
                    // segmentTheta=1.0472<==>60度
                    // 如果算出角度大于60度，则假设这是个平面
//                     在这里对这个序列进行更新，再循环一次也就是所谓的BFS
                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }


        bool feasibleSegment = false;

        // 如果聚类超过segmentValidNum个点，直接标记为一个可用聚类，labelCount需要递增
        if (allPushedIndSize >= segmentValidNum)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum) {
            // 如果聚类点数小于5大于等于5，统计竖直方向上的聚类点数
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;

            // 竖直方向上超过3个也将它标记为有效聚类
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;
        }

        if (feasibleSegment == true) {
            ++labelCount;
        } else {
            for (size_t i = 0; i < allPushedIndSize; ++i) {
                // 标记为999999的是需要舍弃的聚类的点，因为他们的数量小于5个
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    // 发布各类点云内容
    void saveCloud(string path) {

//         PreprocessPCL(segmentedCloudPure);
//         writeKittiPclBinData(segmentedCloudPure,path);
        
        //replace_str(path,"bin","pcd");
        //createDirectory(path);
        //pcl::io::savePCDFileASCII(path, *segmentedCloudPure);

//         writeKittiPclBinData(rmGroundCloud,path);
        replace_str(path,"bin","pcd");
        createDirectory(path);
//         pcl::io::savePCDFileASCII(path, *rmGroundCloud);

    }
};


int rmGround(string seq) {

    Preprocess IP(seq);

    return 1;
}
