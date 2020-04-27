#include "learncv.h"
#include<opencv2/opencv.hpp>
#include<iostream>
#include <stdio.h>

using namespace std;
using namespace cv;
int learnCv(void)
{
	Mat src = imread("/media/zhijian/My Book/hactl_data/map/2019-08--15-59-39_4/front2/walk.tif", 1);
	char image_name[200];//放入要读取的图片的文件名
	int isColor = 1;//彩色
	int fps = 20;//视频的帧率
	int frameWidth = src.cols;
	int frameHeight = src.rows;

	//输出视频保存位置，编码格式，帧率，尺寸，是否彩色
	VideoWriter writer("F:/test_l23.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), fps,Size(frameWidth, frameHeight), isColor);

	for (int i = 1; i < 2880; i++)
	{
		sprintf(image_name, "/media/zhijian/My Book/hactl_data/map/2019-08--15-59-39_4/front2/tra%d.tif", i);//将图片名放入字符串
		src = imread(image_name, 1);
		if (src.empty())
		{
			cout << "Error!!!Can't read the image!!!" << endl;
			break;
		}
		imshow("Output Video", src);
		waitKey(5);
		writer.write(src);//输出视频
	}
	return 0;
}
