#include "file.h"

void file(void)
{
    google::ShutdownGoogleLogging();
    FLAGS_stderrthreshold=google::INFO;
    FLAGS_colorlogtostderr=true;
    FLAGS_minloglevel= google::INFO;
    FLAGS_alsologtostderr=true;
//     ofstream fout("log.txt");
//     for(int i=0;i<10;i++)
//         fout<<"write something"<<'\t'<<i<<endl;
//     fout.close();
//     
//     ifstream fin("log.txt");
//     
//     char buf[20];
//     fin.getline(buf,20);//从文件中读取一行
//     LOG(INFO)<<buf<<endl;
//     
//     string line;
//     getline(fin,line);//读取一行转换成字符串
//     fin.close();
//     
//     LOG(INFO)<<line<<endl;
    
}

