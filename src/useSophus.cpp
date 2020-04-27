#include "useSophus.h"
#include "rotate.h"
void UseSophus( void )
{
    // 沿Z轴转90度的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    
    Sophus::SO3d SO3_R(R);               // Sophus::SO(3)可以直接从旋转矩阵构造
    Eigen::Quaterniond q(R);            // 或者四元数
    Sophus::SO3d SO3_q( q );
    // 上述表达方式都是等价的
    // 输出SO(3)时，以so(3)形式输出
    cout<<"SO(3) from matrix: "<<SO3_R.matrix()<<endl;
    cout<<"SO(3) from quaternion :"<<SO3_q.matrix()<<endl;
    
    // 使用对数映射获得它的李代数
    Eigen::Vector3d so3 = SO3_R.log();
    cout<<"so3 = "<<so3.transpose()<<endl;
    // hat 为向量到反对称矩阵
    cout<<"so3 hat=\n"<<Sophus::SO3d::hat(so3)<<endl;
    // 相对的，vee为反对称到向量
    cout<<"so3 hat vee= "<<Sophus::SO3d::vee( Sophus::SO3d::hat(so3) ).transpose()<<endl; // transpose纯粹是为了输出美观一些
    
    // 增量扰动模型的更新
    Eigen::Vector3d update_so3(1e-4, 0, 0); //假设更新量为这么多
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3)*SO3_R;
    cout<<"SO3d updated = "<<SO3_updated.matrix()<<endl;
    
    /********************萌萌的分割线*****************************/

    
}


void se3andSE3(void)
{
    Eigen::Matrix3d R;
    Vector3d euler(45.0,50.0,30.0);
    EulerToRoMat(euler/180.0*M_PI,R);
    cout<<euler.transpose()/180.0*M_PI<<endl;
    Eigen::Vector3d t(1,2,1);           // 沿X轴平移1
    
    Sophus::SE3d SE3_Rt(R, t);           // 从R,t构造SE(3)
    //输出的so3李代数和真正的平移量
    cout<<"SE3 from R,t= "<<endl<<SE3_Rt.matrix()<<endl;
    cout<<"SE3 matrix= "<<endl<<SE3_Rt.matrix()<<endl;
    // 李代数se(3) 是一个六维向量，方便起见先typedef一下
    typedef Eigen::Matrix<double,6,1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout<<"se3 = "<<se3.transpose()<<endl;
//     cout<<"SE3 = "<<expmap_se3(se3)<<endl;
    // 观察输出，会发现在Sophus中，se(3)的平移在前，旋转在后.
    // 同样的，有hat和vee两个算符
    cout<<"se3 hat = "<<endl<<Sophus::SE3d::hat(se3)<<endl;
    cout<<"se3 hat vee = "<<Sophus::SE3d::vee( Sophus::SE3d::hat(se3) ).transpose()<<endl;
    
    // 最后，演示一下更新
    Vector6d update_se3; //更新量
    update_se3<<1,1,1,0,0,0;
    //sophus里的exp变换，并不是理论上的对李代数求exp变换，要注意
    Sophus::SE3d SE3_im=Sophus::SE3d::exp(update_se3);
    cout<<SE3_im.matrix()<<endl;
    Sophus::SE3d SE3_updated = SE3_im*SE3_Rt;
    cout<<"SE3 updated = "<<endl<<SE3_updated.matrix()<<endl;
    cout<<R<<endl;
    
}

Sophus::SE3d getSE3(Vector3d euler,Eigen::Vector3d t)
{
    Eigen::Matrix3d R;
    EulerToRoMat(euler/180.0*M_PI,R);
    Sophus::SE3d SE3_Rt(R, t);           // 从R,t构造SE(3)
    return SE3_Rt;
}

