#include "rotate.h"
#include "useSophus.h"
/*
 三点注意：
 第一，eigen里的四元数表示，实部在最后
 第二，欧拉角的顺序是yaw，pitch，roll
 第三，旋转矩阵是w系到c系，或者说n系到b系
 */


void Rotate(void)
{
    // 旋转向量使用 AngleAxis, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
    Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d ( 0,0,1 ) );     //沿 Z 轴旋转 45 度
    Eigen::Matrix3d R = rotation_vector.toRotationMatrix();
    Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );//q = Eigen::Quaterniond ( R );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
    
    // 用 AngleAxis 可以进行坐标变换
    Eigen::Vector3d v ( 1,0,0 );
    Eigen::Vector3d v_rotated = rotation_vector * v;
    //打印出他的转置（列变为行，好显示）
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;
    v_rotated = R * v;
    // 或者用旋转矩阵
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;
    // 使用四元数旋转一个向量，使用重载的乘法即可
    v_rotated = q*v; // 注意数学上是qvq^{-1}
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;
    
    Eigen::Vector3d so3(20,12,10);
    Eigen::Matrix3d deltaR=Sophus::SO3d::exp(so3).matrix();
    R=deltaR*R;
    q=deltaR*q;
    // 欧拉角: 可以将旋转矩阵直接转换成欧拉角 // ZYX顺序，即roll pitch yaw顺序
    cout<<"yaw pitch roll = "<<R.eulerAngles( 2,1,0 ).transpose()<<endl;
    cout<<"yaw pitch roll = "<<q.toRotationMatrix().eulerAngles(2,1,0).transpose()<<endl;
    
    //深蓝学院VIO作业一，验证两个旋转结果差不多（好像差很多）
    so3<<0.01,0.02,0.03;
    Eigen::Quaterniond deltaq(1,0.5*so3(0),0.5*so3(1),0.5*so3(2));//初始化顺序是按照实部，虚部顺序，和输出不同
    deltaR=Sophus::SO3d::exp(so3).matrix();
    R=R*deltaR;
    q=q*deltaq;
    cout<<"yaw pitch roll = "<<R.eulerAngles( 2,1,0 ).transpose()<<endl;
    cout<<"yaw pitch roll = "<<q.toRotationMatrix().eulerAngles(2,1,0).transpose()<<endl;
    
    // 欧氏变换矩阵使用 Eigen::Isometry
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
    T.rotate ( rotation_vector );                                     // 按照rotation_vector进行旋转
    T.pretranslate ( Eigen::Vector3d ( 1,3,4 ) );                     // 把平移向量设成(1,3,4)
    cout << "Transform matrix = \n" << T.matrix() <<endl;
    // 用变换矩阵进行坐标变换
    Eigen::Vector3d v_transformed = T*v;                              // 相当于R*v+t
    cout<<"v tranformed = "<<v_transformed.transpose()<<endl;
    
}
/***欧拉角ZYX***/
//Eigen::Vector3d eulerAngle(yaw,pitch,roll);
void EulerToQuat(Vector3d euler,Eigen::Quaterniond& quaternion)
{
    Eigen::AngleAxisd rollAngle(AngleAxisd(euler(2),Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(AngleAxisd(euler(1),Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(AngleAxisd(euler(0),Vector3d::UnitZ()));
    
    quaternion=yawAngle*pitchAngle*rollAngle;
}

void EulerToRoMat(Vector3d euler,Eigen::Matrix3d& rotation_matrix)
{
    Eigen::AngleAxisd rollAngle(AngleAxisd(euler(2),Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(AngleAxisd(euler(1),Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(AngleAxisd(euler(0),Vector3d::UnitZ()));
    
    rotation_matrix=yawAngle*pitchAngle*rollAngle;
}



