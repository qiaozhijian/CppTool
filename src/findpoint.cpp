#include "findpoint.h"
void generateHp(Matrix<double,2,3>& Hp,Vector3d pc)
{
    double fx=300.0,fy=300.0,px=pc(0),py=pc(1),pz=pc(2);
    Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    AngleAxisd rotation_vector ( M_PI/4, Vector3d ( 1,1,1 ) );     //沿 Vector3d 轴旋转 45 度
    rotation_matrix = rotation_vector.toRotationMatrix();
    Hp<<fx/pz,0,-fx*px/pow(pz,2),0,fy/pz,-fy*py/pow(pz,2);
    Hp=Hp*rotation_matrix;
}
void generateHx(Matrix<double,2,6>& Hx,Vector3d pc)
{
    double fx=300.0,fy=300.0,px=pc(0),py=pc(1),pz=pc(2);
    
    Hx<<fx/pz,0,-fx*px/pow(pz,2),-fx*px*py/pow(pz,2),fx+fx*pow(px/pz,2),-fx*py/pz,0,fy/pz,-fy*py/pow(pz,2),-fy-fy*pow(py/pz,2),fy*px*py/pow(pz,2),fy*px/pz;
}
MatXf pinv(MatXf x) 
{ 
    JacobiSVD<MatXf> svd(x,ComputeFullU | ComputeFullV); 
    double pinvtoler=1.e-8; 
    //tolerance 
    MatXf singularValues_inv = svd.singularValues(); 
    for ( long i=0; i<x.cols(); ++i) 
    { 
        if ( singularValues_inv(i) > pinvtoler ) 
            singularValues_inv(i)=1.0/singularValues_inv(i); 
        else singularValues_inv(i)=0; 
    } 
    cout<<svd.matrixV().rows()<<svd.matrixV().cols()<<svd.matrixU().rows()<<svd.matrixU().cols()<<endl;
    cout<<singularValues_inv.asDiagonal().rows()<<singularValues_inv.asDiagonal().cols()<<endl;
    return svd.matrixV()*singularValues_inv.asDiagonal()*svd.matrixU().transpose(); 
} 

//交换数组指定的两行，即进行行变换（具体为行交换）  
void temp(double aa[],double bb[],int n)  
{     
    int i;  
    double temp1;  
    for(i=0 ; i<n ; i++)  
    {  
        temp1 = aa[i];  
        aa[i] = bb[i];  
        bb[i] = temp1;    
    }    
}  
  
double DetM6d(Matrix6d martix)  
{     
    const int n=6;
    double array[n][n];
    int ii,jj,k,u;  
	int iter = 0;  //记录行变换的次数（交换）
    double det1=1,yin;  
 
    for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
            array[i][j]=martix(i,j);
    for(ii=0 ; ii<n; ii++)  
    {     
        if(array[ii][ii] == 0) 
	
        for(jj=ii; jj<n; jj++)  
        {    
            if(array[jj][ii] != 0)  
            {
				temp(array[ii],array[jj],n);//交换两行  
				iter ++;
			}
        }  
 
        for(k=ii+1; k<n; k++)  
		{    
          yin = -1 * array[k][ii] / array[ii][ii] ;  
 
          for(u=0; u<n; u++)  
          {   
              array[k][u] = array[k][u] + array[ii][u] * yin;  
          }  
		}  
   }  
  for(ii=0; ii<n; ii++)  //求对角线的积 即 行列式的值
      det1 = det1 * array[ii][ii];  
  //行变换偶数次符号不变
  if(iter%2 == 1)
	  det1= -det1;
   return (det1);  
} 

double det(int n, double *aa)
{
	if (n == 1)
		return aa[0];
	double *bb = new double[(n - 1)*(n - 1)];//创建n-1阶的代数余子式阵bb    
	int mov = 0;//判断行是否移动   
	double sum = 0.0;//sum为行列式的值  
	for (int arow = 0; arow<n; arow++) // a的行数把矩阵a(nn)赋值到b(n-1)  
	{
		for (int brow = 0; brow<n - 1; brow++)//把aa阵第一列各元素的代数余子式存到bb  
		{    
			mov = arow > brow ? 0 : 1; //bb中小于arow的行，同行赋值，等于的错过，大于的加一  
			for (int j = 0; j<n - 1; j++)  //从aa的第二列赋值到第n列  
			{
				bb[brow*(n - 1) + j] = aa[(brow + mov)*n + j + 1];
			}
		}
		int flag = (arow % 2 == 0 ? 1: -1);//因为列数为0，所以行数是偶数时候，代数余子式为1.  
		sum += flag* aa[arow*n] * det(n - 1, bb);//aa第一列各元素与其代数余子式积的和即为行列式
	}
	delete[]bb;
	return sum;
}


const int m=50;
void findPoint(void)
{
    Matrix<double,2,3> Hp;  
    Matrix<double,2,6> Hx;
    Matrix<double,3,3> Hp1; 
    Matrix<double,3,6> Hx1;
    Matrix<double,3,6> Hc;
    Matrix<double,m*3,6> HcAll;
    Matrix<double,m*3,6> HcAll_;
    double Pc[m][3]={ {10,10,100},{20,10,90},{10,20,110},{20,20,80} };
    int sqrtm=sqrt(m);
    for (int i=0;i<m;i++)
    {
        Pc[i][0]=(i/sqrtm+1)*10;
        Pc[i][1]=(i%sqrtm+1)*10;
        Pc[i][2]=100;
    }
    for(int i=0;i<m;i++)
    {
        generateHp(Hp,Vector3d(Pc[i][0],Pc[i][1],Pc[i][2]));
        generateHx(Hx,Vector3d(Pc[i][0],Pc[i][1],Pc[i][2]));
        Hc=pinv(Hp)*Hx;
        HcAll.block(3*i,0,3,6)<<Hc;
    }
    for(int i=0;i<m;i++)
    {
        generateHp(Hp,Vector3d(Pc[i][0],Pc[i][1],Pc[i][2]));
        generateHx(Hx,Vector3d(Pc[i][0],Pc[i][1],Pc[i][2]));
        Hp1<<Hp,0,0,1;
        Hx1<<Hx,0,0,0,0,0,0;
        Hc=Hp1.inverse()*Hx1;
        HcAll_.block(3*i,0,3,6)<<Hc;
//         cout<<endl<<Hc<<endl;
    }
    
    Eigen::Matrix<double,6,6> result;
//     cout.setf(ios::fixed);                      //功能和下一行的fixed功能一样，同时写没关系
//     cout<<fixed<<setprecision(50)<<endl;      //输出结果为1.20
    pinv(HcAll);
    result=HcAll_.transpose()*HcAll_;
    cout<<endl<<log10(result.determinant())<<endl;
    result=HcAll.transpose()*HcAll;
    cout<<endl<<log10(result.determinant())<<endl;
    
}
