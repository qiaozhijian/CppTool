#include "testHead.h"
// #include "learnMrpt.h"
#include "useSophus.h"
#include "rotate.h"
#include "findpoint.h"
#include "learncv.h"
#include "pcltest.h"
#include <numeric>
#include <memory>
#include "utility.h"
#include "removeGround.h"
class zhijian
{

    int a;
};

class qiao {
public:
    qiao();
    ~qiao();

    static zhijian* cam;

    static Matrix3d gets()
    {
        Matrix3d eye=Matrix3d::Zero();
        int a=0;
        return eye;

    }
};

zhijian* qiao::cam=NULL;

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    
    if(argc<2)
    {
        printf("ERROR: Please follow the example: rosrun pkg node seq \n rosrun lpd_slam preprocess 00 \n");
        return -2;
    }
	
    cout<<" preprocess Started."<<endl;
    std::string seq = argv[1];
    rmGround(seq);
//     shareTest();
    //LearnMrpt();
    //UseSophus();
    //se3andSE3();
    //Rotate();
//     txt2pcd();
    //findPoint();
    //learnCv();
    return 0;
}



