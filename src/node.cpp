#include "freedom/freenode.h"

using namespace freedom;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "freedom");
    ros::NodeHandle nh;
    FreeNode freenode;
    std::cout << "FreeDOM!!!" << std::endl;
    ros::waitForShutdown();

    return 0;
}