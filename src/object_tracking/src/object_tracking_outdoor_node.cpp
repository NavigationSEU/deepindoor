#include "ObjectTrackingOutdoor.h"

int main(int argc, char** argv)
{
    std::cerr<<"=================================================================="<<std::endl;
    ros::init(argc, argv, "object_tracking");
    ros::NodeHandle nh;

    tracking::ObjectTrackingOutdoor tracking(nh);
    tracking.process();

    return 0;
}