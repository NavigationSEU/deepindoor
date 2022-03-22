#include "ObjectTracking.h"
#include "ObjectTrackingDynamicCarrier.h"

int main(int argc, char** argv)
{
    std::cerr<<"=================================================================="<<std::endl;
    ros::init(argc, argv, "dynamic_object_tracking");
    ros::NodeHandle nh;

    tracking::ObjectTrackingDynamicCarrier tracking(nh);
    tracking.process();

    return 0;
}