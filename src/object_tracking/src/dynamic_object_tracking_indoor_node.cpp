#include "ObjectTrackingIndoorDynamicCarrier.h"

int main(int argc, char** argv)
{
    std::cerr<<"=================================================================="<<std::endl;
    ros::init(argc, argv, "dynamic_object_tracking_indoor");
    ros::NodeHandle nh;

    tracking::ObjectTrackingIndoorDynamicCarrier tracking(nh);
    tracking.process();

    return 0;
}