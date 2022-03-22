#pragma once
#include "ObjectTrackingIndoor.h"
#include "ObjectTrackingDynamicCarrier.h"

/**
 * 适用于室内的动载体的目标跟踪
 * ---------------------------------------
 * 
 */ 

namespace tracking
{
    class ObjectTrackingIndoorDynamicCarrier : public ObjectTrackingIndoor, public ObjectTrackingDynamicCarrier
    {
        public:
        explicit ObjectTrackingIndoorDynamicCarrier(ros::NodeHandle& nh);
        virtual ~ObjectTrackingIndoorDynamicCarrier();
        void process();

        protected:
        virtual void readParams(ros::NodeHandle& nh);
        void msgHandleSimple(const sensor_msgs::PointCloud2ConstPtr& raw_pointCloud, const nav_msgs::OdometryConstPtr & odom_msg);
        virtual void segmentObject(PointCloud::Ptr& no_ground_pc_ptr, std::vector<pcl::PointIndices>& indices_vector);
        bool checkObject(pcl::PointCloud<pcl::PointXYZ>::Ptr& checked_obj_ptr);
        
    };
}