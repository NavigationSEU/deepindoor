#pragma once 
#include "ObjectTrackingIndoor.h"

/**
 * 适用于室外的静载体的目标跟踪，相比于ObjectTrackingIndoor变化的点有
 * ----------------------------------------------------------
 * 1. 采用网格聚类的方式
 * 2. 检验的阈值不同，对物体的尺寸要求降低
 * 
 */ 

namespace tracking
{
    class ObjectTrackingOutdoor : public ObjectTrackingIndoor
    {
        public:
        explicit ObjectTrackingOutdoor(ros::NodeHandle& nh);
        virtual ~ObjectTrackingOutdoor();
        void process();

        protected:
        void segmentObject(PointCloud::Ptr& no_ground_pc_ptr, std::vector<pcl::PointIndices>& indices_vector);
        void msgHandleSimple(const sensor_msgs::PointCloud2ConstPtr& raw_pointCloud);
    };
}