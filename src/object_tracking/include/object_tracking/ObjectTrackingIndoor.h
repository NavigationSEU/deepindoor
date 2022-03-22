#pragma once
#include "ObjectTracking.h"

/**
 * 适用于室内的静载体的目标跟踪，相比于ObjectTracking增加的点如下：
 * ------------------------------------------------------------
 * 1. 滤除走廊两边的墙面点云
 * 2. 对聚类的物体增加了检验环节
 * 3. 3d聚类变成2d聚类
 */ 
namespace tracking
{
    class ObjectTrackingIndoor : virtual public ObjectTracking
    {
        public:
        explicit ObjectTrackingIndoor(ros::NodeHandle& nh);
        virtual ~ObjectTrackingIndoor();
        void process();

        protected:
        virtual void readParams(ros::NodeHandle& nh);
        virtual void msgHandle(const sensor_msgs::PointCloud2ConstPtr& raw_pointCloud) override;
        void msgHandleSimple(const sensor_msgs::PointCloud2ConstPtr& raw_pointCloud);       // 简化版没有去除地面点云的操作
        virtual void segmentObject(PointCloud::Ptr& no_ground_pc_ptr, std::vector<pcl::PointIndices>& indices_vector);
        virtual void pointCloudFilter(PointCloud::Ptr& in_pc_ptr, PointCloud::Ptr& filtered_pc_ptr) override;   // 增加把房顶去掉
        virtual void clusterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& no_ground_pc_ptr, std::vector<pcl::PointIndices>& local_indices) override;
        void connectedComponentClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& no_ground_pc_ptr, std::vector<pcl::PointIndices>& local_indices, float range);
        void searchCell(std::array<std::array<int,100>,100>& grid_occupy, int index_x, int index_y, int cluster_id);
        void removeBackgroundPoint(PointCloud::Ptr& no_ground_pc_ptr);
        bool checkObject(pcl::PointCloud<pcl::PointXYZ>::Ptr& checked_obj_ptr);
        void removeDynamicObject(PointCloud::Ptr& in_pc_ptr, std::vector<pcl::PointIndices>& indices_vector);
    
        protected:
        ros::Publisher pub_no_background_points_;
        ros::Publisher pub_static_points_;

        std::string topic_name_pub_no_background_pointCloud_;
        std::string topic_name_pub_static_pointCloud_;

        PointCloud::Ptr no_background_pointCloud_;
        PointCloud::Ptr background_pointCloud_;
        PointCloud::Ptr static_pointCloud_;

        protected:
        enum cellType{LEFT_TOP_CELL, LEFT_BOTTOM_CELL, RIGHT_TOP_CELL, RIGHT_BOTTOM_CELL,
        LEFT_EDGE_CELL, RIGHT_EDGE_CELL, TOP_EDGE_CELL, BOTTOM_EDGE_CELL, NORNAL_CELL};
    

    };
}