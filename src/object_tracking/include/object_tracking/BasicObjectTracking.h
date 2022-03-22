#pragma once 
#include <string>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

#include "DetectedObject.h"
#include "object_tracking/DetectedObjectMsg.h"

namespace tracking
{
    class BasicObjectTracking
    {
        public:
        typedef pcl::PointXYZI Point;
        typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
        public:
        explicit BasicObjectTracking(ros::NodeHandle& nh);
        virtual ~BasicObjectTracking();
        virtual void process();

        protected:

        virtual void readParams(ros::NodeHandle& nh);
        virtual void msgHandle(const sensor_msgs::PointCloud2ConstPtr& raw_pointCloud);

        virtual void pointCloudFilter(PointCloud::Ptr& in_pc_ptr, PointCloud::Ptr& filtered_pc_ptr);
        void removeGroundPoint(PointCloud::Ptr& in_pc_ptr);
        void extractInitialSeeds(PointCloud::Ptr& in_pc_ptr);
        void estimatePlane();
        virtual void clusterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& no_ground_pc_ptr, std::vector<pcl::PointIndices>& local_indices); 
        virtual void segmentObject(PointCloud::Ptr& no_ground_pc_ptr);
        inline void clearPointCloud()
        {
            ground_pointCloud_->clear();
            no_ground_pointCloud_->clear();
        }
        protected:
        std_msgs::Header shared_header_;
        ros::NodeHandle& nh_;

        ros::Subscriber sub_pointCloud_;
        ros::Publisher pub_ground_points_;
        ros::Publisher pub_no_ground_points_;
        ros::Publisher pub_object_;
        ros::Publisher pub_bounding_box_;
        ros::Publisher pub_marker_;
        
        std::string topic_name_sub_pointCloud_;
        std::string topic_name_sub_odometry_;
        std::string topic_name_pub_ground_pointCloud_;
        std::string topic_name_pub_no_ground_pointCloud_;
        std::string topic_name_pub_object_;
        std::string topic_name_pub_bounding_box_;
        std::string topic_name_pub_marker_;

        PointCloud::Ptr ground_pointCloud_;
        PointCloud::Ptr no_ground_pointCloud_;

        /*----------------------------------- */
        int num_iter_;      // 计算地面迭代次数
        int num_LPR_;       // Lowest Point Representative
        double thres_seeds_;    
        double thres_dist_;
        /*----------------------------------- */
        Eigen::Vector3f ground_norm_vector_;
        float ground_d_;
        /*----------------------------------- */
        double max_cluster_dist_;
        int min_cluster_size_;
        int max_cluster_size_; 


        
        

    };
}