#include "ObjectTracking.h"

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

// 只转中心坐标按理说长宽高会出现问题

// 第一帧不转会有隐含问题 如果SLAM比tracking先跑了一段时间的话
// 1-10-11
// tracking 1 SLAM 4 SLAM 4 odom  --> SLAM
// 


namespace tracking
{
    class ObjectTrackingDynamicCarrier : virtual public ObjectTracking 
    {
        public:
        explicit ObjectTrackingDynamicCarrier(ros::NodeHandle& nh);
        virtual ~ObjectTrackingDynamicCarrier();
        virtual void readParams(ros::NodeHandle& nh) override; // 参数名字换了
        virtual void process() override;    // 需要增加订阅

        protected:
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicy;
        typedef message_filters::Synchronizer<SyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
  
        message_filters::Subscriber<sensor_msgs::PointCloud2> sub_sync_cloud_;
        message_filters::Subscriber<nav_msgs::Odometry> sub_sync_odom_;

        Eigen::Affine3f odom_affine_;
        Eigen::Matrix3f lidar_rotation_;
        Eigen::Vector3f lidar_translation_;

        protected:
        virtual void msgHandle(const sensor_msgs::PointCloud2ConstPtr& raw_pointCloud, const nav_msgs::OdometryConstPtr & odom_msg); 
        virtual void segmentObject(PointCloud::Ptr& no_ground_pc_ptr) override;            
        Eigen::Affine3f odo2affine(nav_msgs::Odometry odom);

    };


}
