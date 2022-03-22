#pragma once
#include "BasicObjectTracking.h"
#include "DetectedObject.h"

#define MAX_NUM_OBJECT 500         // 预设存储物体容量
#define THRES_ASSOCIATION 1.5      // 物体关联的阈值
namespace tracking
{
    class ObjectTracking : public BasicObjectTracking 
    {
        public:
        explicit ObjectTracking(ros::NodeHandle& nh);
        virtual ~ObjectTracking();
        virtual void process() override;
        
        protected:
        virtual void msgHandle(const sensor_msgs::PointCloud2ConstPtr& raw_pointCloud) override;
        virtual void segmentObject(PointCloud::Ptr& no_ground_pc_ptr) override;
        protected:

        jsk_recognition_msgs::BoundingBoxArray::Ptr bounding_boxes_msg_ptr_;
        visualization_msgs::MarkerArray::Ptr marker_msg_ptr_;
        
    
        bool is_initial_status = true;
        std::vector<DetectedObject> obj_vec_;
        long int shared_time_;

        int pcd_id = 0;                         // PCD文件编号
        int obj_label_ = 0;                     // 动态物体编号
        std::vector<int> note_cluster_num_;

        protected:
        void saveToPCD(PointCloud::Ptr in_pc_ptr, std::string pcd_path);


    };
}