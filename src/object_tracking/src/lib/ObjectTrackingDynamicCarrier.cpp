#include "ObjectTrackingDynamicCarrier.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <tf/tf.h>

namespace tracking
{
    ObjectTrackingDynamicCarrier::ObjectTrackingDynamicCarrier(ros::NodeHandle& nh):ObjectTracking(nh)
    {

    }

    ObjectTrackingDynamicCarrier::~ObjectTrackingDynamicCarrier()
    {

    }

    void ObjectTrackingDynamicCarrier::readParams(ros::NodeHandle& nh) 
    {
        nh.param<int>("/dynamic_object_tracking_node/num_iter", num_iter_, 3);
        nh.param<int>("/dynamic_object_tracking_node/num_LPR", num_LPR_, 100);
        nh.param<double>("/dynamic_object_tracking_node/thres_seeds", thres_seeds_, 0.8);
        nh.param<double>("/dynamic_object_tracking_node/thres_dist", thres_dist_, 0.15);

        nh.param<double>("/dynamic_object_tracking_node/max_cluster_dist", max_cluster_dist_, 0);
        nh.param<int>("/dynamic_object_tracking_node/max_cluster_size", max_cluster_size_, 0);
        nh.param<int>("/dynamic_object_tracking_node/min_cluster_size", min_cluster_size_, 0);

        nh.param<std::string>("/dynamic_object_tracking_node/topic_name_sub_pointCloud", topic_name_sub_pointCloud_, "");
        nh.param<std::string>("/dynamic_object_tracking_node/topic_name_sub_odometry", topic_name_sub_odometry_, "");
        nh.param<std::string>("/dynamic_object_tracking_node/topic_name_pub_ground_pointCloud", topic_name_pub_ground_pointCloud_, "/object_tracking/ground_pointCloud");
        nh.param<std::string>("/dynamic_object_tracking_node/topic_name_pub_no_ground_pointCloud", topic_name_pub_no_ground_pointCloud_, "/object_tracking/no_ground_pointCloud");
        nh.param<std::string>("/dynamic_object_tracking_node/topic_name_pub_object", topic_name_pub_object_, "/object_tracking/object");
        nh.param<std::string>("/dynamic_object_tracking_node/topic_name_pub_bounding_box", topic_name_pub_bounding_box_, "/object_tracking/bounding_box");
        nh.param<std::string>("/dynamic_object_tracking_node/topic_name_pub_marker", topic_name_pub_marker_, "/object_tracking/marker");

    }

    void ObjectTrackingDynamicCarrier::process()
    {
        readParams(nh_);   // 这个读取参数到底有什么问题

        sub_sync_cloud_.subscribe(nh_, topic_name_sub_pointCloud_, 10);
        sub_sync_odom_.subscribe(nh_, topic_name_sub_odometry_, 10);

        sync_.reset(new Sync(SyncPolicy(5), sub_sync_cloud_, sub_sync_odom_));
        sync_->registerCallback(boost::bind(&ObjectTrackingDynamicCarrier::msgHandle, this, _1, _2));

        pub_ground_points_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name_pub_ground_pointCloud_, 5);
        pub_no_ground_points_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name_pub_no_ground_pointCloud_, 5);
        pub_bounding_box_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(topic_name_pub_bounding_box_, 55);
        pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_name_pub_marker_, 5);

        ros::spin();

    }


    void ObjectTrackingDynamicCarrier::segmentObject(PointCloud::Ptr& no_ground_pc_ptr)
    {
        // XYZI -> XYZ
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*no_ground_pc_ptr, *in_pc_ptr);

        // 聚类分割
        std::vector<pcl::PointIndices> local_indices;
        clusterPointCloud(in_pc_ptr, local_indices);
        
        if(is_initial_status)
        {
            // 初始状态，所有object 均构造放入obj_list中
            // is_initial_status = false;
            // 统计每个聚类属性
            for(size_t i=0; i<local_indices.size();++i)
            {
                float min_x = std::numeric_limits<float>::max();
                float max_x = -std::numeric_limits<float>::max();
                float min_y = std::numeric_limits<float>::max();
                float max_y = -std::numeric_limits<float>::max();
                float min_z = std::numeric_limits<float>::max();
                float max_z = -std::numeric_limits<float>::max();
                pcl::PointXYZ centroid;

                for(auto indices_iter = local_indices[i].indices.begin();
                indices_iter != local_indices[i].indices.end(); ++indices_iter)
                {
                    pcl::PointXYZ p;
                    p.x = in_pc_ptr->points[*indices_iter].x;
                    p.y = in_pc_ptr->points[*indices_iter].y;
                    p.z = in_pc_ptr->points[*indices_iter].z;

                    centroid.x += p.x;
                    centroid.y += p.y;
                    centroid.z += p.z;

                    if (p.x < min_x)
                    {
                        min_x = p.x;
                    }
                    if (p.y < min_y)
                    {
                        min_y = p.y;
                    }
                    if (p.z < min_z)
                    {
                        min_z = p.z;
                    }
                    if (p.x > max_x)
                    {
                        max_x = p.x;
                    }
                    if (p.y > max_y)
                    {
                        max_y = p.y;
                    }
                    if (p.z > max_z)
                    {
                        max_z = p.z;
                    }
                }

                centroid.x /= local_indices[i].indices.size();
                centroid.y /= local_indices[i].indices.size();
                centroid.z /= local_indices[i].indices.size();

                float length = max_x - min_x;
                float width = max_y - min_y;
                float height = max_z - min_z;

                // 不要大物体
                if((length > 1.5|| length < 0.2) || (width > 1.5|| width < 0.2) || height > 2.2){continue;}

                // 转到SLAM第一帧坐标系
                Eigen::Vector3f centroid_initial_current_frame(centroid.x, centroid.y, centroid.z);
                Eigen::Vector3f centroid_initial_global_frame(0,0,0);
                centroid_initial_global_frame = lidar_rotation_*centroid_initial_current_frame + lidar_translation_;
                DetectedObject obj(length, width, height, pcl::PointXYZ(centroid_initial_global_frame[0], centroid_initial_global_frame[1], centroid_initial_global_frame[2]), shared_time_, obj_label_,10);
                obj.setCurrentFrameCentroid(centroid);
                // obj.note();
                ++obj_label_;
                obj_vec_.push_back(obj);
            }

        }

        else
        {
            // 非初始状态，和前一时刻做关联 obj_list 中不存在的物体才需要构造， 存在的改变属性即可
            for(size_t i=0; i<local_indices.size();++i)
            {
                float min_x = std::numeric_limits<float>::max();
                float max_x = -std::numeric_limits<float>::max();
                float min_y = std::numeric_limits<float>::max();
                float max_y = -std::numeric_limits<float>::max();
                float min_z = std::numeric_limits<float>::max();
                float max_z = -std::numeric_limits<float>::max();
                pcl::PointXYZ centroid;

                for(auto indices_iter = local_indices[i].indices.begin();
                indices_iter != local_indices[i].indices.end(); ++indices_iter)
                {
                    pcl::PointXYZ p;
                    p.x = in_pc_ptr->points[*indices_iter].x;
                    p.y = in_pc_ptr->points[*indices_iter].y;
                    p.z = in_pc_ptr->points[*indices_iter].z;

                    centroid.x += p.x;
                    centroid.y += p.y;
                    centroid.z += p.z;

                    if (p.x < min_x)
                    {
                        min_x = p.x;
                    }
                    if (p.y < min_y)
                    {
                        min_y = p.y;
                    }
                    if (p.z < min_z)
                    {
                        min_z = p.z;
                    }
                    if (p.x > max_x)
                    {
                        max_x = p.x;
                    }
                    if (p.y > max_y)
                    {
                        max_y = p.y;
                    }
                    if (p.z > max_z)
                    {
                        max_z = p.z;
                    }
                }

                centroid.x /= local_indices[i].indices.size();
                centroid.y /= local_indices[i].indices.size();
                centroid.z /= local_indices[i].indices.size();

                float length = max_x - min_x;
                float width = max_y - min_y;
                float height = max_z - min_z;

                // 不要大物体
                if((length > 1.5|| length < 0.2) || (width > 1.5|| width < 0.2) || height > 2.2){continue;}


                // 坐标转换到第一帧坐标系上  
                Eigen::Vector3f centroid_current_frame(centroid.x, centroid.y, centroid.z);
                Eigen::Vector3f centroid_global_frame(0,0,0);
                centroid_global_frame = lidar_rotation_*centroid_current_frame + lidar_translation_;
                

                // 关联
                std::vector<float> dist(obj_vec_.size());
                std::vector<int> dist_ind(obj_vec_.size());
                int count = 0;
                for(auto& obj:obj_vec_)
                {
                    dist_ind[count] = count;
                    float delta_x = centroid_global_frame[0] - obj.centroid().x;
                    float delta_y = centroid_global_frame[1] - obj.centroid().y;
                    float delta_z = centroid_global_frame[2] - obj.centroid().z;
                    dist[count] = sqrt(pow(delta_x,2) + pow(delta_y,2) + pow(delta_z,2));
                    ++count;
                }
                std::sort(dist_ind.begin(),dist_ind.end(),
                [&dist](int i1, int i2){return dist[i1] < dist[i2];});

                // 与上时刻该物体的距离在阈值之内，关联成功，改数据即可，无需重构
                if(dist[dist_ind[0]] < THRES_ASSOCIATION)
                {
                    obj_vec_[dist_ind[0]].setCurrentFrameCentroid(centroid);
                    obj_vec_[dist_ind[0]].setCentroid(pcl::PointXYZ(centroid_global_frame[0],centroid_global_frame[1],centroid_global_frame[2]));
                    obj_vec_[dist_ind[0]].setLength(length);
                    obj_vec_[dist_ind[0]].setWidth(width);
                    obj_vec_[dist_ind[0]].setHeight(height);
                    obj_vec_[dist_ind[0]].setNtime(shared_time_);
                    obj_vec_[dist_ind[0]].is_watched = true;
 
                    // 如果记录的数据足够，开始对速度和加速度进行估计，并可以通过kalman估计位置和长宽高
                    if(obj_vec_[dist_ind[0]].isStartCalState())
                    {
                        obj_vec_[dist_ind[0]].calState();

                        // kalman估计位置和外形
                        obj_vec_[dist_ind[0]].kalmanEstimateState();
                    }
                }

                // 关联失败，重构
                else
                {
                    DetectedObject obj(length, width, height, pcl::PointXYZ(centroid_global_frame[0],centroid_global_frame[1],centroid_global_frame[2]), shared_time_, obj_label_, 10);
                    obj.setCurrentFrameCentroid(centroid);
                    ++obj_label_;
                    obj_vec_.push_back(obj);
                }
            }
        }
    }

    Eigen::Affine3f ObjectTrackingDynamicCarrier::odo2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        std::cerr<<"++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;
        std::cerr<<x<<"   "<<y<<"   "<<z<<std::endl;
        std::cerr<<"++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;


        tf::Quaternion q;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);


    }



    void ObjectTrackingDynamicCarrier::msgHandle(const sensor_msgs::PointCloud2ConstPtr& raw_pointCloud, const nav_msgs::OdometryConstPtr& odom_msg)
    {
        ROS_INFO("Received msgs.");
        // 里程计消息处理
        odom_affine_ = odo2affine(*odom_msg);
        lidar_rotation_ = odom_affine_.rotation();
        lidar_translation_ = odom_affine_.translation();
        std::cerr<<lidar_rotation_<<std::endl;
        std::cerr<<lidar_translation_<<std::endl;

        // 点云消息处理
        PointCloud::Ptr in_pc_ptr(new PointCloud);
        PointCloud::Ptr filtered_pc_ptr(new PointCloud);
        shared_header_ = raw_pointCloud->header;
        shared_time_ = shared_header_.stamp.toNSec();
        pcl::fromROSMsg(*raw_pointCloud, *in_pc_ptr);



        // 功能部分
        pointCloudFilter(in_pc_ptr, filtered_pc_ptr);
        clearPointCloud();
        removeGroundPoint(filtered_pc_ptr);
        segmentObject(no_ground_pointCloud_);
     

        // 发布环节   
        sensor_msgs::PointCloud2 no_ground_pc_msg;
        
        pcl::toROSMsg(*no_ground_pointCloud_, no_ground_pc_msg);
        no_ground_pc_msg.header = shared_header_;
        bounding_boxes_msg_ptr_->header = shared_header_;

        if (is_initial_status)
        {
            is_initial_status = false;
            for (auto &obj : obj_vec_)
            {
                if (obj.is_watched)
                {
                    obj.note();
                    jsk_recognition_msgs::BoundingBox temp_box_msg;
                    temp_box_msg.header = shared_header_;
                    temp_box_msg.label = obj.label();
                    temp_box_msg.pose.position.x = obj.currentFrameCentroid().x;
                    temp_box_msg.pose.position.y = obj.currentFrameCentroid().y;
                    temp_box_msg.pose.position.z = obj.currentFrameCentroid().z;
                    temp_box_msg.dimensions.x = obj.length();
                    temp_box_msg.dimensions.y = obj.width();
                    temp_box_msg.dimensions.z = obj.height();
                    bounding_boxes_msg_ptr_->boxes.push_back(temp_box_msg);

                    visualization_msgs::Marker temp_marker;
                    temp_marker.header = shared_header_;
                    temp_marker.action = visualization_msgs::Marker::ADD;
                    temp_marker.pose.orientation.w = 1.0;
                    temp_marker.id = temp_box_msg.label;
                    temp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                    temp_marker.scale.x = 0.2;
                    temp_marker.scale.y = 0.2;
                    temp_marker.scale.z = 0.2;
                    temp_marker.color.r = 0.0f;
                    temp_marker.color.g = 1.0f;
                    temp_marker.color.b = 0.0f;
                    temp_marker.color.a = 1.0;
                    temp_marker.lifetime = ros::Duration(0.3);
                    temp_marker.pose.position.x = temp_box_msg.pose.position.x;
                    temp_marker.pose.position.y = temp_box_msg.pose.position.y;
                    temp_marker.pose.position.z = temp_box_msg.pose.position.z + 0.4;
                    temp_marker.text = "ID: " + std::to_string(temp_box_msg.label) + "\n" + "X: " + std::to_string(temp_box_msg.pose.position.x) + "\n" + "Y: " + std::to_string(temp_box_msg.pose.position.y) + "\n" + "Z: " + std::to_string(temp_box_msg.pose.position.z) + "\n" + "Vel: " + std::to_string(obj.velocity()) + "\n" + "Acc: " + std::to_string(obj.acc());
                    marker_msg_ptr_->markers.push_back(temp_marker);
                }
            }
        }

        else
        {
            for (auto &obj : obj_vec_)
            {
                if (obj.is_watched)
                {
                    obj.note();
                    jsk_recognition_msgs::BoundingBox temp_box_msg;
                    temp_box_msg.header = shared_header_;
                    temp_box_msg.label = obj.label();
                    temp_box_msg.pose.position.x = obj.currentFrameCentroid().x;
                    temp_box_msg.pose.position.y = obj.currentFrameCentroid().y;
                    temp_box_msg.pose.position.z = obj.currentFrameCentroid().z;
                    temp_box_msg.dimensions.x = obj.length();
                    temp_box_msg.dimensions.y = obj.width();
                    temp_box_msg.dimensions.z = obj.height();
                    bounding_boxes_msg_ptr_->boxes.push_back(temp_box_msg);

                    visualization_msgs::Marker temp_marker;
                    temp_marker.header = shared_header_;
                    temp_marker.action = visualization_msgs::Marker::MODIFY;
                    temp_marker.pose.orientation.w = 1.0;
                    temp_marker.id = temp_box_msg.label;  
                    temp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                    temp_marker.scale.x = 0.2;
                    temp_marker.scale.y = 0.2;
                    temp_marker.scale.z = 0.2;
                    temp_marker.color.r = 0.0f;
                    temp_marker.color.g = 1.0f;
                    temp_marker.color.b = 0.0f;
                    temp_marker.color.a = 1.0;
                    temp_marker.lifetime = ros::Duration(0.3);
                    temp_marker.pose.position.x = temp_box_msg.pose.position.x;
                    temp_marker.pose.position.y = temp_box_msg.pose.position.y;
                    temp_marker.pose.position.z = temp_box_msg.pose.position.z + 0.4;
                    temp_marker.text = "ID: " + std::to_string(temp_box_msg.label) + "\n" + "X: " + std::to_string(temp_box_msg.pose.position.x) + "\n" + "Y: " + std::to_string(temp_box_msg.pose.position.y) + "\n" + "Z: " + std::to_string(temp_box_msg.pose.position.z) + "\n" + "Vel: " + std::to_string(obj.velocity()) + "\n" + "Acc: " + std::to_string(obj.acc());
                    marker_msg_ptr_->markers.push_back(temp_marker);
                }
            }
        }

        pub_no_ground_points_.publish(no_ground_pc_msg);
        pub_bounding_box_.publish(*bounding_boxes_msg_ptr_);
        pub_marker_.publish(*marker_msg_ptr_);

        // 重置状态 
        bounding_boxes_msg_ptr_->boxes.clear();
        marker_msg_ptr_->markers.clear();
        for(auto& obj:obj_vec_)
        {
            obj.is_watched = false;
        }

    }



}