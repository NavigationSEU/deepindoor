#include "ObjectTracking.h"
#include "DetectedObject.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <limits>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

namespace tracking
{
    ObjectTracking::ObjectTracking(ros::NodeHandle &nh) : BasicObjectTracking(nh),
                                                          bounding_boxes_msg_ptr_(new jsk_recognition_msgs::BoundingBoxArray),
                                                          marker_msg_ptr_(new visualization_msgs::MarkerArray)
    {
        obj_vec_.reserve(MAX_NUM_OBJECT);
    }

    ObjectTracking::~ObjectTracking()
    {
        std::cerr << "=========================================================" << std::endl;
        std::cerr << obj_vec_.size() << std::endl;
        std::cerr << "=========================================================" << std::endl;
    }

    void ObjectTracking::process()
    {
        readParams(nh_);
        sub_pointCloud_ = nh_.subscribe(topic_name_sub_pointCloud_, 5, &ObjectTracking::msgHandle, this);
        pub_ground_points_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name_pub_ground_pointCloud_, 5);
        pub_no_ground_points_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name_pub_no_ground_pointCloud_, 5);
        pub_bounding_box_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(topic_name_pub_bounding_box_, 5);
        pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_name_pub_marker_, 5);

        ros::spin();
    }

    void ObjectTracking::segmentObject(PointCloud::Ptr &no_ground_pc_ptr)
    {
        // XYZI -> XYZ
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*no_ground_pc_ptr, *in_pc_ptr);

        // 聚类分割
        std::vector<pcl::PointIndices> local_indices;
        clusterPointCloud(in_pc_ptr, local_indices);

        if (is_initial_status)
        {
            // 初始状态，所有object 均构造放入obj_list中
            // is_initial_status = false;
            // 统计每个聚类属性
            for (size_t i = 0; i < local_indices.size(); ++i)
            {
                // 把obj的属性打点好，然后实例化DetectedObject
                float min_x = std::numeric_limits<float>::max();
                float max_x = -std::numeric_limits<float>::max();
                float min_y = std::numeric_limits<float>::max();
                float max_y = -std::numeric_limits<float>::max();
                float min_z = std::numeric_limits<float>::max();
                float max_z = -std::numeric_limits<float>::max();
                pcl::PointXYZ centroid;

                for (auto indices_iter = local_indices[i].indices.begin();
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
                if (length > 1.5 || length < 0.2 || width > 1.5 || width < 0.2 || height > 2.2)
                {
                    continue;
                }

                DetectedObject obj(length, width, height, centroid, shared_time_, obj_label_);
                // obj.note();
                ++obj_label_;
                obj_vec_.push_back(obj);
            }
        }

        else
        {
            // 非初始状态，和前一时刻做关联 obj_list 中不存在的物体才需要构造， 存在的改变属性即可
            for (size_t i = 0; i < local_indices.size(); ++i)
            {
                // 把obj的属性打点好，然后实例化DetectedObject
                float min_x = std::numeric_limits<float>::max();
                float max_x = -std::numeric_limits<float>::max();
                float min_y = std::numeric_limits<float>::max();
                float max_y = -std::numeric_limits<float>::max();
                float min_z = std::numeric_limits<float>::max();
                float max_z = -std::numeric_limits<float>::max();
                pcl::PointXYZ centroid;

                for (auto indices_iter = local_indices[i].indices.begin();
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
                if (length > 2.2 || length < 0.2 || width > 2.2 || width < 0.2 || height > 2.2)
                {
                    continue;
                }

                // 关联
                std::vector<float> dist(obj_vec_.size());
                std::vector<int> dist_ind(obj_vec_.size());
                int count = 0;
                for (auto &obj : obj_vec_)
                {
                    dist_ind[count] = count;
                    float delta_x = centroid.x - obj.centroid().x;
                    float delta_y = centroid.y - obj.centroid().y;
                    float delta_z = centroid.z - obj.centroid().z;
                    dist[count] = sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));
                    ++count;
                }
                std::sort(dist_ind.begin(), dist_ind.end(),
                          [&dist](int i1, int i2)
                          { return dist[i1] < dist[i2]; });

                // 与上时刻该物体的距离在阈值之内，关联成功，改数据即可，无需重构
                if (dist[dist_ind[0]] < THRES_ASSOCIATION)
                {
                    obj_vec_[dist_ind[0]].setCentroid(centroid);
                    obj_vec_[dist_ind[0]].setLength(length);
                    obj_vec_[dist_ind[0]].setWidth(width);
                    obj_vec_[dist_ind[0]].setHeight(height);
                    obj_vec_[dist_ind[0]].setNtime(shared_time_);
                    obj_vec_[dist_ind[0]].is_watched = true;
                    // obj_vec_[dist_ind[0]].note();        发布前记录了
                    // 如果记录的数据足够，开始对速度和加速度进行估计，并可以通过kalman估计位置和长宽高
                    if (obj_vec_[dist_ind[0]].isStartCalState())
                    {
                        obj_vec_[dist_ind[0]].calState();

                        // kalman估计位置和外形
                        obj_vec_[dist_ind[0]].kalmanEstimateState();
                    }
                }

                // 关联失败，重构
                else
                {
                    DetectedObject obj(length, width, height, centroid, shared_time_, obj_label_);
                    // obj.note();   发布前记录了
                    ++obj_label_;
                    obj_vec_.push_back(obj);
                }
            }
        }
    }

    void ObjectTracking::saveToPCD(PointCloud::Ptr in_pc_ptr, std::string pcd_path)
    {
        std::string pcd_file = pcd_path + std::to_string(pcd_id) + ".pcd";
        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZI>(pcd_file, *in_pc_ptr, true);
        ++pcd_id;
    }

    void ObjectTracking::msgHandle(const sensor_msgs::PointCloud2ConstPtr &raw_pointCloud)
    {
        ROS_INFO("Received msgs.");
        PointCloud::Ptr in_pc_ptr(new PointCloud);
        PointCloud::Ptr filtered_pc_ptr(new PointCloud);
        shared_header_ = raw_pointCloud->header;
        shared_time_ = shared_header_.stamp.toNSec();

        pcl::fromROSMsg(*raw_pointCloud, *in_pc_ptr);

        // 点云另存PCD
        // saveToPCD(in_pc_ptr, "/home/wang/bag/indoor_pcd_file/");

        // 功能部分
        pointCloudFilter(in_pc_ptr, filtered_pc_ptr);
        clearPointCloud();
        removeGroundPoint(filtered_pc_ptr);
        segmentObject(no_ground_pointCloud_);
        // 另存地面点、非地面点
        // saveToPCD(ground_pointCloud_, "/home/wang/bag/indoor_pcd_file_ground/");
        // saveToPCD(no_ground_pointCloud_, "/home/wang/bag/indoor_pcd_file_no_ground/");

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
                    temp_box_msg.pose.position.x = obj.centroid().x;
                    temp_box_msg.pose.position.y = obj.centroid().y;
                    temp_box_msg.pose.position.z = obj.centroid().z;
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
                    temp_box_msg.pose.position.x = obj.centroid().x;
                    temp_box_msg.pose.position.y = obj.centroid().y;
                    temp_box_msg.pose.position.z = obj.centroid().z;
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
        for (auto &obj : obj_vec_)
        {
            obj.is_watched = false;
        }
    }
}