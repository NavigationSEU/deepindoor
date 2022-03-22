#include "BasicObjectTracking.h"
#include "DetectedObject.h"
#include "object_tracking/DetectedObjectMsg.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <limits>

#include <pcl/visualization/cloud_viewer.h>
namespace tracking
{

    BasicObjectTracking::BasicObjectTracking(ros::NodeHandle &nh):nh_(nh),ground_pointCloud_(new PointCloud),no_ground_pointCloud_(new PointCloud)
    {

    }

    BasicObjectTracking::~BasicObjectTracking()
    {
    }

    void BasicObjectTracking::process()
    {
        readParams(nh_);
        sub_pointCloud_ = nh_.subscribe(topic_name_sub_pointCloud_, 5, &BasicObjectTracking::msgHandle, this);
        pub_ground_points_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name_pub_ground_pointCloud_, 5);
        pub_no_ground_points_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name_pub_no_ground_pointCloud_, 5);
        pub_object_ = nh_.advertise<object_tracking::DetectedObjectMsg>(topic_name_pub_object_, 3);
        pub_bounding_box_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(topic_name_pub_bounding_box_, 5);
        pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_name_pub_marker_, 5);

        ros::spin();
    }


    void BasicObjectTracking::readParams(ros::NodeHandle &nh)
    {
        nh.param<int>("/object_tracking_node/num_iter", num_iter_, 3);
        nh.param<int>("/object_tracking_node/num_LPR", num_LPR_, 100);
        nh.param<double>("/object_tracking_node/thres_seeds", thres_seeds_, 0.8);
        nh.param<double>("/object_tracking_node/thres_dist", thres_dist_, 0.15);

        nh.param<double>("/object_tracking_node/max_cluster_dist", max_cluster_dist_, 0);
        nh.param<int>("/object_tracking_node/max_cluster_size", max_cluster_size_, 0);
        nh.param<int>("/object_tracking_node/min_cluster_size", min_cluster_size_, 0);

        nh.param<std::string>("/object_tracking_node/topic_name_sub_pointCloud", topic_name_sub_pointCloud_, "");
        nh.param<std::string>("/object_tracking_node/topic_name_sub_odometry", topic_name_sub_odometry_, "");
        nh.param<std::string>("/object_tracking_node/topic_name_pub_ground_pointCloud", topic_name_pub_ground_pointCloud_, "/object_tracking/ground_pointCloud");
        nh.param<std::string>("/object_tracking_node/topic_name_pub_no_ground_pointCloud", topic_name_pub_no_ground_pointCloud_, "/object_tracking/no_ground_pointCloud");
        nh.param<std::string>("/object_tracking_node/topic_name_pub_object", topic_name_pub_object_, "/object_tracking/object");
        nh.param<std::string>("/object_tracking_node/topic_name_pub_bounding_box", topic_name_pub_bounding_box_, "/object_tracking/bounding_box");
        nh.param<std::string>("/object_tracking_node/topic_name_pub_marker", topic_name_pub_marker_, "/object_tracking/marker");
    }

    /** \brief 对原始点云进行三重滤波 
     *  1. 去Nan
     *  2. 直通滤波 范围前后左右20m
     *  3. 体素滤波
     * @param 
     * @return 
     */
    void BasicObjectTracking::pointCloudFilter(PointCloud::Ptr &in_pc_ptr, PointCloud::Ptr &filtered_pc_ptr)
    {
        // 去Nan
        std::vector<int> index;
        PointCloud::Ptr temp_pc_ptr(new PointCloud);
        pcl::removeNaNFromPointCloud(*in_pc_ptr, *temp_pc_ptr, index);

        // 直通滤波 原来是-20 - 20
        pcl::PassThrough<Point> pass;
        pass.setInputCloud(temp_pc_ptr);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-10.0, 10.0);
        pass.filter(*temp_pc_ptr);

        pass.setFilterFieldName("x");
        pass.setFilterLimits(-0.1, 0.1);
        pass.setFilterLimitsNegative(true);
        pass.filter(*temp_pc_ptr);

        pass.setFilterFieldName("y");
        pass.setFilterLimitsNegative(false);
        pass.setFilterLimits(-10.0, 10.0);
        pass.filter(*temp_pc_ptr);

        pass.setFilterFieldName("y");
        pass.setFilterLimits(-0.1, 0.1);
        pass.setFilterLimitsNegative(true);
        pass.filter(*temp_pc_ptr);

        // 体素滤波
        pcl::VoxelGrid<Point> voxel;
        voxel.setInputCloud(temp_pc_ptr);
        voxel.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel.filter(*filtered_pc_ptr);

        // pcl::visualization::CloudViewer view("cloud");
        // view.showCloud(temp_pc_ptr);
        // getchar();
    }

    /** \brief 挑选最低的若干点作为初始地面点 
      * 1. 排序找最低的若干点
      * 2. 在这些点均值附近的点认为是地面点
      * @param 
      * @return 
      */
    void BasicObjectTracking::extractInitialSeeds(PointCloud::Ptr &in_pc_ptr)
    {
        // 根据高度从小到大排序
        sort(in_pc_ptr->begin(), in_pc_ptr->end(), [](const Point &p1, const Point &p2)
             { return p1.z < p2.z; });

        // 计算最小的num_LPR_个点平均值
        double height_LPR = 0;
        for (size_t i = 0; i < num_LPR_; i++)
        {
            height_LPR += in_pc_ptr->points[i].z;
        }
        height_LPR /= num_LPR_;

        // 在height_LPR阈值内的点认为是地面点
        for (size_t i = 0; i < in_pc_ptr->points.size(); i++)
        {
            if (in_pc_ptr->points[i].z < height_LPR + thres_seeds_)
            {
                ground_pointCloud_->push_back(in_pc_ptr->points[i]);
            }
        }
    }

    /** \brief 通过地面点的特征向量估计地面的法线
     * 1. n.Tx+d = 0
     * @param 
     * @return 
     */
    void BasicObjectTracking::estimatePlane()
    {
        Eigen::Vector4f mean;
        Eigen::Matrix3f cov;
        pcl::computeMeanAndCovarianceMatrix(*ground_pointCloud_, cov, mean);
        Eigen::JacobiSVD<Eigen::Matrix3f> SVD(cov, Eigen::DecompositionOptions::ComputeFullU);
        ground_norm_vector_ = SVD.matrixU().col(2);
        auto mean_point = mean.head<3>();
        ground_d_ = -ground_norm_vector_.transpose() * mean_point;
    }

    /** \brief 对滤波后的点云进行地面点去除 
     *  1. 最初的地面点，一般取最低的一群点作为地面点
     *  2. 迭代根据地面点拟合地面
     *  3. 迭代根据拟合的平面判断点是否在平面上
     * @param 
     * @return 
     */
    void BasicObjectTracking::removeGroundPoint(PointCloud::Ptr &in_pc_ptr)
    {
        // 最初的地面点
        extractInitialSeeds(in_pc_ptr);

        for (size_t i = 0; i < num_iter_; i++)
        {
            // 根据地面点拟合平面
            estimatePlane();

            // 重新选择地面点
            ground_pointCloud_->clear();
            no_ground_pointCloud_->clear();

            // 通过拟合的平面方程判断点是否在平面上
            Eigen::Vector3f pt_vec;
            for (auto pt : in_pc_ptr->points)
            {
                pt_vec.x() = pt.x;
                pt_vec.y() = pt.y;
                pt_vec.z() = pt.z;

                if (ground_norm_vector_.transpose() * pt_vec + ground_d_ > thres_dist_)
                    no_ground_pointCloud_->push_back(pt);
                else
                    ground_pointCloud_->push_back(pt);
            }
        }
    }

    void BasicObjectTracking::clusterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& no_ground_pc_ptr, std::vector<pcl::PointIndices>& local_indices)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(no_ground_pc_ptr);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extractor;
        cluster_extractor.setInputCloud(no_ground_pc_ptr);
        cluster_extractor.setClusterTolerance(max_cluster_dist_);
        cluster_extractor.setMinClusterSize(min_cluster_size_);
        cluster_extractor.setMaxClusterSize(max_cluster_size_);
        cluster_extractor.setSearchMethod(tree);
        cluster_extractor.extract(local_indices);
    }

    void BasicObjectTracking::segmentObject(PointCloud::Ptr& no_ground_pc_ptr)
    {
               // XYZI -> XYZ
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*no_ground_pc_ptr, *in_pc_ptr);

        // 聚类分割
        std::vector<pcl::PointIndices> local_indices;
        clusterPointCloud(in_pc_ptr, local_indices);
        
        std::vector<pcl::PointXYZ> centroid_vec;
        std::vector<float> length_vec;
        std::vector<float> width_vec;
        std::vector<float> height_vec;

        // 统计每个聚类属性
        for(size_t i=0; i<local_indices.size();i++)
        {
            // 把obj的属性打点好，然后实例化DetectedObject
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
            if(length > 2.2 || width > 2.2 || height > 2.2){continue;}
            
            centroid_vec.push_back(centroid);
            length_vec.push_back(length);
            width_vec.push_back(width);
            height_vec.push_back(height);
        }

        jsk_recognition_msgs::BoundingBoxArray::Ptr bounding_box_msg(new jsk_recognition_msgs::BoundingBoxArray);
        visualization_msgs::MarkerArray::Ptr marker_msg(new visualization_msgs::MarkerArray);
        object_tracking::DetectedObjectMsg::Ptr obj_msg(new object_tracking::DetectedObjectMsg);
        bounding_box_msg->header = shared_header_;
        obj_msg->boxes.header = shared_header_;

        for(size_t i = 0;i<length_vec.size();i++)
        {
            jsk_recognition_msgs::BoundingBox temp_box_msg;
            temp_box_msg.header = shared_header_;
            temp_box_msg.pose.position.x = centroid_vec[i].x;
            temp_box_msg.pose.position.y = centroid_vec[i].y;
            temp_box_msg.pose.position.z = centroid_vec[i].z;
            temp_box_msg.dimensions.x = length_vec[i];
            temp_box_msg.dimensions.y = width_vec[i];
            temp_box_msg.dimensions.z = height_vec[i];
            bounding_box_msg->boxes.push_back(temp_box_msg);
        }
        // for(auto &obj : obj_vec_)
        // {        
        //     jsk_recognition_msgs::BoundingBox temp_box_msg;
        //     temp_box_msg.header = shared_header_;
        //     temp_box_msg.pose.position.x = obj.centroid().x;
        //     temp_box_msg.pose.position.y = obj.centroid().y;
        //     temp_box_msg.pose.position.z = obj.centroid().z;
        //     temp_box_msg.dimensions.x = obj.length();
        //     temp_box_msg.dimensions.y = obj.width();
        //     temp_box_msg.dimensions.z = obj.height();
        //     obj_msg->boxes.boxes.push_back(temp_box_msg);
        //     bounding_box_msg->boxes.push_back(temp_box_msg);

        //     visualization_msgs::Marker temp_marker;
        //     temp_marker.header = shared_header_;
        //     temp_marker.action = visualization_msgs::Marker::ADD;
        //     temp_marker.pose.orientation.w = 1.0;
        //     temp_marker.id = obj_msg->markers.markers.size();
        //     temp_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        //     temp_marker.scale.z = 0.5;
        //     temp_marker.color.r = 255;
        //     temp_marker.color.a = 1.0;
        //     temp_marker.pose.position.x = temp_box_msg.pose.position.x;
        //     temp_marker.pose.position.y = temp_box_msg.pose.position.y;
        //     temp_marker.pose.position.z = temp_box_msg.pose.position.z + 0.4;
        //     temp_marker.text = std::to_string(temp_box_msg.pose.position.x)+"\n"
        //     +std::to_string(temp_box_msg.pose.position.y)+"\n"
        //     +std::to_string(temp_box_msg.pose.position.z);
        //     obj_msg->markers.markers.push_back(temp_marker);
        //     marker_msg->markers.push_back(temp_marker);
        // }    

        // 发布
        pub_bounding_box_.publish(bounding_box_msg);
        // pub_marker_.publish(marker_msg);
        // pub_object_.publish(obj_msg);

    }


    void BasicObjectTracking::msgHandle(const sensor_msgs::PointCloud2ConstPtr &raw_pointCloud)
    {
        ROS_INFO("Received msgs.");


        PointCloud::Ptr in_pc_ptr(new PointCloud);
        PointCloud::Ptr filtered_pc_ptr(new PointCloud);
        shared_header_ = raw_pointCloud->header;

        pcl::fromROSMsg(*raw_pointCloud, *in_pc_ptr);
        pointCloudFilter(in_pc_ptr, filtered_pc_ptr);
        clearPointCloud();
        removeGroundPoint(filtered_pc_ptr);
        segmentObject(no_ground_pointCloud_);
        

    }
}