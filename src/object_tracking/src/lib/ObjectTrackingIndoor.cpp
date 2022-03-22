#include "ObjectTrackingIndoor.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <array>

namespace tracking
{
    ObjectTrackingIndoor::ObjectTrackingIndoor(ros::NodeHandle &nh) : ObjectTracking(nh), no_background_pointCloud_(new PointCloud), background_pointCloud_(new PointCloud),static_pointCloud_(new PointCloud)
    {
    }

    ObjectTrackingIndoor::~ObjectTrackingIndoor()
    {
    }

    void ObjectTrackingIndoor::readParams(ros::NodeHandle& nh)
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
        nh.param<std::string>("/object_tracking_node/topic_name_pub_no_background_pointCloud", topic_name_pub_no_background_pointCloud_, "/object_tracking/no_background_pointCloud");
        nh.param<std::string>("/object_tracking_node/topic_name_pub_static_pointCloud", topic_name_pub_static_pointCloud_, "/object_tracking/static_pointCloud");
    }

    void ObjectTrackingIndoor::process()
    {
        readParams(nh_);
        sub_pointCloud_ = nh_.subscribe(topic_name_sub_pointCloud_, 5, &ObjectTrackingIndoor::msgHandleSimple, this);
        pub_ground_points_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name_pub_ground_pointCloud_, 5);
        pub_no_ground_points_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name_pub_no_ground_pointCloud_, 5);
        pub_no_background_points_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name_pub_no_background_pointCloud_, 5);
        pub_static_points_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name_pub_static_pointCloud_, 5);
        pub_bounding_box_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(topic_name_pub_bounding_box_, 5);
        pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_name_pub_marker_, 5);

        ros::spin();


    }


    void ObjectTrackingIndoor::pointCloudFilter(PointCloud::Ptr& in_pc_ptr, PointCloud::Ptr& filtered_pc_ptr)
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

        pass.setFilterFieldName("z");
        pass.setFilterLimits(-2.0, 1.8);
        pass.setFilterLimitsNegative(false);
        pass.filter(*temp_pc_ptr);

        // 体素滤波
        pcl::VoxelGrid<Point> voxel;
        voxel.setInputCloud(temp_pc_ptr);
        voxel.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel.filter(*filtered_pc_ptr);

    }

    /**
     * \brief 投影到2d做聚类
     */ 
    void ObjectTrackingIndoor::clusterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& no_ground_pc_ptr, std::vector<pcl::PointIndices>& local_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*no_ground_pc_ptr, *cloud_2d_ptr);
        for(auto& point:cloud_2d_ptr->points)
        {
            point.z = 0 ;
        }

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_2d_ptr);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extractor;
        cluster_extractor.setInputCloud(cloud_2d_ptr);
        cluster_extractor.setClusterTolerance(max_cluster_dist_);
        cluster_extractor.setMinClusterSize(min_cluster_size_);
        cluster_extractor.setMaxClusterSize(max_cluster_size_);
        cluster_extractor.setSearchMethod(tree);
        cluster_extractor.extract(local_indices);
    }

    /**
     * \brief 网格化后的聚类方式
     * @param range 探测x、y轴负半轴的范围
     */ 
    void ObjectTrackingIndoor::connectedComponentClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& no_ground_pc_ptr, std::vector<pcl::PointIndices>& local_indices, float range)
    {
        // 网格化点云 20m*20m 20cm*20cm为1个cell
        std::array<std::array<int, 100>, 100> grid_statistics_num;
        for(int index_x = 0; index_x < 100; index_x++)
        {
            for(int index_y = 0; index_y < 100; index_y++)
            {
                grid_statistics_num[index_x][index_y] = 0;
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*no_ground_pc_ptr, *temp_cloud_ptr);
        
        // 
        std::array<std::array<pcl::PointIndices,100>,100> grid_point_index;
        float cell_width = range / 50;     // range 10mcell_width 20cm
        for(int i=0; i< temp_cloud_ptr->points.size();i++)
        {
            temp_cloud_ptr->points[i].x += range;   // +10m
            temp_cloud_ptr->points[i].y += range;
            int index_x = floor(temp_cloud_ptr->points[i].x / cell_width);
            int index_y = floor(temp_cloud_ptr->points[i].y / cell_width);
            grid_point_index[index_x][index_y].indices.push_back(i);
            grid_statistics_num[index_x][index_y] += 1;
        }
        
        // 对每个网格的点云进行检验  不符合要求的将grid_statistics_num[i][j]清空
        // for(int index_x = 0; index_x < 100; ++index_x)
        // {
        //     for(int index_y = 0; index_y < 100; ++index_y)
        //     {

        //     }
        // }
        
        // for(int index_x = 0; index_x < 100; index_x++)
        // {
        //     for(int index_y = 0; index_y < 100; index_y++)
        //     {
        //         if(grid_statistics_num[index_x][index_y] > 0)
        //         {
        //             std::cerr<<index_x<<"  "<<index_y<<std::endl;
        //         }
        //     }
        // }

       
    

        // 对于有点的cell，扩大其邻域
        std::array<std::array<int,100>,100> grid_occupy;
        for(int index_x = 0; index_x < 100; index_x++)
        {
            for(int index_y = 0; index_y < 100; index_y++)
            {
                grid_occupy[index_x][index_y] = 0;
            }
        }

        if(grid_statistics_num[0][0] > 1)
        {
            grid_occupy[0][0] = -1;
            grid_occupy[0][1] = -1;
            grid_occupy[1][0] = -1;
            grid_occupy[1][1] = -1;
        }
        if(grid_statistics_num[99][99] > 1)
        {
            grid_occupy[99][99] = -1;
            grid_occupy[98][99] = -1;
            grid_occupy[99][98] = -1;
            grid_occupy[98][98] = -1;
        }
        if(grid_statistics_num[0][99] > 1)
        {
            grid_occupy[0][99] = -1;
            grid_occupy[0][98] = -1;
            grid_occupy[1][99] = -1;
            grid_occupy[1][98] = -1;
        }
        if(grid_statistics_num[99][0] > 1)
        {
            grid_occupy[99][0] = -1;
            grid_occupy[99][1] = -1;
            grid_occupy[98][0] = -1;
            grid_occupy[98][1] = -1;
        }
        for(int index = 1; index < 99; ++index)
        {
            if(grid_statistics_num[0][index] > 1)
            {
                grid_occupy[0][index] = -1;
                grid_occupy[0][index-1] = -1;
                grid_occupy[0][index+1] = -1;
                grid_occupy[1][index] = -1;
                grid_occupy[1][index-1] = -1;
                grid_occupy[1][index+1] = -1;
            }
            if(grid_statistics_num[index][0] > 1)
            {
                grid_occupy[index][0] = -1;
                grid_occupy[index-1][0] = -1;
                grid_occupy[index+1][0] = -1;
                grid_occupy[index][1] = -1;
                grid_occupy[index-1][1] = -1;
                grid_occupy[index+1][1] = -1;
            }
            if(grid_statistics_num[99][index] > 1)
            {
                grid_occupy[99][index] = -1;
                grid_occupy[99][index-1] = -1;
                grid_occupy[99][index+1] = -1;
                grid_occupy[98][index] = -1;
                grid_occupy[98][index-1] = -1;
                grid_occupy[98][index+1] = -1;
            }
            if(grid_statistics_num[index][99] > 1)
            {
                grid_occupy[index][99] = -1;
                grid_occupy[index-1][99] = -1;
                grid_occupy[index+1][99] = -1;
                grid_occupy[index][98] = -1;
                grid_occupy[index-1][98] = -1;
                grid_occupy[index+1][98] = -1;

            }
        }
        for(int index_x = 1; index_x < 99; index_x++)
        {
            for(int index_y = 1; index_y < 99; index_y++)
            {
                if(grid_statistics_num[index_x][index_y] > 1)
                {
                    grid_occupy[index_x][index_y] = -1;
                    grid_occupy[index_x-1][index_y] = -1;
                    grid_occupy[index_x+1][index_y] = -1;
                    grid_occupy[index_x][index_y-1] = -1;
                    grid_occupy[index_x][index_y+1] = -1;
                    grid_occupy[index_x-1][index_y-1] = -1;
                    grid_occupy[index_x-1][index_y+1] = -1;
                    grid_occupy[index_x+1][index_y-1] = -1;
                    grid_occupy[index_x+1][index_y+1] = -1;
                }
            }
        }

        // 根据占据图的连同性来聚类   
        int cluster_id = 1;
        for(int index_x = 0; index_x < 100; index_x++)
        {
            for(int index_y = 0; index_y < 100; index_y++)
            {
                if(grid_occupy[index_x][index_y] == -1)
                {
                    // 找周围占据情况为-1的点，纳入到自己的类别中
                    searchCell(grid_occupy, index_x, index_y, cluster_id);
                    ++cluster_id;    
                }
            }
        }  

        // 把同类的点索引放入 local_indices, 1~cluster_id -1 个类
        local_indices.resize(cluster_id);
        for(int index_x = 0; index_x < 100; index_x++)
        {
            for(int index_y = 0; index_y <100; index_y++)
            {
                if(grid_occupy[index_x][index_y] > 0)
                {
                    for(auto& ind : grid_point_index[index_x][index_y].indices)
                    {
                        local_indices[grid_occupy[index_x][index_y]].indices.push_back(ind);
                    }
                }
            }
        } 
    }

    /**
     * \brief 递归地搜索同类的cell
     */ 
    void ObjectTrackingIndoor::searchCell(std::array<std::array<int,100>,100>& grid_occupy, int index_x, int index_y, int cluster_id)
    {   
        cellType cell_type;
        if(index_x == 0 || index_y == 0 || index_x == 99 || index_y == 99)
        {
            if(index_x == 0)
            {
                switch(index_y)
                {
                    case 0:
                    cell_type = LEFT_TOP_CELL;
                    break;

                    case 99:
                    cell_type = RIGHT_TOP_CELL;
                    break;

                    default:
                    cell_type = TOP_EDGE_CELL; 
                    break;
                }
            }
            else if(index_x == 99)
            {
                switch(index_y)
                {
                    case 0:
                    cell_type = LEFT_BOTTOM_CELL;
                    break;

                    case 99:
                    cell_type = RIGHT_BOTTOM_CELL;
                    break;

                    default:
                    cell_type = BOTTOM_EDGE_CELL;
                }
            }

            else if(index_y == 0)
            {
                switch(index_x)
                {
                    case 0:
                    cell_type = LEFT_TOP_CELL;
                    break;

                    case 99:
                    cell_type = LEFT_BOTTOM_CELL;
                    break;

                    default:
                    cell_type = LEFT_EDGE_CELL;
                    break;
                }
            }

            else
            {
                switch(index_x)
                {
                    case 0:
                    cell_type = RIGHT_TOP_CELL;
                    break;

                    case 99:
                    cell_type = RIGHT_BOTTOM_CELL;
                    break;

                    default:
                    cell_type = RIGHT_EDGE_CELL;
                    break;
                }
            }
        }
        else
        {
            cell_type = NORNAL_CELL;
        }

        switch(cell_type)
        {
            case LEFT_TOP_CELL:
            grid_occupy[index_x][index_y] = cluster_id;
            if(grid_occupy[index_x+1][index_y] == -1){searchCell(grid_occupy,index_x+1, index_y, cluster_id);}
            if(grid_occupy[index_x][index_y+1] == -1){searchCell(grid_occupy,index_x, index_y+1, cluster_id);}
            if(grid_occupy[index_x+1][index_y+1] == -1){searchCell(grid_occupy,index_x+1, index_y+1, cluster_id);}    
            break;

            case RIGHT_TOP_CELL:
            grid_occupy[index_x][index_y] = cluster_id;
            if(grid_occupy[index_x][index_y-1] == -1){searchCell(grid_occupy,index_x, index_y-1, cluster_id);}
            if(grid_occupy[index_x+1][index_y] == -1){searchCell(grid_occupy,index_x+1, index_y, cluster_id);}
            if(grid_occupy[index_x+1][index_y-1] == -1){searchCell(grid_occupy,index_x+1, index_y-1, cluster_id);}
            break;

            case LEFT_BOTTOM_CELL:
            grid_occupy[index_x][index_y] = cluster_id;
            if(grid_occupy[index_x][index_y+1] == -1){searchCell(grid_occupy,index_x, index_y+1, cluster_id);}
            if(grid_occupy[index_x-1][index_y] == -1){searchCell(grid_occupy,index_x-1, index_y, cluster_id);}
            if(grid_occupy[index_x-1][index_y+1] == -1){searchCell(grid_occupy,index_x-1, index_y+1, cluster_id);}
            break;

            case RIGHT_BOTTOM_CELL:
            grid_occupy[index_x][index_y] = cluster_id;
            if(grid_occupy[index_x-1][index_y] == -1){searchCell(grid_occupy,index_x-1, index_y, cluster_id);}
            if(grid_occupy[index_x][index_y-1] == -1){searchCell(grid_occupy,index_x, index_y-1, cluster_id);}
            if(grid_occupy[index_x-1][index_y-1] == -1){searchCell(grid_occupy,index_x-1, index_y-1, cluster_id);}
            break;

            case TOP_EDGE_CELL:
            grid_occupy[index_x][index_y] = cluster_id;
            if(grid_occupy[index_x][index_y-1] == -1){searchCell(grid_occupy,index_x, index_y-1, cluster_id);}
            if(grid_occupy[index_x][index_y+1] == -1){searchCell(grid_occupy,index_x, index_y+1, cluster_id);}
            if(grid_occupy[index_x+1][index_y-1] == -1){searchCell(grid_occupy,index_x+1, index_y-1, cluster_id);}
            if(grid_occupy[index_x+1][index_y] == -1){searchCell(grid_occupy,index_x+1, index_y, cluster_id);}
            if(grid_occupy[index_x+1][index_y+1] == -1){searchCell(grid_occupy,index_x+1, index_y+1, cluster_id);}    
            break;

            case BOTTOM_EDGE_CELL:
            grid_occupy[index_x][index_y] = cluster_id;
            if(grid_occupy[index_x][index_y-1] == -1){searchCell(grid_occupy,index_x, index_y-1, cluster_id);}
            if(grid_occupy[index_x][index_y+1] == -1){searchCell(grid_occupy,index_x, index_y+1, cluster_id);}
            if(grid_occupy[index_x-1][index_y-1] == -1){searchCell(grid_occupy,index_x-1, index_y-1, cluster_id);}
            if(grid_occupy[index_x-1][index_y] == -1){searchCell(grid_occupy,index_x-1, index_y, cluster_id);}
            if(grid_occupy[index_x-1][index_y+1] == -1){searchCell(grid_occupy,index_x-1, index_y+1, cluster_id);} 
            break;

            case LEFT_EDGE_CELL:
            grid_occupy[index_x][index_y] = cluster_id;
            if(grid_occupy[index_x-1][index_y] == -1){searchCell(grid_occupy,index_x-1, index_y, cluster_id);}
            if(grid_occupy[index_x+1][index_y] == -1){searchCell(grid_occupy,index_x+1, index_y, cluster_id);}
            if(grid_occupy[index_x-1][index_y+1] == -1){searchCell(grid_occupy,index_x-1, index_y+1, cluster_id);}
            if(grid_occupy[index_x][index_y+1] == -1){searchCell(grid_occupy,index_x, index_y+1, cluster_id);}
            if(grid_occupy[index_x+1][index_y+1] == -1){searchCell(grid_occupy,index_x+1, index_y+1, cluster_id);} 
            break;

            case RIGHT_EDGE_CELL:
            grid_occupy[index_x][index_y] = cluster_id;
            if(grid_occupy[index_x-1][index_y] == -1){searchCell(grid_occupy,index_x-1, index_y, cluster_id);}
            if(grid_occupy[index_x+1][index_y] == -1){searchCell(grid_occupy,index_x+1, index_y, cluster_id);}
            if(grid_occupy[index_x-1][index_y-1] == -1){searchCell(grid_occupy,index_x-1, index_y-1, cluster_id);}
            if(grid_occupy[index_x][index_y-1] == -1){searchCell(grid_occupy,index_x, index_y-1, cluster_id);}
            if(grid_occupy[index_x+1][index_y-1] == -1){searchCell(grid_occupy,index_x+1, index_y-1, cluster_id);}
            break;

            case NORNAL_CELL:
            grid_occupy[index_x][index_y] = cluster_id;
            if(grid_occupy[index_x-1][index_y] == -1){searchCell(grid_occupy,index_x-1, index_y, cluster_id);}
            if(grid_occupy[index_x+1][index_y] == -1){searchCell(grid_occupy,index_x+1, index_y, cluster_id);}
            if(grid_occupy[index_x-1][index_y-1] == -1){searchCell(grid_occupy,index_x-1, index_y-1, cluster_id);}
            if(grid_occupy[index_x][index_y-1] == -1){searchCell(grid_occupy,index_x, index_y-1, cluster_id);}
            if(grid_occupy[index_x+1][index_y-1] == -1){searchCell(grid_occupy,index_x+1, index_y-1, cluster_id);}
            if(grid_occupy[index_x-1][index_y+1] == -1){searchCell(grid_occupy,index_x-1, index_y+1, cluster_id);}
            if(grid_occupy[index_x][index_y+1] == -1){searchCell(grid_occupy,index_x, index_y+1, cluster_id);}
            if(grid_occupy[index_x+1][index_y+1] == -1){searchCell(grid_occupy,index_x+1, index_y+1, cluster_id);}
            break;

            default:
            printf("enum cell_type error.");
            break;

        }

    }

    

    /**
     * \brief 去除两边的墙面内点
     * 
     */
    void ObjectTrackingIndoor::removeBackgroundPoint(PointCloud::Ptr &no_ground_pc_ptr)
    {
        // XYZI - XYZ
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr copy_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*no_ground_pc_ptr, *in_pc_ptr);
        pcl::copyPointCloud(*in_pc_ptr, *copy_pc_ptr);


        // 这里可以再进行一次下采样，让计算法向量少一些

        // 计算法向量
        pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        normal_estimation.setInputCloud(in_pc_ptr);
        normal_estimation.setNumberOfThreads(10);
        normal_estimation.setSearchMethod(tree);
        normal_estimation.setKSearch(10);
        normal_estimation.compute(*normal_ptr);

        // 找法向量水平的点 前200个
        std::vector<int> index(normal_ptr->points.size());
        for (int i = 0; i < normal_ptr->points.size(); i++)
        {
            index[i] = i;
        }

        sort(index.begin(), index.end(), [normal_ptr](const int &index1, const int &index2)
             { return abs(normal_ptr->points[index1].normal_z) < abs(normal_ptr->points[index2].normal_z); });

        pcl::PointCloud<pcl::PointXYZ>::Ptr fitting_plane_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        int num_selected = 200;
        for (size_t i = 0; i < num_selected; i++)
        {
            fitting_plane_pc_ptr->points.push_back(in_pc_ptr->points[index[i]]);
        }

        // 拟合2个墙面
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::SACSegmentation<pcl::PointXYZ> seg;

        int num_plane = 2;
        std::vector<pcl::ModelCoefficients> coeff_vector;
        for (size_t i = 0; i< num_plane; i++)
        {
            pcl::PointIndices::Ptr temp_indices_ptr(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr temp_coeff_ptr(new pcl::ModelCoefficients);
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.1);
            seg.setInputCloud(in_pc_ptr);
            seg.segment(*temp_indices_ptr, *temp_coeff_ptr); 
            coeff_vector.push_back(*temp_coeff_ptr);        

            extract.setInputCloud(in_pc_ptr);
            extract.setIndices(temp_indices_ptr);
            extract.setNegative(true);
            extract.filter(*in_pc_ptr);
        }

        // 计算每个点和墙面的距离，距离过大则删除  
        pcl::PointIndices::Ptr plane_indice_ptr(new pcl::PointIndices);
        float threshold = 0.2;
        for(auto& coeff : coeff_vector)
        {
            for(size_t i=0; i< copy_pc_ptr->points.size();++i)
            {
                float dis = pcl::pointToPlaneDistance(copy_pc_ptr->points[i], coeff.values[0], coeff.values[1], coeff.values[2], coeff.values[3]);
                if(dis < threshold)
                {
                    plane_indice_ptr->indices.push_back(i);
                }
            }
        }
        
        pcl::ExtractIndices<Point> extract2;
        extract2.setInputCloud(no_ground_pointCloud_);
        extract2.setIndices(plane_indice_ptr);
        extract2.setNegative(true);
        extract2.filter(*no_background_pointCloud_);

        extract2.setNegative(false);
        extract2.filter(*background_pointCloud_);
    }

    /**
     * \brief 对聚类的物体点云进行曲率检验
     * 
     */ 
    bool ObjectTrackingIndoor::checkObject(pcl::PointCloud<pcl::PointXYZ>::Ptr& checked_obj_ptr)
    {
        Eigen::Vector4f mean;
        Eigen::Matrix3f cov;
        pcl::computeMeanAndCovarianceMatrix(*checked_obj_ptr, cov, mean);
        Eigen::JacobiSVD<Eigen::Matrix3f> SVD(cov, Eigen::DecompositionOptions::ComputeFullU);
        Eigen::Vector3f value_vec = SVD.singularValues();
        if((value_vec[2] / (value_vec[0] + value_vec[1] + value_vec[2])) > 0.015)
        {
            return true;
        }
        else
        {
            return false;
        }  
    }


    /**
     * \brief 与ObjectTracking相比增加了检验环节
     */ 
    void ObjectTrackingIndoor::segmentObject(PointCloud::Ptr& no_ground_pc_ptr, std::vector<pcl::PointIndices>& indices_vector)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*no_ground_pc_ptr, *in_pc_ptr);

        // 聚类分割
        std::vector<pcl::PointIndices> local_indices;
        clusterPointCloud(in_pc_ptr, local_indices);
        // connectedComponentClustering(in_pc_ptr, local_indices, 10);

        if (is_initial_status)
        {
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

                // 检验环节
                /////////////////////////////////////////////////////////////////////////////
                // 形状检验
                // 室内条件
                if (length > 1.2 || length < 0.2 || width > 1.2 || width < 0.2 || height > 2.2)
                {
                    continue;
                }
                
                // 室外条件
                // if(length > 3.0 || length < 0.2 || width > 3.0 || width < 0.2 || height > 2.5)
                // {
                //     continue;
                // }

                // 表面曲率检验
                pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*in_pc_ptr, local_indices[i], *obj_cloud_ptr);
                if(!checkObject(obj_cloud_ptr))
                {
                    continue;
                }
                /////////////////////////////////////////////////////////////////////////////



                DetectedObject obj(length, width, height, centroid, shared_time_, obj_label_,5);
                indices_vector.push_back(local_indices[i]);
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

                // 检验环节
                //////////////////////////////////////////////////////////////////////////////////////
                // 形状检验
                // 室内条件
                if (length > 1.2 || length < 0.2 || width > 1.2 || width < 0.2 || height > 2.2)
                {
                    continue;
                }

                // 室外条件
                // if(length > 3.0 || length < 0.2 || width > 3.0 || width < 0.2 || height > 2.5)
                // {
                //     continue;
                // }
                // 表面曲率检验
                pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*in_pc_ptr, local_indices[i], *obj_cloud_ptr);
                if(!checkObject(obj_cloud_ptr))
                {
                    continue;
                }
                //////////////////////////////////////////////////////////////////////////////////////

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
                    DetectedObject obj(length, width, height, centroid, shared_time_, obj_label_,5);
                    ++obj_label_;
                    obj_vec_.push_back(obj);
                }
                indices_vector.push_back(local_indices[i]);
            }
        }
    }


    /**
     * \brief 移除动态物体，将静态点云存入static_pointCloud_
     * @param in_pc_ptr 原始点云
     */ 
    void ObjectTrackingIndoor::removeDynamicObject(PointCloud::Ptr& in_pc_ptr, std::vector<pcl::PointIndices>& indices_vector)
    {
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        pcl::PointIndices::Ptr indice_ptr(new pcl::PointIndices);
        for(auto& indices : indices_vector)
        {
            for(auto& ind : indices.indices)
            {
                indice_ptr->indices.push_back(ind);
            }
        }
        extract.setInputCloud(in_pc_ptr);
        extract.setIndices(indice_ptr);
        extract.setNegative(true);
        extract.filter(*static_pointCloud_);
    }

    /**
     * \brief 回调函数
     */
    void ObjectTrackingIndoor::msgHandle(const sensor_msgs::PointCloud2ConstPtr &raw_pointCloud)
    {
        PointCloud::Ptr in_pc_ptr(new PointCloud);
        PointCloud::Ptr filtered_pc_ptr(new PointCloud);
        shared_header_ = raw_pointCloud->header;
        shared_time_ = shared_header_.stamp.toNSec();

        pcl::fromROSMsg(*raw_pointCloud, *in_pc_ptr);

       
        pointCloudFilter(in_pc_ptr, filtered_pc_ptr);
        clearPointCloud();
        static_pointCloud_->points.clear();
        removeGroundPoint(filtered_pc_ptr);
        removeBackgroundPoint(no_ground_pointCloud_);

        std::vector<pcl::PointIndices> indices_vector;
        segmentObject(no_background_pointCloud_, indices_vector);  // 索引不配套
        // segmentObject(no_ground_pointCloud_);
        removeDynamicObject(no_background_pointCloud_, indices_vector);

        *static_pointCloud_ += *background_pointCloud_;

        // 发布环节
        sensor_msgs::PointCloud2 no_ground_pc_msg;
        sensor_msgs::PointCloud2 no_background_pc_msg;
        sensor_msgs::PointCloud2 static_pc_msg;

        pcl::toROSMsg(*no_ground_pointCloud_, no_ground_pc_msg);
        pcl::toROSMsg(*no_background_pointCloud_, no_background_pc_msg);
        pcl::toROSMsg(*static_pointCloud_, static_pc_msg);

        no_background_pc_msg.header = shared_header_;
        no_ground_pc_msg.header = shared_header_;
        static_pc_msg.header = shared_header_;
        bounding_boxes_msg_ptr_->header = shared_header_;

        if (is_initial_status && obj_vec_.size() > 0)
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

        if(is_initial_status && obj_vec_.size() == 0)
        {
            // 第一帧没东西，跳过, 仍然是开始标志位
            return;

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

        pub_no_background_points_.publish(no_background_pc_msg);
        pub_no_ground_points_.publish(no_ground_pc_msg);
        pub_static_points_.publish(static_pc_msg);
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


    /** \brief 简化版回调，没有去除背景点云的操作
     * 
     */ 
    void ObjectTrackingIndoor::msgHandleSimple(const sensor_msgs::PointCloud2ConstPtr& raw_pointCloud)
    {
        PointCloud::Ptr in_pc_ptr(new PointCloud);
        PointCloud::Ptr filtered_pc_ptr(new PointCloud);
        shared_header_ = raw_pointCloud->header;
        shared_time_ = shared_header_.stamp.toNSec();

        pcl::fromROSMsg(*raw_pointCloud, *in_pc_ptr);

       
        pointCloudFilter(in_pc_ptr, filtered_pc_ptr);
        clearPointCloud();
        static_pointCloud_->points.clear();
        removeGroundPoint(filtered_pc_ptr);

        std::vector<pcl::PointIndices> indices_vector;
        segmentObject(no_ground_pointCloud_, indices_vector);  
        removeDynamicObject(no_ground_pointCloud_, indices_vector);

        *static_pointCloud_ += *ground_pointCloud_;

        // 发布环节
        sensor_msgs::PointCloud2 no_ground_pc_msg;
        sensor_msgs::PointCloud2 static_pc_msg;

        pcl::toROSMsg(*no_ground_pointCloud_, no_ground_pc_msg);
        pcl::toROSMsg(*static_pointCloud_, static_pc_msg);

        no_ground_pc_msg.header = shared_header_;
        static_pc_msg.header = shared_header_;
        bounding_boxes_msg_ptr_->header = shared_header_;

        if (is_initial_status && obj_vec_.size() > 0)
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

        if(is_initial_status && obj_vec_.size() == 0)
        {
            // 第一帧没东西，跳过, 仍然是开始标志位
            return;

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
        pub_static_points_.publish(static_pc_msg);
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