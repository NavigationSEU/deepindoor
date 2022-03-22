#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include "CircularBuffer.h"

namespace tracking
{
    class DetectedObject
    {
    public:
        DetectedObject(float length, float width, float height, pcl::PointXYZ centroid, long int Ntime, int label, int history_length = 20);
        virtual ~DetectedObject();

        inline void setLength(float length) { length_ = length; }
        inline void setWidth(float width) { width_ = width; }
        inline void setHeight(float height) { height_ = height; }
        inline void setCentroid(pcl::PointXYZ centroid) { centroid_ = centroid; }
        inline void setCurrentFrameCentroid(pcl::PointXYZ centroid){current_frame_centroid_ = centroid;}
        inline void setLabel(int label) { label_ = label; }
        inline void setNtime(long int Ntime) { Ntime_ = Ntime; }

        inline float length() { return length_; }
        inline float width() { return width_; }
        inline float height() { return height_; }
        inline int label() { return label_; }
        inline long int Ntime() { return Ntime_; }
        inline pcl::PointXYZ &centroid() { return centroid_; }
        inline pcl::PointXYZ &currentFrameCentroid(){return current_frame_centroid_;}
        inline float velocity(){return velocity_;}
        inline float acc(){return acc_;}
        inline bool isStartCalState()
        {
            if(history_buffer_.size() == history_buffer_.capacity())
            return true;
        }


        void note();
        void calState();
        void kalmanEstimateState(bool estimate_vel_acc = false);

        friend std::ostream &operator<<(std::ostream &out, DetectedObject &object);

    public:
        bool is_watched = true;   // 此刻是否被观测到
        bool is_Stop = true;      // 该物体是否为静止
        bool is_tracking = false; // 该物体是否被跟踪

    protected:
        struct History
        {
            long int Ntime_;
            float length_;
            float width_;
            float height_;
            float velocity_ = 0;
            float acc_ = 0;
            pcl::PointXYZ centroid_;
            float operator-(const DetectedObject::History &object) const;
        };

    protected:
        long int Ntime_; // 以纳秒为单位的时间
        int label_;      // 物体的编号
        pcl::PointXYZ min_point_;
        pcl::PointXYZ max_point_;
        pcl::PointXYZ centroid_;
        pcl::PointXYZ current_frame_centroid_;
        float length_;
        float width_;
        float height_;
        float velocity_ = 0;
        float acc_ = 0;

        CircularBuffer<History> history_buffer_;
        
    };

    std::ostream &operator<<(std::ostream &out, DetectedObject &object);
    

}