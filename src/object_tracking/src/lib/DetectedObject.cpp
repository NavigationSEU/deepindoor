#include "DetectedObject.h"

namespace tracking
{
    DetectedObject::DetectedObject(float length, float width, float height, pcl::PointXYZ centroid, long int Ntime, int label, int history_length)
    :length_(length), width_(width), height_(height), centroid_(centroid), Ntime_(Ntime), label_(label), history_buffer_(history_length)
    {

    }

    DetectedObject::~DetectedObject()
    {

    }

    void DetectedObject::note()
    {
        History current_history;
        current_history.Ntime_ = this->Ntime_;
        current_history.length_ = this->length_;
        current_history.width_ = this->width_;
        current_history.height_ = this->height_;
        current_history.centroid_ = this->centroid_;
        history_buffer_.push(current_history);

    }

    // 计算速度和加速度
    void DetectedObject::calState()
    {
        long int time1 = history_buffer_.first().Ntime_;
        long int time2 = history_buffer_.half().Ntime_;
        long int time3 = history_buffer_.last().Ntime_;
        float sec1 = (time2 - time1) / pow(10,9);
        float sec2 = (time3 - time2) / pow(10,9);
        float displacement1 = history_buffer_.half() - history_buffer_.first();
        float displacement2 = history_buffer_.last() - history_buffer_.half();
        float velocity1 = displacement1 / sec1;
        float velocity2 = displacement2 / sec2;
        float acc = (velocity2 - velocity1) / sec1;
        velocity_ = velocity2;
        acc_ = acc;
    }

    // 对物体的位置和长宽高进行估计
    void DetectedObject::kalmanEstimateState(bool estimate_vel_acc)
    {
        // 长宽高总是单独估计
        float q_length = 0.05;
        float q_width = 0.05;
        float q_height = 0.05;
        float r_lenght = 0.2;
        float r_width = 0.2;
        float r_height = 0.2;

        float length_estimated = length_*q_length/(q_length + r_height) + history_buffer_.last().length_*r_lenght/(q_length + r_lenght);
        float width_estimated = width_*q_width/(q_width + r_width) + history_buffer_.last().width_*r_width/(q_width + r_width);
        float height_estimated = height_*q_height/(q_height + r_height) + history_buffer_.last().height_*r_height/(q_height + r_height);

        length_ = length_estimated;
        width_ = width_estimated;
        height_ = height_estimated;
        
        
        long int time2 = history_buffer_.last().Ntime_;


    }

    float DetectedObject::History::operator-(const DetectedObject::History &object) const
    {
        return std::sqrt(pow(this->centroid_.x - object.centroid_.x, 2) 
        + pow(this->centroid_.y - object.centroid_.y, 2) 
        + pow(this->centroid_.z - object.centroid_.z, 2));
    }

    std::ostream& operator<<(std::ostream& out, DetectedObject& object)
    {
        out<<"length: "<<object.length_<<std::endl
        <<"width: "<<object.width_<<std::endl
        <<"height: "<<object.height_<<std::endl
        <<"centroid: "<<object.centroid_.x<<','<<object.centroid_.y<<','<<object.centroid_.z<<std::endl;
        return out;
    }
}