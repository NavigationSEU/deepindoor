#pragma once

namespace tracking
{
    template <typename T>
    class CircularBuffer
    {
    public:
        explicit CircularBuffer(const size_t &capacity);

        ~CircularBuffer()
        {

        }

        const size_t &size() const
        {
            return size_;
        }

        const size_t &capacity() const
        {
            return capacity_;
        }

        bool isEmpty() const
        {
            return size_ == 0;
        }

        /** \brief 返回缓存第i个元素
         * 
         *  @param i 索引 
         *  @return 缓存第i个元素
         * 
         */
        const T &operator[](const size_t &i)
        {
            return buffer_[(startIdx_ + i) % capacity_];
        }

        const T &first()
        {
            return buffer_[startIdx_];
        }

        const T &half()
        {
            size_t idx = size_ == 0 ? 0 : (startIdx_ + (int)(0.5*size_)) % capacity_;
            return buffer_[idx];
        }

        const T &last()
        {
            size_t idx = size_ == 0 ? 0 : (startIdx_ + size_ - 1) % capacity_;
            return buffer_[idx];
        }

        CircularBuffer<T>& push(const T &value)
        {
            if (size_ < capacity_)
            {
                buffer_[size_] = value;
                size_++;
            }
            else
            {
                buffer_[startIdx_] = value;
                startIdx_ = (startIdx_ + 1) % capacity_;
            }
            return *this;
        }

    private:
        size_t capacity_; // 缓存的容量
        size_t size_;     // 缓存现在的大小
        size_t startIdx_; // 现在的头索引
        std::shared_ptr<T[]> buffer_;
    };
    
    template<class T>
    CircularBuffer<T>::CircularBuffer(const size_t &capacity):capacity_(capacity),size_(0),startIdx_(0)
    {
        // std::shared_ptr<T> buffer_(new T[capacity_],[](T* p){delete []p;});
        std::shared_ptr<T[]> temp_buffer_(new T[capacity_]);
        buffer_ = temp_buffer_;
    }

}