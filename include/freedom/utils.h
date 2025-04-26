#ifndef _UTILS_H
#define _UTILS_H

#include <ros/ros.h>
#include <unordered_map>
#include <Eigen/Eigen>
#include <geometry_msgs/TransformStamped.h>

#include "freedom/common_types.h"

namespace freedom{
// 简化获取参数的函数
class Param{
public:
    Param(ros::NodeHandle& nh): nh_(nh){}

    template <typename T>
    // 若提供default_value，则在找不到参数时提供默认值
    void getParam(const std::string& param_name, T& param_value, const T& default_value)
    {
        if(!nh_.param(param_name,param_value,default_value))
            ROS_ERROR_STREAM("param missing:" << param_name << ", set default:" << default_value);
        
        return;
    }

    template <typename T>
    // 若不提供default_value，则在找不到参数时退出
    void getParam(const std::string& param_name, T& param_value)
    {
        if(!nh_.param(param_name,param_value,T()))
        {
            ROS_ERROR_STREAM("param missing:" << param_name << ", node exit");
            ros::shutdown();
            std::exit(EXIT_FAILURE);
        }
    }

private:
    ros::NodeHandle& nh_;
};

// 计时器
class Timer{
public:
    struct TimerData
    {
        std::string name;
        std::chrono::high_resolution_clock::time_point start_time;
        double total_time = 0;
        int count = 0;
        bool is_running = false;

        TimerData(const std::string& timer_name = "") : name(timer_name) {}

        void start()
        {
            start_time = std::chrono::high_resolution_clock::now();
            is_running = true;
        }

        void stop() {
            if (!is_running) {
                std::cout << "Timer is not running!" << std::endl;
                return;
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            total_time += elapsed;
            ++ count;
            is_running = false;

            std::cout << "[" << name << "] time: " << elapsed << " ms" << std::endl;
        }

        void reset()
        {
            total_time = 0.0;
            count = 0;
            is_running = false;
        }
    };

    TimerData& operator[](const std::string& timer_name)
    {
        for (auto& timer : timers_) {
            if (timer.name == timer_name) {
                return timer;
            }
        }
        timers_.emplace_back(timer_name);
        return timers_.back();
    }

    Timer(){}
    ~Timer()
    {
        if(timers_.size() > 0)
        {
            std::cout << "\n\033[1;33m================= Timer Summary =================\033[0m\n";
            double total_average_time = 0.0;
            for (const auto& timer : timers_) {
                if (timer.count > 0)
                {
                    std::cout << "  [" << timer.name << "] Average: " << (timer.total_time / timer.count) << " ms" << std::endl;
                    total_average_time += timer.total_time / timer.count;
                }
                else
                    std::cout << "  [" << timer.name << "] No recorded intervals" << std::endl;
            }
            std::cout << "  [total] Average: " << total_average_time << " ms" << std::endl;
            std::cout << "\033[1;33m=================================================\033[0m\n";
        }
    }

private:
    std::vector<TimerData> timers_;
};

// TransformStamped to Eigen
inline void transformfromTFToEigen(const geometry_msgs::TransformStamped& transform_stamped, Eigen::Isometry3d& transform)
{
    transform = Eigen::Isometry3d::Identity();

    Eigen::Vector3d t(transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z);
    Eigen::Quaterniond q(transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z);

    transform.translate(t);  // 设置平移
    transform.rotate(q);     // 设置旋转

    return;
}

// 线程安全的vector遍历分配
// 需要保证get_ptr的指针使用周期内vector不发生改变
template <typename T>
class VectorElementGetter {
public:
    explicit VectorElementGetter(std::vector<T,Eigen::aligned_allocator<T>>& elements): elements_(elements), current_index_(0){}

    bool get_ptr(T*& element) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_index_ >= elements_.size())
            return false;

        element = &elements_[current_index_];
        ++ current_index_;
        return true;
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_index_ = 0;
    }

private:
    std::mutex mutex_;
    std::vector<T,Eigen::aligned_allocator<T>>& elements_;
    size_t current_index_;
};

template <typename T>
class constVectorElementGetter {
public:
    explicit constVectorElementGetter(const std::vector<T,Eigen::aligned_allocator<T>>& elements): elements_(elements), current_index_(0){}

    bool get_ptr(const T*& element) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_index_ >= elements_.size())
            return false;

        element = &elements_[current_index_];
        ++ current_index_;
        return true;
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_index_ = 0;
    }

private:
    std::mutex mutex_;
    const std::vector<T,Eigen::aligned_allocator<T>>& elements_;
    size_t current_index_;
};

// 默认最小idx为0,0,0且最大值相同
inline void incrementIdx(Index& idx, const unsigned int& max_idx)
{
    idx.z()++;
    if (idx.z() >= max_idx)
    {
        idx.z() = 0;
        idx.y()++;

        if (idx.y() >= max_idx)
        {
            idx.y() = 0;
            idx.x()++;
        }
    }
}

// 默认最小idx为0,0,0
inline void incrementIdx(Index& idx, const Index& max_idx)
{
    idx.z()++;
    if (idx.z() >= max_idx.z())
    {
        idx.z() = 0;
        idx.y()++;

        if (idx.y() >= max_idx.y())
        {
            idx.y() = 0;
            idx.x()++;
        }
    }
}

inline void incrementIdx(Index& idx, const Index& min_idx, const Index& max_idx)
{
    idx.z()++;
    if (idx.z() >= max_idx.z())
    {
        idx.z() = min_idx.z();
        idx.y()++;

        if (idx.y() >= max_idx.y())
        {
            idx.y() = min_idx.y();
            idx.x()++;
        }
    }
}

class Neighbours{
public:
    Neighbours(){}
    inline void set_params(unsigned int connectivity);
    std::vector<Index> offsets;
};

inline void Neighbours::set_params(unsigned int connectivity)
{
    offsets.reserve(26);
    offsets.emplace_back(Index(-1,  0,  0));
    offsets.emplace_back(Index( 1,  0,  0));
    offsets.emplace_back(Index( 0, -1,  0));
    offsets.emplace_back(Index( 0,  1,  0));
    offsets.emplace_back(Index( 0,  0, -1));
    offsets.emplace_back(Index( 0,  0,  1));
    offsets.emplace_back(Index(-1, -1,  0));
    offsets.emplace_back(Index(-1,  1,  0));
    offsets.emplace_back(Index( 1, -1,  0));
    offsets.emplace_back(Index( 1,  1,  0));
    offsets.emplace_back(Index( 0, -1, -1));
    offsets.emplace_back(Index( 0, -1,  1));
    offsets.emplace_back(Index( 0,  1, -1));
    offsets.emplace_back(Index( 0,  1,  1));
    offsets.emplace_back(Index(-1,  0, -1));
    offsets.emplace_back(Index( 1,  0, -1));
    offsets.emplace_back(Index(-1,  0,  1));
    offsets.emplace_back(Index( 1,  0,  1));
    offsets.emplace_back(Index(-1, -1, -1));
    offsets.emplace_back(Index(-1, -1,  1));
    offsets.emplace_back(Index(-1,  1, -1));
    offsets.emplace_back(Index(-1,  1,  1));
    offsets.emplace_back(Index( 1, -1, -1));
    offsets.emplace_back(Index( 1, -1,  1));
    offsets.emplace_back(Index( 1,  1, -1));
    offsets.emplace_back(Index( 1,  1,  1));

    if(connectivity == 6 || connectivity == 18 || connectivity == 26)
        offsets.resize(connectivity);
    else
        std::cout << "Connectivity not supproted" << std::endl;
}

class ProgressBar
{
public:
    ProgressBar(std::string prefix_,int bar_length_,int total_steps_,int skip_):
        prefix(prefix_),
        bar_length(bar_length_),
        total_steps(total_steps_),
        skip(skip_),
        end(total_steps_%skip_),
        current_step(0),
        start_time(ros::Time::now()),
        last_print_time(ros::Time::now()){}
    
    inline void step()
    {
        current_step ++;

        if(!time_to_print())
            return;
        
        unsigned int cur_bar_length = (bar_length * current_step)/total_steps;
        std::string bar_string(cur_bar_length,'=');
        std::string empty_bar_string(bar_length - cur_bar_length,' ');
        
        std::cout << std::left << std::setw(12) << prefix << "[" << bar_string << ">" << empty_bar_string << "]" << (100 * current_step)/total_steps << "% " << std::fixed << std::setprecision(3) << (last_print_time - start_time).toSec() << "s\r";
        std::cout.flush();

        if(current_step == total_steps)
            std::cout << std::endl;
    }

    inline bool time_to_print()
    {
        ros::Time now = ros::Time::now();

        if(current_step%skip == end || (now - last_print_time).toSec() > 0.0333)
        {
            last_print_time = now;
            return true;
        }

        return false;
    }

private:
    std::string prefix;
    unsigned int bar_length;
    unsigned int total_steps;
    unsigned int current_step;
    unsigned int skip;
    unsigned int end;

    ros::Time start_time;
    ros::Time last_print_time;
};

}
#endif