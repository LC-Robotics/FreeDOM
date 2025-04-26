#include "freedom/raycast.h"

namespace freedom{
void RayCaster::setRayEnd(const Point& end_, const Index& end_idx_)
{
    // 重置idx到初始位置
    idx = start_idx;

    end = end_;
    end_idx = end_idx_;

    PointBias velocity = end - start;
    step_x = velocity.x() > 0? 1 : velocity.x()<0? -1 : 0;
    step_y = velocity.y() > 0? 1 : velocity.y()<0? -1 : 0;
    step_z = velocity.z() > 0? 1 : velocity.z()<0? -1 : 0;

    t_step_x = velocity.x() != 0 ? res / abs(velocity.x()) : MAX_TIME;
    t_step_y = velocity.y() != 0 ? res / abs(velocity.y()) : MAX_TIME;
    t_step_z = velocity.z() != 0 ? res / abs(velocity.z()) : MAX_TIME;

    t_to_next_bound_x = tToNextBound(start.x(),velocity.x(),t_step_x);
    t_to_next_bound_y = tToNextBound(start.y(),velocity.y(),t_step_y);
    t_to_next_bound_z = tToNextBound(start.z(),velocity.z(),t_step_z);
}

// 使用do{}while(step())的方式以从开始的体素遍历
// 将结束判断放在末尾以不包括末端体素
// 将结束判断放在开头以包括末端体素
bool RayCaster::step()
{   
    // 结束判断放在末尾，遍历时不包括末端体素
    if(t_to_next_bound_x < t_to_next_bound_y && t_to_next_bound_x < t_to_next_bound_z)
    {
        idx.x() += step_x;
        t_to_next_bound_x += t_step_x;
    }
    else if(t_to_next_bound_y < t_to_next_bound_z)
    {
        idx.y() += step_y;
        t_to_next_bound_y += t_step_y;
    }
    else
    {
        idx.z() += step_z;
        t_to_next_bound_z += t_step_z;
    }

    return (t_to_next_bound_x < MAX_TIME || t_to_next_bound_y < MAX_TIME || t_to_next_bound_z < MAX_TIME);


    // // 结束判断放在开头，遍历时包括末端体素
    // if(t_to_next_bound_x >= MAX_TIME && t_to_next_bound_y >= MAX_TIME && t_to_next_bound_z >= MAX_TIME)
    // {
    //     // std::cout << "return false" << std::endl;
    //     return false;
    // }

    // if(t_to_next_bound_x < t_to_next_bound_y && t_to_next_bound_x < t_to_next_bound_z)
    // {
    //     idx.x() += step_x;
    //     t_to_next_bound_x += t_step_x;

    //     // std::cout << "t_to_next_bound_x" << t_to_next_bound_x << std::endl;
    // }
    // else if(t_to_next_bound_y < t_to_next_bound_z)
    // {
    //     idx.y() += step_y;
    //     t_to_next_bound_y += t_step_y;

    //     // std::cout << "t_to_next_bound_y" << t_to_next_bound_y << std::endl;
    // }
    // else
    // {
    //     idx.z() += step_z;
    //     t_to_next_bound_z += t_step_z;

    //     // std::cout << "t_to_next_bound_z" << t_to_next_bound_z << std::endl;
    // }

    // // std::cout << "return true" << std::endl;
    // return true;
}














}