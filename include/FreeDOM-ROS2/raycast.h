//
// Created by ZhiangQi on 25-5-5.
//

#ifndef RAYCAST_H
#define RAYCAST_H
#include <FreeDOM-ROS2/common_types.h>

namespace freedom{
    const double MAX_TIME = 1.0;
    class RayCaster
    {
    public:
        RayCaster(const Point& start_, const Index& start_idx_, Index& idx_, const double& res_) :
            start(start_), start_idx(start_idx_), idx(idx_), res(res_) {}

        void setRayEnd(const Point& end_, const Index& end_idx_);
        bool step();

    private:
        Point start;
        Index start_idx;

        Point end;
        Index end_idx;

        int step_x;
        int step_y;
        int step_z;

        double t_step_x;
        double t_step_y;
        double t_step_z;

        double t_to_next_bound_x;
        double t_to_next_bound_y;
        double t_to_next_bound_z;

        Index& idx;
        double res;

        inline double tToNextBound(const double& pos,const double& vel,const double& t_step)
        {
            if(vel == 0) return MAX_TIME;
            double offset = -fmod(pos,res)/vel;
            return offset < 0 ? offset + t_step : offset;
        }
    };
}
#endif //RAYCAST_H
