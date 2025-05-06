//
// Created by ZhiangQi on 25-5-5.
//

#ifndef DEPTH_IMAGE_H
#define DEPTH_IMAGE_H

#include <opencv4/opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "FreeDOM-ROS2/common_types.h"

namespace freedom
{
    class DepthImage
    {
    public:
        struct DepthImageConfig
        {
            double lidar_horizon_fov;
            double lidar_vertical_fov_upper;
            double lidar_vertical_fov_lower;
            unsigned int depth_image_vertical_lines;            // depth image的垂直像素数

            double depth_image_min_range;                       // 忽略这个距离以下的点
            double max_raycast_enhancement_range;   // raycast_enhancement最远距离
            double raycast_enhancement_depth_margin;            // 论文中的safety margin r_m

            unsigned int inpaint_size;              // 单次inpaint大小
            unsigned int erosion_size;              // enhancement region的腐蚀大小
            double min_raycast_enhancement_area;        // 最小enhancement region
            double top_margin;                      // 用于顶部判断的margin

            bool learn_fov;                         // 若开启learn_fov，则不输出enhancement点云，并在程序退出时保存学习到的fov
            bool enable_fov_mask;                   // 开启fov mask以应对非矩形fov或固定遮挡
            std::string fov_mask_path;              // 保存或读取的fov mask路径

            unsigned int num_threads;
        };

        DepthImage(){}

        void set_params(const DepthImageConfig& config);

        void raycast_enhancement(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform);
        inline const cv::Mat& get_depth_image() const { return *depth_image; }
        inline const cv::Mat& get_raycast_enhance_region() const { return *raycast_enhance_region; }
        inline const cv::Mat& get_inpainted_image() const { return *inpainted_image; }
        inline const cv::Mat& get_eroded_raycast_enhance_region() const { return *eroded_raycast_enhance_region; }
        inline const cv::Mat& get_fov_image() const { return *fov_image; }
        inline const Points& get_enhanced_pointcloud() const { return enhanced_pointcloud; }

        ~DepthImage();
    private:
        // LiDAR fov
        double lidar_horizon_fov;
        double lidar_vertical_fov;
        double lidar_vertical_fov_upper;
        double lidar_vertical_fov_lower;

        // depth image
        unsigned int rows;
        unsigned int cols;
        double depth_unit,depth_unit_inv;
        double horizon_res,vertical_res;
        double horizon_res_half,vertical_res_half;
        double horizon_res_inv,vertical_res_inv;

        double vertical_fov;
        double vertical_fov_upper;
        double vertical_fov_lower;

        // raycast enhancement 参数
        double depth_image_min_range;
        double max_raycast_enhancement_range;
        unsigned int raycast_enhancement_depth_margin;

        unsigned int inpaint_size;
        unsigned int erosion_size;
        unsigned int min_raycast_enhancement_area_pixel_num;
        unsigned int top_margin;

        // 全景图对应参数
        bool is_panorama;
        unsigned int col_margin;     //根据卷积核大小得到的图像两侧区域大小，等效于图像左右连通
        unsigned int cols_w_margin;
        unsigned int image_col_min;  //图最小列数（不包括margin）
        unsigned int image_col_max;  //图最大列数（不包括margin）
        cv::Range right_margin_src;
        cv::Range left_margin_src;
        cv::Range right_margin;
        cv::Range left_margin;
        cv::Range image_col_range;

        // learn fov mask
        bool learn_fov;
        unsigned int scan_count;
        std::unique_ptr<cv::Mat> average_image;

        // fov mask
        bool enable_fov_mask;
        std::string fov_mask_path;

        // images
        std::unique_ptr<cv::Mat> depth_image;
        std::unique_ptr<cv::Mat> raycast_enhance_region;
        std::unique_ptr<cv::Mat> inpainted_image;
        std::unique_ptr<cv::Mat> eroded_raycast_enhance_region;
        std::unique_ptr<cv::Mat> fov_image;

        // enhanced pointcloud
        Points enhanced_pointcloud;

        // erosion core
        cv::Mat erosion_element;

        unsigned int num_threads;

        void reset();

        bool point2idx(const pcl::PointXYZ& point, unsigned int& row, unsigned int& col,unsigned int& depth) const;

        void idx2point(unsigned int row, unsigned int col,unsigned int depth,Point& point) const;
    };


}

#endif //DEPTH_IMAGE_H
