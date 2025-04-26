#include "freedom/depth_image.h"

namespace freedom{
void DepthImage::set_params(const DepthImageConfig& config)
{
    lidar_horizon_fov = config.lidar_horizon_fov;
    lidar_vertical_fov_upper = config.lidar_vertical_fov_upper;
    lidar_vertical_fov_lower = config.lidar_vertical_fov_lower;
    lidar_vertical_fov = lidar_vertical_fov_upper - lidar_vertical_fov_lower;
    rows = config.depth_image_vertical_lines;
    
    depth_image_min_range = config.depth_image_min_range;
    max_raycast_enhancement_range = config.max_raycast_enhancement_range;
    inpaint_size = config.inpaint_size;
    erosion_size = config.erosion_size;
    learn_fov = config.learn_fov;
    enable_fov_mask = config.enable_fov_mask;
    fov_mask_path = config.fov_mask_path;

    depth_unit = max_raycast_enhancement_range/255.0;  // depthImage为CV_8UC1型，取值为0到255
    depth_unit_inv = 255.0/max_raycast_enhancement_range;

    vertical_res = lidar_vertical_fov / (rows - 1);
    vertical_res_half = vertical_res / 2.0;
    vertical_res_inv = 1.0 / vertical_res;
    vertical_fov = vertical_res * rows;
    vertical_fov_upper = lidar_vertical_fov_upper + vertical_res_half;
    vertical_fov_lower = lidar_vertical_fov_lower - vertical_res_half;

    cols = static_cast<unsigned int>(std::ceil(lidar_horizon_fov / vertical_res));
    horizon_res = lidar_horizon_fov / cols;
    horizon_res_half = horizon_res / 2.0;
    horizon_res_inv = 1.0 / horizon_res;

    raycast_enhancement_depth_margin = static_cast<unsigned int>(std::ceil(config.raycast_enhancement_depth_margin * depth_unit_inv));

    min_raycast_enhancement_area_pixel_num = static_cast<unsigned int>(std::ceil(rows * cols * config.min_raycast_enhancement_area));
    top_margin = static_cast<unsigned int>(std::ceil(config.top_margin * rows));

    erosion_element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                                cv::Point(erosion_size, erosion_size));

    is_panorama = (lidar_horizon_fov < CV_2PI)? false : true;
    col_margin = is_panorama? erosion_size + inpaint_size : 0;
    cols_w_margin  = cols + col_margin * 2;     // 此为加上补偿边的列数
    image_col_min = col_margin;                 // image的列下标从image_col_min到image_col_max-1
    image_col_max = cols + col_margin;
    right_margin_src.start = image_col_min;                 //需要填充到右侧的列数下标为image_col_min到image_col_min + col_margin - 1
    right_margin_src.end = image_col_min + col_margin;
    left_margin_src.start = image_col_max - col_margin;     //需要填充到左侧的列数下标为image_col_max - col_margin到image_col_max - 1
    left_margin_src.end = image_col_max;
    right_margin.start = image_col_max;
    right_margin.end = cols_w_margin;
    left_margin.start = 0;
    left_margin.end = image_col_min;
    image_col_range.start = image_col_min;
    image_col_range.end = image_col_max;

    depth_image = std::make_unique<cv::Mat>(rows, cols_w_margin, CV_8UC1);
    raycast_enhance_region = std::make_unique<cv::Mat>(rows, cols_w_margin, CV_8UC1);
    inpainted_image = std::make_unique<cv::Mat>(rows, cols_w_margin, CV_8UC1);
    eroded_raycast_enhance_region = std::make_unique<cv::Mat>(rows, cols_w_margin, CV_8UC1);

    fov_image = std::make_unique<cv::Mat>(rows, cols_w_margin, CV_8UC1);
    fov_image->setTo(cv::Scalar(255));
    if(enable_fov_mask && !learn_fov)
    {
        cv::Mat raw_fov_image = cv::imread(fov_mask_path, cv::IMREAD_GRAYSCALE);
        if(static_cast<unsigned int>(raw_fov_image.rows) != rows || static_cast<unsigned int>(raw_fov_image.cols) != cols)
            std::cerr << "FOV mask shape does not match, (" << rows << "," << cols << ") required, (" << raw_fov_image.rows << "," << raw_fov_image.cols << ") provided." << std::endl;
        
        raw_fov_image.copyTo((*fov_image)(cv::Range::all(), image_col_range));
        
        if(is_panorama)
        {
            fov_image->colRange(right_margin_src).copyTo(fov_image->colRange(right_margin));
            fov_image->colRange(left_margin_src).copyTo(fov_image->colRange(left_margin));
        }
    }

    if(learn_fov)
        average_image = std::make_unique<cv::Mat>(cv::Mat::zeros(rows, cols, CV_32FC1));

    scan_count = 0;
    num_threads = config.num_threads;
}

void DepthImage::raycast_enhancement(const pcl::PointCloud<pcl::PointXYZ>& cloud, const Eigen::Isometry3d& transform)
{
    // 清空上一帧的数据
    reset();
    
    unsigned int row,col,depth;

    // 生成深度图与深度补全范围
    size_t cloud_size = cloud.size();
    for(size_t id = 0; id < cloud_size; ++id)
    {
        if(!point2idx(cloud[id], row, col, depth))
            continue;
        
        if(depth < depth_image->at<uchar>(row, col))
            depth_image->at<uchar>(row, col) = depth;
        
        raycast_enhance_region->at<uchar>(row, col) = 0;
    }

    if(learn_fov)
    {
        scan_count ++;
        for (unsigned int i = 0; i < rows; ++i)
        {
            for (unsigned int j = 0; j < cols; ++j)
            {
                float new_value = (raycast_enhance_region->at<uchar>(i, j + col_margin) < 128)? 0.0:1.0;
                average_image->at<float>(i, j) = new_value*(1.0/scan_count) + average_image->at<float>(i, j)*(static_cast<float>(scan_count-1)/scan_count);
            }
        }
        return;
    }

    // 将depth_image和raycast_enhance左右两侧图像分别复制到另一侧的margin
    if(is_panorama)
    {
        depth_image->colRange(right_margin_src).copyTo(depth_image->colRange(right_margin));
        depth_image->colRange(left_margin_src).copyTo(depth_image->colRange(left_margin));
        raycast_enhance_region->colRange(right_margin_src).copyTo(raycast_enhance_region->colRange(right_margin));
        raycast_enhance_region->colRange(left_margin_src).copyTo(raycast_enhance_region->colRange(left_margin));
    }

    // 将depth_image腐蚀（局部最小）以保证边缘深度的安全
    cv::morphologyEx(*depth_image, *inpainted_image, cv::MORPH_ERODE, erosion_element);

    // 补充深度缺失的部分
    cv::inpaint(*inpainted_image, *raycast_enhance_region, *inpainted_image, inpaint_size, cv::INPAINT_TELEA);

    // 对深度补全范围进行腐蚀
    cv::morphologyEx(*raycast_enhance_region, *eroded_raycast_enhance_region, cv::MORPH_ERODE, erosion_element);

    // 去除过小的raycast_enhancement区域
    cv::Mat labels, stats, centroids;
    int numComponents = cv::connectedComponentsWithStats(*eroded_raycast_enhance_region, labels, stats, centroids, 4, CV_16U, cv::CCL_DEFAULT);

    for(int i = 1; i < numComponents; ++i)
    {
        if( static_cast<unsigned int>(stats.at<int>(i, cv::CC_STAT_AREA)) <= min_raycast_enhancement_area_pixel_num || 
            static_cast<unsigned int>(stats.at<int>(i, cv::CC_STAT_TOP)) > top_margin || 
            static_cast<unsigned int>(stats.at<int>(i, cv::CC_STAT_TOP) + stats.at<int>(i, cv::CC_STAT_HEIGHT)) >= rows)
            eroded_raycast_enhance_region->setTo(cv::Scalar(0), labels == i);
    }

    for(unsigned int col = image_col_min; col < image_col_max; ++col)
    {
        for(unsigned int row = 0; row < rows; ++row)
        {
            if(eroded_raycast_enhance_region->at<uchar>(row,col) <= 0)
                continue;
            
            Point raycast_enhancement_point;
            idx2point(row,col,inpainted_image->at<uchar>(row,col),raycast_enhancement_point);
            enhanced_pointcloud.emplace_back(transform * raycast_enhancement_point);
        }
    }
}

DepthImage::~DepthImage()
{
    if(learn_fov)
    {
        cv::Mat normalized_image;
        cv::Mat normalized_image_erode;
        cv::Mat fov_image;
        cv::Mat fov_image_result;
        average_image->convertTo(normalized_image, CV_8UC1, 255);

        cv::Mat raycast_enhancement_region_inflate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
        cv::Mat raycast_enhancement_region_erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5), cv::Point(2, 2));
        cv::erode(normalized_image, normalized_image_erode, raycast_enhancement_region_inflate);
        cv::threshold(normalized_image_erode, fov_image, 225, 255, cv::THRESH_BINARY_INV);
        cv::erode(fov_image, fov_image_result, raycast_enhancement_region_erode);

        if(cv::imwrite(fov_mask_path, fov_image_result))
            std::cout << "Image successfully saved!" << std::endl;
        else
            std::cerr << "Failed to save the image." << std::endl;
    }
}

void DepthImage::reset()
{
    depth_image->setTo(cv::Scalar(255));

    // 图像补全范围应该有根据常遮挡范围图像初始化的处理，待完善
    // raycast_enhance->setTo(cv::Scalar(255));
    if(enable_fov_mask && !learn_fov)
    {
        *raycast_enhance_region = fov_image->clone();
        *eroded_raycast_enhance_region = fov_image->clone();
    }
    else
    {
        raycast_enhance_region->setTo(cv::Scalar(255));
        eroded_raycast_enhance_region->setTo(cv::Scalar(255));
    }

    Points().swap(enhanced_pointcloud);
}

bool DepthImage::point2idx(const pcl::PointXYZ& point, unsigned int& row, unsigned int& col,unsigned int& depth) const
{
    double row_angle = std::atan2(point.z, std::sqrt(point.x*point.x + point.y*point.y));
    double col_angle = std::atan2(point.y, point.x);

    int row_ = std::floor((vertical_fov_upper - row_angle)/vertical_res);
    int col_ = std::floor((lidar_horizon_fov/2.0 - col_angle)/horizon_res) + col_margin;    // 此为全景图列数

    if( row_ < 0 ||
        row_ >= static_cast<int>(rows) ||
        col_ <  static_cast<int>(image_col_min) ||
        col_ >= static_cast<int>(image_col_max))
        return false;

    row = static_cast<unsigned int>(row_);
    col = static_cast<unsigned int>(col_);

    double point_depth = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);

    // 用于滤除激光雷达附近的奇怪鬼影
    if(point_depth < depth_image_min_range)
        return false;

    depth = std::min(255u,static_cast<unsigned int>(std::floor(depth_unit_inv * point_depth)));
    return true;
}

void DepthImage::idx2point(unsigned int row, unsigned int col,unsigned int depth,Point& point) const
{
    double row_angle = vertical_fov_upper - row * vertical_res - vertical_res_half;
    double col_angle = lidar_horizon_fov/2.0 - (col - col_margin) * horizon_res - horizon_res_half;

    double point_depth = (  depth > raycast_enhancement_depth_margin?
                                depth - raycast_enhancement_depth_margin : 0)  * depth_unit;

    double point_depth_projected = point_depth * std::cos(row_angle);
    point = Point(  point_depth_projected * std::cos(col_angle),
                    point_depth_projected * std::sin(col_angle),
                    point_depth * std::sin(row_angle));
}
}