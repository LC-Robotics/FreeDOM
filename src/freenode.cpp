//
// Created by ZhiangQi on 25-5-5.
//

#include "FreeDOM-ROS2/freenode.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace freedom{
FreeNode::FreeNode():Node("free_node")
{

    // 获取ros参数与FreeDOM参数以及visualizer的参数
    FreeDOM::Config map_config;
    Visualizer::Config vis_config;
    get_params(map_config,vis_config);

    // 设置FreeDOM与visualizer的config
    static_map.set_params(map_config);
    mapping_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // 使用回调组创建订阅
    rclcpp::SubscriptionOptions options;
    options.callback_group = mapping_callback_group_;

    pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic,
        rclcpp::QoS(100),
        std::bind(&FreeNode::pointcloud_callback, this, std::placeholders::_1),
        options);  // 加入 options 参数
    if(enable_visualization) visualizer.set_params(vis_config,*this);

    // 设置可视化回调
    if(enable_visualization)
    {
        static_map.set_scan_removal_callback([&](const ScanMap& scan){
            visualizer.visualize_scan_removal_result(scan);});

        if(enable_raycast_enhancement)
            static_map.set_raycast_enhancement_callback([&](const DepthImage& image){
                visualizer.visualize_raycast_enhancement_result(image);});

        static_map.set_map_removal_callback([&](const MRMap& map){
            visualizer.visualize_map_removal_result(map);});
    }

    // 为建图线程设置独立回调队列
    mapping_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

    // 设置订阅
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic,
        rclcpp::QoS(100),
        std::bind(&FreeNode::pointcloud_callback, this, std::placeholders::_1));

    save_map_sub = this->create_subscription<std_msgs::msg::Empty>(
        save_map_topic,
        10,
        std::bind(&FreeNode::save_map_callback, this, std::placeholders::_1));
    // 启动建图线程
    thread = std::thread(&FreeNode::mapping_thread, this);
}

FreeNode::~FreeNode()
{
    if (thread.joinable())
        thread.join();
}

void FreeNode::get_params(FreeDOM::Config& map_config, Visualizer::Config& vis_config)
{
    // 声明所有参数及其默认值
    // 基本参数
    this->declare_parameter("pointcloud_topic", "/unilidar/cloud");
    this->declare_parameter("map_tf_frame", "unilidar_lidar");
    this->declare_parameter("sensor_tf_frame", "unilidar_lidar");
    this->declare_parameter("save_map_topic", "save_map");
    this->declare_parameter("save_map_path", "map.pcd");
    this->declare_parameter("enable_visualization", true);
    this->declare_parameter("raycast_enhancement.enable_raycast_enhancement", false);

    // 传感器参数
    this->declare_parameter("sensor.min_range", 0.0);
    this->declare_parameter("sensor.max_range", 50.0);
    this->declare_parameter("sensor.min_z", -20.0);
    this->declare_parameter("sensor.max_z", 20.0);

    // 地图参数
    this->declare_parameter("map.sub_voxel_size", 0.1);
    this->declare_parameter("map.voxel_depth", 2);
    this->declare_parameter("map.block_depth", 5);
    this->declare_parameter("map.enable_local_map", false);
    this->declare_parameter("map.local_map_range", 100.0);
    this->declare_parameter("map.local_map_min_z", 20.0);
    this->declare_parameter("map.local_map_max_z", -20.0);
    this->declare_parameter("map.raycast_max_range", 100.0);
    this->declare_parameter("map.raycast_min_z", 20.0);
    this->declare_parameter("map.raycast_max_z", -20.0);
    this->declare_parameter("map.counts_to_free", 6);
    this->declare_parameter("map.counts_to_revert", 20);
    this->declare_parameter("map.conservative_connectivity", 26);
    this->declare_parameter("map.aggressive_connectivity", 124);

    // 光线投射增强参数
    //this->declare_parameter("raycast_enhancement.enable_raycast_enhancement", false);
    this->declare_parameter("raycast_enhancement.lidar_horizon_fov_degree", 0.0);
    this->declare_parameter("raycast_enhancement.lidar_vertical_fov_upper_degree", 0.0);
    this->declare_parameter("raycast_enhancement.lidar_vertical_fov_lower_degree", 0.0);
    this->declare_parameter("raycast_enhancement.depth_image_vertical_lines", 32);
    this->declare_parameter("raycast_enhancement.depth_image_min_range", 0.0);
    this->declare_parameter("raycast_enhancement.max_raycast_enhancement_range", 50.0);
    this->declare_parameter("raycast_enhancement.raycast_enhancement_depth_margin", 0.0);
    this->declare_parameter("raycast_enhancement.inpaint_size", 3);
    this->declare_parameter("raycast_enhancement.erosion_size", 0);
    this->declare_parameter("raycast_enhancement.min_raycast_enhancement_area", 0.0);
    this->declare_parameter("raycast_enhancement.depth_image_top_margin", 0.0);
    this->declare_parameter("raycast_enhancement.learn_fov", false);
    this->declare_parameter("raycast_enhancement.enable_fov_mask", false);
    this->declare_parameter("raycast_enhancement.fov_mask_path", "");
    this->declare_parameter("raycast_enhancement.fov_mask_name", "");

    // 线程参数
    this->declare_parameter("num_threads", 8);

    // 获取基本参数
    pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
    map_tf_frame = this->get_parameter("map_tf_frame").as_string();
    sensor_tf_frame = this->get_parameter("sensor_tf_frame").as_string();
    save_map_topic = this->get_parameter("save_map_topic").as_string();
    save_map_path = this->get_parameter("save_map_path").as_string();
    enable_visualization = this->get_parameter("enable_visualization").as_bool();
    enable_raycast_enhancement = this->get_parameter("raycast_enhancement.enable_raycast_enhancement").as_bool();

    // 获取传感器参数
    map_config.sensor_min_range = this->get_parameter("sensor.min_range").as_double();
    map_config.sensor_max_range = this->get_parameter("sensor.max_range").as_double();
    map_config.sensor_min_z = this->get_parameter("sensor.min_z").as_double();
    map_config.sensor_max_z = this->get_parameter("sensor.max_z").as_double();

    // 获取地图参数
    map_config.sub_voxel_size = this->get_parameter("map.sub_voxel_size").as_double();
    map_config.voxel_depth = this->get_parameter("map.voxel_depth").as_int();
    map_config.block_depth = this->get_parameter("map.block_depth").as_int();

    map_config.enable_local_map = this->get_parameter("map.enable_local_map").as_bool();
    if(map_config.enable_local_map)
    {
        map_config.local_map_range = this->get_parameter("map.local_map_range").as_double();
        map_config.local_map_min_z = this->get_parameter("map.local_map_min_z").as_double();
        map_config.local_map_max_z = this->get_parameter("map.local_map_max_z").as_double();
    }

    map_config.raycast_max_range = this->get_parameter("map.raycast_max_range").as_double();
    map_config.raycast_min_z = this->get_parameter("map.raycast_min_z").as_double();
    map_config.raycast_max_z = this->get_parameter("map.raycast_max_z").as_double();

    map_config.counts_to_free = this->get_parameter("map.counts_to_free").as_int();
    map_config.counts_to_revert = this->get_parameter("map.counts_to_revert").as_int();

    map_config.conservative_connectivity = this->get_parameter("map.conservative_connectivity").as_int();
    map_config.aggressive_connectivity = this->get_parameter("map.aggressive_connectivity").as_int();

    map_config.enable_raycast_enhancement = this->get_parameter("raycast_enhancement.enable_raycast_enhancement").as_bool();
    if(map_config.enable_raycast_enhancement)
    {
        double lidar_horizon_fov_degree = this->get_parameter("raycast_enhancement.lidar_horizon_fov_degree").as_double();
        double lidar_vertical_fov_upper_degree = this->get_parameter("raycast_enhancement.lidar_vertical_fov_upper_degree").as_double();
        double lidar_vertical_fov_lower_degree = this->get_parameter("raycast_enhancement.lidar_vertical_fov_lower_degree").as_double();

        map_config.lidar_horizon_fov = lidar_horizon_fov_degree * CV_PI / 180.0;
        map_config.lidar_vertical_fov_upper = lidar_vertical_fov_upper_degree * CV_PI / 180.0;
        map_config.lidar_vertical_fov_lower = lidar_vertical_fov_lower_degree * CV_PI / 180.0;

        map_config.depth_image_vertical_lines = this->get_parameter("raycast_enhancement.depth_image_vertical_lines").as_int();
        map_config.depth_image_min_range = this->get_parameter("raycast_enhancement.depth_image_min_range").as_double();
        map_config.max_raycast_enhancement_range = this->get_parameter("raycast_enhancement.max_raycast_enhancement_range").as_double();
        map_config.raycast_enhancement_depth_margin = this->get_parameter("raycast_enhancement.raycast_enhancement_depth_margin").as_double();

        map_config.inpaint_size = this->get_parameter("raycast_enhancement.inpaint_size").as_int();
        map_config.erosion_size = this->get_parameter("raycast_enhancement.erosion_size").as_int();

        map_config.min_raycast_enhancement_area = this->get_parameter("raycast_enhancement.min_raycast_enhancement_area").as_double();
        map_config.depth_image_top_margin = this->get_parameter("raycast_enhancement.depth_image_top_margin").as_double();

        map_config.learn_fov = this->get_parameter("raycast_enhancement.learn_fov").as_bool();
        map_config.enable_fov_mask = this->get_parameter("raycast_enhancement.enable_fov_mask").as_bool();

        if(map_config.learn_fov || map_config.enable_fov_mask)
        {
            std::string fov_mask_path = this->get_parameter("raycast_enhancement.fov_mask_path").as_string();
            std::string fov_mask_name = this->get_parameter("raycast_enhancement.fov_mask_name").as_string();
            map_config.fov_mask_path = fov_mask_path + fov_mask_name;
        }
    }

    map_config.num_threads = this->get_parameter("num_threads").as_int();

    // 设置visualizer参数
    vis_config.map_tf_frame = this->get_parameter("map_tf_frame").as_string();
    vis_config.sub_voxel_size = this->get_parameter("map.sub_voxel_size").as_double();
    vis_config.voxel_depth = this->get_parameter("map.voxel_depth").as_int();
    vis_config.block_depth = this->get_parameter("map.block_depth").as_int();
    vis_config.enable_raycast_enhancement = this->get_parameter("raycast_enhancement.enable_raycast_enhancement").as_bool();
}

    void FreeNode::mapping_thread()
{
    // ROS2 的线程安全检查
    if (!rclcpp::ok()) {
        return;
    }
    // 使用执行器替代 callAvailable
    // 这个执行器将处理分配给 mapping_callback_group_ 的所有回调
    try {
        RCLCPP_INFO(this->get_logger(), "Mapping thread started");
        // 添加安全检查
        if (!mapping_executor_) {
            RCLCPP_ERROR(this->get_logger(), "Mapping executor not initialized!");
            return; // 如果执行器为空，立即返回
        }
        rclcpp::spin(this->get_node_base_interface());
        RCLCPP_INFO(this->get_logger(), "Mapping thread stopped");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in mapping thread: %s", e.what());
    }catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception in mapping thread");
    }
}

    void FreeNode::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*pointcloud, cloud);
    if(cloud.points.empty())
    {
        RCLCPP_WARN(this->get_logger(), "pointcloud empty");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "lidar_transform: pointcloud received, %zu points", cloud.points.size());

    geometry_msgs::msg::TransformStamped transformStamped;
    Eigen::Isometry3d transform;

    rclcpp::Time transform_time = pointcloud->header.stamp;
    try{
        // 等待点云时间戳对应的tf可用
        std::string error_msg;
        if(!tf_buffer->canTransform(map_tf_frame, sensor_tf_frame, transform_time, rclcpp::Duration::from_seconds(0.5), &error_msg))
        {
            RCLCPP_WARN(this->get_logger(), "no tf available from frame: %s to frame: %s at time: %.6f, error: %s",
                        map_tf_frame.c_str(), sensor_tf_frame.c_str(),
                        rclcpp::Time(transform_time).seconds(), error_msg.c_str());
            return;
        }
        transformStamped = tf_buffer->lookupTransform(map_tf_frame, sensor_tf_frame, transform_time);
        transform = tf2::transformToEigen(transformStamped);
    }
    catch(tf2::TransformException &ex){
        RCLCPP_WARN(this->get_logger(), "TF exception: %s", ex.what());
        return;
    }

    static_map.pointcloud_integrate(cloud, transform);
}

    void FreeNode::save_map_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received save map trigger. Saving map...");
    static_map.save_map(save_map_path);
    RCLCPP_INFO(this->get_logger(), "Static map saved successfully to %s", save_map_path.c_str());
}
}