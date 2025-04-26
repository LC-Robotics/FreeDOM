#include "freedom/freenode.h"

namespace freedom{
FreeNode::FreeNode()
{
    ros::NodeHandle nh;     // 全局nh,用于订阅话题
    ros::NodeHandle nh_private("~");        // 局部nh,用于获取param和发布话题

    // 获取ros参数与FreeDOM参数以及visualizer的参数
    FreeDOM::Config map_config;
    Visualizer::Config vis_config;
    get_params(nh_private,map_config,vis_config);

    // 设置FreeDOM与visualizer的config
    static_map.set_params(map_config);
    if(enable_visualization) visualizer.set_params(vis_config,nh_private);

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
    nh.setCallbackQueue(&callback_queue);

    // 设置订阅
    tf_buffer.reset(new tf2_ros::Buffer(ros::Duration(10.0)));
    tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer));
    pointcloud_sub = nh.subscribe(pointcloud_topic, 100, &FreeNode::pointcloud_callback, this);
    save_map_sub = nh.subscribe(save_map_topic, 10, &FreeNode::save_map_callback, this);

    // 启动建图线程
    thread = std::thread(&FreeNode::mapping_thread, this);
}

FreeNode::~FreeNode()
{
    if (thread.joinable())
        thread.join();
}

void FreeNode::get_params(ros::NodeHandle &nh_private, FreeDOM::Config& map_config,Visualizer::Config& vis_config)
{   
    Param param(nh_private);

    // 获取ros params
    param.getParam<std::string>("pointcloud_topic",pointcloud_topic);
    param.getParam<std::string>("map_tf_frame",map_tf_frame);
    param.getParam<std::string>("sensor_tf_frame",sensor_tf_frame);
    param.getParam<std::string>("save_map_topic",save_map_topic);
    param.getParam<std::string>("save_map_path",save_map_path);
    param.getParam<bool>("enable_visualization",enable_visualization,true);
    param.getParam<bool>("raycast_enhancement/enable_raycast_enhancement",enable_raycast_enhancement,false);

    // 获取FreeDOM params
    param.getParam<double>("sensor/min_range",map_config.sensor_min_range,0.0);
    param.getParam<double>("sensor/max_range",map_config.sensor_max_range,50.0);
    param.getParam<double>("sensor/min_z",map_config.sensor_min_z,-20.0);
    param.getParam<double>("sensor/max_z",map_config.sensor_max_z,20.0);

    int voxel_depth,block_depth;
    param.getParam<double>("map/sub_voxel_size",map_config.sub_voxel_size,0.1);
    param.getParam<int>("map/voxel_depth",voxel_depth,2);
    param.getParam<int>("map/block_depth",block_depth,5);
    map_config.voxel_depth = voxel_depth;
    map_config.block_depth = block_depth;

    param.getParam<bool>("map/enable_local_map",map_config.enable_local_map,false);
    if(map_config.enable_local_map)
    {
        param.getParam<double>("map/local_map_range",map_config.local_map_range,100.0);
        param.getParam<double>("map/local_map_min_z",map_config.local_map_min_z,20.0);
        param.getParam<double>("map/local_map_max_z",map_config.local_map_max_z,-20.0);
    }

    param.getParam<double>("map/raycast_max_range",map_config.raycast_max_range,100.0);
    param.getParam<double>("map/raycast_min_z",map_config.raycast_min_z,20.0);
    param.getParam<double>("map/raycast_max_z",map_config.raycast_max_z,-20.0);

    int counts_to_free,counts_to_revert;
    param.getParam<int>("map/counts_to_free",counts_to_free,6);
    param.getParam<int>("map/counts_to_revert",counts_to_revert,20);
    map_config.counts_to_free = counts_to_free;
    map_config.counts_to_revert = counts_to_revert;

    int conservative_connectivity,aggressive_connectivity;
    param.getParam<int>("map/conservative_connectivity",conservative_connectivity,26);
    map_config.conservative_connectivity = conservative_connectivity;
    param.getParam<int>("map/aggressive_connectivity",aggressive_connectivity,124);
    map_config.aggressive_connectivity = aggressive_connectivity;

    param.getParam<bool>("raycast_enhancement/enable_raycast_enhancement",map_config.enable_raycast_enhancement,false);
    if(map_config.enable_raycast_enhancement)
    {
        double lidar_horizon_fov_degree,lidar_vertical_fov_upper_degree,lidar_vertical_fov_lower_degree;
        param.getParam<double>("raycast_enhancement/lidar_horizon_fov_degree",lidar_horizon_fov_degree,0.0);
        param.getParam<double>("raycast_enhancement/lidar_vertical_fov_upper_degree",lidar_vertical_fov_upper_degree,0.0);
        param.getParam<double>("raycast_enhancement/lidar_vertical_fov_lower_degree",lidar_vertical_fov_lower_degree,0.0);

        map_config.lidar_horizon_fov = lidar_horizon_fov_degree*CV_PI/180.0;
        map_config.lidar_vertical_fov_upper = lidar_vertical_fov_upper_degree*CV_PI/180.0;
        map_config.lidar_vertical_fov_lower = lidar_vertical_fov_lower_degree*CV_PI/180.0;

        int depth_image_vertical_lines;
        param.getParam<int>("raycast_enhancement/depth_image_vertical_lines",depth_image_vertical_lines,32);
        map_config.depth_image_vertical_lines = depth_image_vertical_lines;

        param.getParam<double>("raycast_enhancement/depth_image_min_range",map_config.depth_image_min_range,0.0);
        param.getParam<double>("raycast_enhancement/max_raycast_enhancement_range",map_config.max_raycast_enhancement_range,50.0);
        param.getParam<double>("raycast_enhancement/raycast_enhancement_depth_margin",map_config.raycast_enhancement_depth_margin,0.0);

        int inpaint_size,erosion_size;
        param.getParam<int>("raycast_enhancement/inpaint_size",inpaint_size,3);
        param.getParam<int>("raycast_enhancement/erosion_size",erosion_size,0);
        map_config.inpaint_size = inpaint_size;
        map_config.erosion_size = erosion_size;

        param.getParam<double>("raycast_enhancement/min_raycast_enhancement_area",map_config.min_raycast_enhancement_area,0.0);
        param.getParam<double>("raycast_enhancement/depth_image_top_margin",map_config.depth_image_top_margin,0.0);

        param.getParam<bool>("learn_fov",map_config.learn_fov,false);
        param.getParam<bool>("raycast_enhancement/enable_fov_mask",map_config.enable_fov_mask,false);

        if(map_config.learn_fov || map_config.enable_fov_mask)
        {   
            std::string fov_mask_path,fov_mask_name;
            param.getParam<std::string>("fov_mask_path",fov_mask_path);
            param.getParam<std::string>("raycast_enhancement/fov_mask_name",fov_mask_name);

            map_config.fov_mask_path = fov_mask_path + fov_mask_name;
        }
    }

    int num_threads;
    param.getParam<int>("num_threads",num_threads,8);
    map_config.num_threads = num_threads;

    // 获取visualizer参数
    param.getParam<std::string>("map_tf_frame",vis_config.map_tf_frame);
    param.getParam<double>("map/sub_voxel_size",vis_config.sub_voxel_size,0.1);
    param.getParam<int>("map/voxel_depth",vis_config.voxel_depth,2);
    param.getParam<int>("map/block_depth",vis_config.block_depth,5);
    param.getParam<bool>("raycast_enhancement/enable_raycast_enhancement",vis_config.enable_raycast_enhancement,false);
}

void FreeNode::mapping_thread()
{
    // 独立单线程建图
    while (ros::ok())
        callback_queue.callAvailable(ros::WallDuration(0.001));
}

void FreeNode::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*pointcloud, cloud);
    if(cloud.points.empty())
    {
        ROS_WARN_STREAM("pointcloud empty");
        return;
    }

    ROS_INFO("lidar_transform: pointcloud recieved,%zu points",cloud.points.size());

    geometry_msgs::TransformStamped transformStamped;
    Eigen::Isometry3d transform;

    ros::Time transform_time = pointcloud->header.stamp;
    try{
        //等待点云时间戳对应的tf可用
        if(!tf_buffer->canTransform(map_tf_frame,sensor_tf_frame,transform_time,ros::Duration(0.5)))
        {
            ROS_WARN_STREAM("no tf available from frame: " << map_tf_frame << "to frame: " << sensor_tf_frame << "at time: " << transform_time.toSec());
            return;
        }
        transformStamped = tf_buffer->lookupTransform(map_tf_frame,sensor_tf_frame,transform_time);
        transformfromTFToEigen(transformStamped, transform);
    }
    catch(tf2::TransformException &ex){
        ROS_WARN_STREAM(ex.what());
        return;
    }

    static_map.pointcloud_integrate(cloud,transform);
}

void FreeNode::save_map_callback(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_INFO("Received save map trigger. Saving map...");
    static_map.save_map(save_map_path);
    ROS_INFO("Static map saved successfully to %s",save_map_path.c_str());
}
}