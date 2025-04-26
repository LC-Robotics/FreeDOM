#include <ros/ros.h>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "lidar/point_types.h"
#include "freedom/utils.h"
#include "freedom/common_types.h"

using namespace freedom;

bool frame_skipped;
unsigned int last_frame;
double voxel_size;
double voxel_size_half;
double voxel_size_inv;
double min_range;
double max_range;
double min_range_squared;
double max_range_squared;
std::string map_tf_frame;
std::string sensor_tf_frame;
std::string pointcloud_topic;
std::string save_map_topic;
std::string save_map_path;
bool enable_save_frame;
std::string save_frame_path;
ros::Subscriber pointcloud_sub;
ros::Subscriber save_map_sub;
tf2_ros::Buffer tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;
std::unordered_map<Eigen::Vector3i,std::pair<Label,Eigen::Vector3d>,IndexHash> static_voxels;

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& pointcloud)
{
    if(pointcloud->header.seq != last_frame + 1 && last_frame != 0)
        frame_skipped = true;
    
    if(frame_skipped)
        ROS_WARN("seq error");

    //转换为PCL点云
    pcl::PointCloud<evaluate_pcl::Point>::Ptr cloud_ptr(new pcl::PointCloud<evaluate_pcl::Point>());
    pcl::fromROSMsg(*pointcloud, *cloud_ptr);
    if(cloud_ptr->points.empty())
    {
        ROS_WARN("pointcloud empty");
        return;
    }

    ROS_INFO("lidar_transform:Pointcloud recieved,%zu points",cloud_ptr->points.size());

    try{
    //等待第一个点时刻的tf可用
    ros::Time transform_time = pointcloud->header.stamp;

    if(!tf_buffer.canTransform(map_tf_frame,sensor_tf_frame,transform_time,ros::Duration(10.0)))
    {
        ROS_WARN("no tf");
        return;
    }

    geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(map_tf_frame,sensor_tf_frame,pointcloud->header.stamp);

    Eigen::Isometry3d transform;
    Eigen::Vector3d point_pos_transformed;

    // 每帧保存点云
    pcl::PointCloud<pcl::PointXYZ> frame_pc;

    for(auto& point : *cloud_ptr)
    {
        assert(point.label == 9 || point.label == 251);
        Label label = static_cast<Label>(point.label);

        Eigen::Vector3d point_pos(point.x,point.y,point.z);

        transformfromTFToEigen(transformStamped,transform);
        point_pos_transformed = transform * point_pos;

        double range_squared = (point_pos_transformed - transform.translation()).squaredNorm();

        if( range_squared < min_range_squared || 
            range_squared > max_range_squared)
            continue;
        
        // 保存每帧点云
        if(enable_save_frame)
        {
            pcl::PointXYZ frame_point(point_pos_transformed.x(),point_pos_transformed.y(),point_pos_transformed.z());
            frame_pc.push_back(frame_point);
        }

        Eigen::Vector3i voxel_idx( std::floor(point_pos_transformed.x()*voxel_size_inv),
                            std::floor(point_pos_transformed.y()*voxel_size_inv),
                            std::floor(point_pos_transformed.z()*voxel_size_inv));
        
        auto it = static_voxels.find(voxel_idx);
        // voxel已经有标签，若为static则不需要处理，若为dynamic则在label为static时替换
        if(it != static_voxels.end())
        {
            if(it->second.first == LABEL_DYNAMIC && label == LABEL_STATIC)
            {
                static_voxels[voxel_idx].first = label;
                static_voxels[voxel_idx].second = Eigen::Vector3d(point_pos_transformed.x(),point_pos_transformed.y(),point_pos_transformed.z());
            }
        }
        // voxel还没有标签，直接填充
        else
        {
            static_voxels[voxel_idx].first = label;
            static_voxels[voxel_idx].second = Eigen::Vector3d(point_pos_transformed.x(),point_pos_transformed.y(),point_pos_transformed.z());
        }
    }

    // 保存每帧点云
    if(enable_save_frame)
    {
        frame_pc.sensor_origin_ = Eigen::Vector4f(transform.translation().x(), transform.translation().y(), transform.translation().z(), 1.0);
        pcl::io::savePCDFileASCII(save_frame_path + std::to_string(pointcloud->header.seq) + ".pcd", frame_pc);
    }

    }
    catch(tf2::TransformException &ex){
        ROS_WARN("%s", ex.what());
        return;
    }

    last_frame = pointcloud->header.seq;

    return;
}

void save_static_map_callback(const std_msgs::Empty::ConstPtr& msg)
{
    pcl::PointCloud<evaluate_pcl::Point> pointcloud_point;
    pcl::PointCloud<evaluate_pcl::Point> pointcloud_voxel;
    pointcloud_point.reserve(static_voxels.size());
    pointcloud_voxel.reserve(static_voxels.size());

    for(const auto& it : static_voxels)
    {
        evaluate_pcl::Point point;
        evaluate_pcl::Point voxel_point;

        point.x = it.second.second.x();
        point.y = it.second.second.y();
        point.z = it.second.second.z();
        point.label = static_cast<std::uint16_t>(it.second.first);

        voxel_point.x = it.first.x() * voxel_size + voxel_size_half;
        voxel_point.y = it.first.y() * voxel_size + voxel_size_half;
        voxel_point.z = it.first.z() * voxel_size + voxel_size_half;
        voxel_point.label = static_cast<std::uint16_t>(it.second.first);

        pointcloud_point.push_back(point);
        pointcloud_voxel.push_back(voxel_point);
    }

    pointcloud_point.width = pointcloud_point.points.size();
    pointcloud_point.height = 1;
    pcl::io::savePCDFileASCII(save_map_path + "ground_truth_point.pcd", pointcloud_point);

    pointcloud_voxel.width = pointcloud_voxel.points.size();
    pointcloud_voxel.height = 1;
    pcl::io::savePCDFileASCII(save_map_path + "ground_truth_voxel.pcd", pointcloud_voxel);

    ROS_INFO("Static map saved at:%s",save_map_path.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ground_truth_generate");
    ros::NodeHandle nh;

    if(!nh.param("voxel_size",voxel_size,0.4)) ROS_ERROR("param missing:voxel_size");
    if(!nh.param("min_range",min_range,2.7)) ROS_ERROR("param missing:min_range");
    if(!nh.param("max_range",max_range,1000.0)) ROS_ERROR("param missing:max_range");
    if(!nh.param("map_tf_frame",map_tf_frame,std::string(""))) ROS_ERROR("param missing:map_tf_frame");
    if(!nh.param("sensor_tf_frame",sensor_tf_frame,std::string(""))) ROS_ERROR("param missing:sensor_tf_frame");
    if(!nh.param("pointcloud_topic",pointcloud_topic,std::string(""))) ROS_ERROR("param missing:pointcloud_topic");
    if(!nh.param("save_map_topic",save_map_topic,std::string(""))) ROS_ERROR("param missing:save_map_topic");
    if(!nh.param("save_map_path",save_map_path,std::string(""))) ROS_ERROR("param missing:save_map_path");
    if(!nh.param("enable_save_frame",enable_save_frame,false)) ROS_ERROR("param missing:enable_save_frame");
    if(!nh.param("save_frame_path",save_frame_path,std::string(""))) ROS_ERROR("param missing:save_frame_path");

    min_range_squared = min_range * min_range;
    max_range_squared = max_range * max_range;

    voxel_size_half = voxel_size/2.0;
    voxel_size_inv = 1.0/voxel_size;

    frame_skipped = false;
    last_frame = 0;

    // 初始化tf_listener
    tf_listener.reset(new tf2_ros::TransformListener(tf_buffer));

    pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic,100,pointcloud_callback);
    save_map_sub = nh.subscribe(save_map_topic, 10, save_static_map_callback);

    ros::spin();
    return 0;
}