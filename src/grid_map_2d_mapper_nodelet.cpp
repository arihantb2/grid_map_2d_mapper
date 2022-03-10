/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Stefan Kohlbrecher, TU Darmstadt
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Stefan Kohlbrecher
 */

#include <grid_map_2d_mapper/grid_map_2d_mapper_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_msgs/GridMap.h>

#include <er_nav_interface/map_io/map_io.h>
#include <er_nav_interface/frame_collection/frame_collection.h>

#include <chrono>
#include <thread>

// TODO: Remove after testing
// #include <tf2_ros/transform_broadcaster.h>

namespace grid_map_2d_mapper
{
GridMap2DMapperNodelet::GridMap2DMapperNodelet()
{
}

void GridMap2DMapperNodelet::onInit()
{
    boost::mutex::scoped_lock lock(connect_mutex_);
    private_nh_ = getPrivateNodeHandle();

    private_nh_.param<std::string>("target_frame", target_frame_, "");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");

    private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
    private_nh_.param<double>("min_height", min_height_, -0.3);
    private_nh_.param<double>("max_height", max_height_, 1.3);
    private_nh_.param<double>("angle_min", angle_min_, -M_PI / 1.0);
    private_nh_.param<double>("angle_max", angle_max_, M_PI / 1.0);
    private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 180.0);
    private_nh_.param<double>("scan_time", scan_time_, 0.0);
    private_nh_.param<double>("range_min", range_min_, 0.45);
    private_nh_.param<double>("range_max", range_max_, 15.0);
    private_nh_.param<bool>("no_mapping", no_mapping_, false);

    int concurrency_level;
    private_nh_.param<int>("concurrency_level", concurrency_level, 1);
    private_nh_.param<bool>("use_inf", use_inf_, true);

    bool offline_mapping = private_nh_.param<bool>("offline_mapping", false);

    grid_map_.add("occupancy_log_odds");
    grid_map_.add("occupancy_prob");
    grid_map_.setGeometry(grid_map::Length(2.0, 2.0), 0.05);
    grid_map_.setFrameId(map_frame_);

    log_odds_free_ = probToLogOdds(0.4);
    log_odds_occ_ = probToLogOdds(0.75);

    min_log_odds_ = log_odds_free_ * 20;
    max_log_odds_ = log_odds_occ_ * 20;
    ROS_INFO("log odds free: %f log odds occ: %f", log_odds_free_, log_odds_occ_);

    obstacle_scan_pub_ = private_nh_.advertise<sensor_msgs::LaserScan>("obstacle_scan", 10, false);
    occupancy_map_pub_ = private_nh_.advertise<nav_msgs::OccupancyGrid>("map", 10, false);
    map_throttled_pub_ = private_nh_.advertise<nav_msgs::OccupancyGrid>("map_throttled", 10, false);
    grid_map_pub_ = private_nh_.advertise<grid_map_msgs::GridMap>("grid_map", 10, false);

    map_service_ = private_nh_.advertiseService("map", &GridMap2DMapperNodelet::mapServiceCallback, this);

    if (offline_mapping)
    {
        // tf2_ros::TransformBroadcaster tf_broadcaster;

        ros::Publisher final_map_pub = private_nh_.advertise<nav_msgs::OccupancyGrid>("final_map", 1, true);
        // ros::Publisher cloud_pub = private_nh_.advertise<sensor_msgs::PointCloud2>("input_cloud", 10, false);
        // ros::Publisher tf_cloud_pub = private_nh_.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 10, false);

        // Load map from maps folder
        std::string maps_folder = private_nh_.param<std::string>("map_folder", "/home/arihant/.ros/maps");
        std::string map_name = private_nh_.param<std::string>("map_name", "MAP_NAME_NOT_SET");
        double loop_duration_ms = private_nh_.param<double>("loop_duration_ms", 100.0);

        er_nav_interface::map_io::MapIO map_io(std::make_shared<er_nav_interface::map_io::MapIO::Params>(maps_folder, "exr2", 0.1, 0.1));
        er_nav_interface::map_container::FrameCollection map_frames = map_io.load(map_name);

        std::vector<er_nav_interface::map_container::Frame> frames = map_frames.frames();

        Eigen::Affine3f lidar_to_baselink_tf = Eigen::Affine3f::Identity();

        const size_t num_frames = frames.size();
        std::cout << "[" << num_frames << "] keyframes to process...\n";
        for (size_t i = 0; i < num_frames; ++i)
        {
            if (i == 0)
            {
                lidar_to_baselink_tf = frames[0].pose.inverse();
            }

            Eigen::Affine3f world_to_lidar_pose = frames[i].pose;
            Eigen::Affine3f world_to_baselink_pose = world_to_lidar_pose * lidar_to_baselink_tf;

            // geometry_msgs::TransformStamped world_to_baselink = tf2::eigenToTransform(world_to_baselink_pose.cast<double>());
            // world_to_baselink.header.frame_id = "map";
            // world_to_baselink.child_frame_id = "base_link";
            // world_to_baselink.header.stamp = ros::Time::now();
            // tf_broadcaster.sendTransform(world_to_baselink);

            // geometry_msgs::TransformStamped baselink_to_lidar = tf2::eigenToTransform(lidar_to_baselink_tf.inverse().cast<double>());
            // baselink_to_lidar.header.frame_id = "base_link";
            // baselink_to_lidar.child_frame_id = "velodyne_frame";
            // baselink_to_lidar.header.stamp = ros::Time::now();
            // tf_broadcaster.sendTransform(baselink_to_lidar);

            // sensor_msgs::PointCloud2 cloud_input;
            // pcl::toROSMsg(*(frames[i].cloud), cloud_input);
            // cloud_input.header.stamp = ros::Time::now();
            // cloud_input.header.frame_id = "velodyne_frame";
            // cloud_pub.publish(cloud_input);

            std::cout << "Processing frame [" << i + 1 << " / " << num_frames << "] ...";
            auto start_time = std::chrono::high_resolution_clock::now();

            auto cloud_in_baselink_frame = er_nav_interface::utility::transformPointCloud(*(frames[i].cloud), lidar_to_baselink_tf.inverse());

            sensor_msgs::PointCloud2 transformed_cloud;
            pcl::toROSMsg(cloud_in_baselink_frame, transformed_cloud);
            transformed_cloud.header.stamp = ros::Time::now();
            transformed_cloud.header.frame_id = "base_link";
            // tf_cloud_pub.publish(transformed_cloud);

            processCloud(transformed_cloud, world_to_baselink_pose.cast<double>());

            std::chrono::duration<double, std::milli> time_elapsed = std::chrono::high_resolution_clock::now() - start_time;
            std::cout << " finished in [" << time_elapsed.count() << " ms]\n";

            if (time_elapsed.count() < loop_duration_ms)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(int(loop_duration_ms - time_elapsed.count())));
            }
        }

        nav_msgs::OccupancyGrid occ_prob_grid_msg;
        grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_, "occupancy_prob", 0.0, 1.0, occ_prob_grid_msg);
        final_map_pub.publish(occ_prob_grid_msg);

        return;
    }

    dyn_rec_server_.reset(new ReconfigureServer(config_mutex_, private_nh_));
    dyn_rec_server_->setCallback(boost::bind(&GridMap2DMapperNodelet::reconfigureCallback, this, _1, _2));

    // Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
    if (concurrency_level == 1)
    {
        nh_ = getNodeHandle();
    }
    else
    {
        nh_ = getMTNodeHandle();
    }

    // Only queue one pointcloud per running thread
    if (concurrency_level > 0)
    {
        input_queue_size_ = concurrency_level;
    }
    else
    {
        input_queue_size_ = boost::thread::hardware_concurrency();
    }

    // if pointcloud target frame specified, we need to filter by transform availability
    if (!target_frame_.empty())
    {
        tf2_.reset(new tf2_ros::Buffer());
        tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
        message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, 20, nh_));
        message_filter_->registerCallback(boost::bind(&GridMap2DMapperNodelet::cloudCb, this, _1));
        message_filter_->registerFailureCallback(boost::bind(&GridMap2DMapperNodelet::failureCb, this, _1, _2));
    }
    else  // otherwise setup direct subscription
    {
        tf2_.reset(new tf2_ros::Buffer());
        tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
        sub_.registerCallback(boost::bind(&GridMap2DMapperNodelet::cloudCb, this, _1));
    }

    syscommand_subscriber_ = private_nh_.subscribe("syscommand", 10, &GridMap2DMapperNodelet::syscommandCallback, this);
    sub_.subscribe(private_nh_, "input_points", input_queue_size_);

    double map_publish_period = private_nh_.param("map_publish_period", 1.0);
    map_throttled_pub_timer_ = nh_.createTimer(ros::Duration(map_publish_period), &GridMap2DMapperNodelet::mapThrottledPubTimer, this);
}

void GridMap2DMapperNodelet::connectCb()
{
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (occupancy_map_pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
    {
        NODELET_INFO("Got a subscriber to map, starting subscriber to pointcloud");
        sub_.subscribe(nh_, "/scan_matched_points2", input_queue_size_);
    }
}

void GridMap2DMapperNodelet::disconnectCb()
{
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (occupancy_map_pub_.getNumSubscribers() == 0)
    {
        NODELET_INFO("No subscribers to scan, shutting down subscriber to pointcloud");
        sub_.unsubscribe();
    }
}

void GridMap2DMapperNodelet::syscommandCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data == "reset" || msg->data == "reset_2d_map")
    {
        grid_map_.clear("occupancy_log_odds");
        grid_map_.clear("occupancy_prob");
        ROS_INFO("Cleared grid_map_2d_mapper map!");
    }
}

void GridMap2DMapperNodelet::reconfigureCallback(grid_map_2d_mapper::GridMap2DMapperConfig& config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %f %f", config.min_height, config.max_height);

    min_height_ = config.min_height;
    max_height_ = config.max_height;
    no_mapping_ = !config.mapping_active;
}

void GridMap2DMapperNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
    NODELET_WARN_STREAM_THROTTLE(1.0,
                                 "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to " << message_filter_->getTargetFramesString());
}

void GridMap2DMapperNodelet::mapThrottledPubTimer(const ros::TimerEvent& event)
{
    if (map_throttled_pub_.getNumSubscribers() > 0)
    {
        grid_map::Matrix& grid_data = grid_map_["occupancy_log_odds"];

        grid_map::Matrix& grid_data_prob = grid_map_["occupancy_prob"];

        size_t total_size = grid_data.rows() * grid_data.cols();
        for (size_t i = 0; i < total_size; ++i)
        {
            const float& cell = grid_data.data()[i];

            if (cell != cell)
            {
                grid_data_prob.data()[i] = cell;
            }
            else if (cell < 0.0)
            {
                grid_data_prob.data()[i] = 0.0;
            }
            else
            {
                grid_data_prob.data()[i] = 1.0;
            }
        }

        nav_msgs::OccupancyGrid occ_grid_msg;

        grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_, "occupancy_prob", 0.0, 1.0, occ_grid_msg);
        map_throttled_pub_.publish(occ_grid_msg);
    }
}

bool GridMap2DMapperNodelet::mapServiceCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res)
{
    ROS_INFO("grid_mapper_2d map service called");

    grid_map::Matrix& grid_data = grid_map_["occupancy_log_odds"];

    grid_map::Matrix& grid_data_prob = grid_map_["occupancy_prob"];

    size_t total_size = grid_data.rows() * grid_data.cols();
    for (size_t i = 0; i < total_size; ++i)
    {
        const float& cell = grid_data.data()[i];

        if (cell != cell)
        {
            grid_data_prob.data()[i] = cell;
        }
        else if (cell < 0.0)
        {
            grid_data_prob.data()[i] = 0.0;
        }
        else
        {
            grid_data_prob.data()[i] = 1.0;
        }
    }

    grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_, "occupancy_prob", 0.0, 1.0, res.map);

    return true;
}

void GridMap2DMapperNodelet::processCloud(const sensor_msgs::PointCloud2& cloud_in_baselink_frame, const Eigen::Affine3d& world_to_baselink_tf)
{
    /**
     *
     * Copy "ground plane" points from input cloud to laser scan object
     *
     */

    // build laserscan output
    sensor_msgs::LaserScanPtr output;
    output = boost::make_shared<sensor_msgs::LaserScan>();

    output->header = cloud_in_baselink_frame.header;
    output->angle_min = angle_min_;
    output->angle_max = angle_max_;
    output->angle_increment = angle_increment_;
    output->time_increment = 0.0;
    output->scan_time = scan_time_;
    output->range_min = range_min_;
    output->range_max = range_max_;

    // determine amount of rays to create
    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);

    // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    if (use_inf_)
    {
        output->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
    }
    else
    {
        output->ranges.assign(ranges_size, output->range_max + 1.0);
    }

    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in_baselink_frame, "x"), iter_y(cloud_in_baselink_frame, "y"),
         iter_z(cloud_in_baselink_frame, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);

        if (std::isnan(point.x()) || std::isnan(point.y()) || std::isnan(point.z()))
        {
            NODELET_DEBUG("rejected for nan in point (%f, %f, %f)\n", point.x(), point.y(), point.z());
            continue;
        }

        if (point.z() > max_height_ || point.z() < min_height_)
        {
            NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", point.z(), min_height_, max_height_);
            continue;
        }

        double range = hypot(point.x(), point.y());
        if (range < range_min_)
        {
            NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, point.x(), point.y(), point.z());
            continue;
        }

        double angle = atan2(point.y(), point.x());
        if (angle < output->angle_min || angle > output->angle_max)
        {
            NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
            continue;
        }

        // overwrite range at laserscan ray if new range is smaller
        int index = (angle - output->angle_min) / output->angle_increment;
        if (range < output->ranges[index])
        {
            output->ranges[index] = range;
        }
    }

    if (obstacle_scan_pub_.getNumSubscribers() > 0)
    {
        sensor_msgs::LaserScan output_bl = *output;
        output_bl.header.frame_id = "base_link";
        obstacle_scan_pub_.publish(output_bl);
    }

    // Cloud reduced contains all cloud points within (min_height, max_height)
    sensor_msgs::PointCloud2 cloud_reduced;
    projector_.projectLaser(*output, cloud_reduced);

    if (no_mapping_)
        return;

    Eigen::Vector3d sensor_frame_world_pos(world_to_baselink_tf.translation());

    /**
     *
     * Compute grid map dimensions and center
     * Update grid map size
     * Update grid map occupancy
     *
     */

    grid_map::Matrix& grid_data = grid_map_["occupancy_log_odds"];

    grid_map::Position sensor_position(sensor_frame_world_pos.x(), sensor_frame_world_pos.y());

    end_points_.clear();

    Eigen::Vector2d min_coords(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    Eigen::Vector2d max_coords(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());

    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_reduced, "x"), iter_y(cloud_reduced, "y"), iter_z(cloud_reduced, "z");
         iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        Eigen::Vector3d point_in_baselink_frame(*iter_x, *iter_y, *iter_z);
        Eigen::Vector3d point_in_world_frame = world_to_baselink_tf * point_in_baselink_frame;

        if (max_coords.x() < point_in_world_frame.x())
            max_coords.x() = point_in_world_frame.x();
        if (max_coords.y() < point_in_world_frame.y())
            max_coords.y() = point_in_world_frame.y();
        if (min_coords.x() > point_in_world_frame.x())
            min_coords.x() = point_in_world_frame.x();
        if (min_coords.y() > point_in_world_frame.y())
            min_coords.y() = point_in_world_frame.y();

        end_points_.push_back(point_in_world_frame);
    }

    Eigen::Vector2d center((min_coords + max_coords) * 0.5);
    Eigen::Vector2d lengths(max_coords - min_coords);

    grid_map::GridMap update_area;
    update_area.setGeometry(lengths.array() + 0.5, grid_map_.getResolution(), center);
    grid_map_.extendToInclude(update_area);

    size_t end_points_size = end_points_.size();

    std::vector<grid_map::Index> curr_ray;

    for (size_t i = 0; i < end_points_size; ++i)
    {
        const Eigen::Vector3d& end_point(end_points_[i]);
        grid_map::Position end_point_position(end_point.x(), end_point.y());

        curr_ray.clear();

        for (grid_map::LineIterator iterator(grid_map_, sensor_position, end_point_position); !iterator.isPastEnd(); ++iterator)
        {
            curr_ray.push_back(grid_map::Index(*iterator));
        }

        size_t curr_ray_size = curr_ray.size();

        if (curr_ray_size > 2)
        {
            // All points until last point on line are free
            size_t r = 0;
            for (; r < curr_ray_size - 1; ++r)
            {
                const grid_map::Index& index = curr_ray[r];

                if (std::isnan(grid_data(index(0), index(1))))
                    grid_data(index(0), index(1)) = 0.0;

                if (min_log_odds_ < grid_data(index(0), index(1)))
                    grid_data(index(0), index(1)) += log_odds_free_;
            }

            // Last point in line is occupied
            const grid_map::Index& index = curr_ray[r];

            if (std::isnan(grid_data(index(0), index(1))))
                grid_data(index(0), index(1)) = 0.0;

            if (max_log_odds_ > grid_data(index(0), index(1)))
                grid_data(index(0), index(1)) += log_odds_occ_;
        }
    }

    if (grid_map_pub_.getNumSubscribers() > 0)
    {
        grid_map_msgs::GridMap grid_map_msg;
        grid_map::GridMapRosConverter::toMessage(grid_map_, grid_map_msg);
        grid_map_pub_.publish(grid_map_msg);
    }

    if (occupancy_map_pub_.getNumSubscribers() > 0)
    {
        grid_map::Matrix& grid_data_prob = grid_map_["occupancy_prob"];

        size_t total_size = grid_data.rows() * grid_data.cols();
        for (size_t i = 0; i < total_size; ++i)
        {
            const float& cell = grid_data.data()[i];

            if (cell != cell)
            {
                grid_data_prob.data()[i] = cell;
            }
            else if (cell < 0.0)
            {
                grid_data_prob.data()[i] = 0.0;
            }
            else
            {
                grid_data_prob.data()[i] = 1.0;
            }
        }

        nav_msgs::OccupancyGrid occ_grid_msg;

        grid_map::GridMapRosConverter::toOccupancyGrid(grid_map_, "occupancy_prob", 0.0, 1.0, occ_grid_msg);
        occupancy_map_pub_.publish(occ_grid_msg);
    }
}

void GridMap2DMapperNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    /**
     *
     * Get current world to sensor transform from TF tree
     *
     */

    std::string target_frame;
    if (target_frame_.empty())
    {
        target_frame = cloud_msg->header.frame_id;
    }
    else
    {
        target_frame = target_frame_;
    }

    // Transform cloud if necessary
    sensor_msgs::PointCloud2ConstPtr cloud_in_baselink_frame;

    if (target_frame != cloud_msg->header.frame_id)
    {
        try
        {
            sensor_msgs::PointCloud2Ptr cloud;
            cloud.reset(new sensor_msgs::PointCloud2);
            tf2_->transform(*cloud_msg, *cloud, target_frame_, ros::Duration(tolerance_));
            cloud_in_baselink_frame = cloud;
        }
        catch (tf2::TransformException ex)
        {
            NODELET_ERROR_STREAM("Transform failure: " << ex.what());
            return;
        }
    }
    else
    {
        cloud_in_baselink_frame = cloud_msg;
    }

    ros::Time stamp = cloud_msg->header.stamp;
    geometry_msgs::TransformStamped world_to_baselink_tf;

    try
    {
        world_to_baselink_tf = tf2_->lookupTransform(map_frame_, target_frame, stamp, ros::Duration(1.0));
    }
    catch (tf2::TransformException& ex)
    {
        NODELET_WARN("Cannot lookup transform, skipping map update!: %s", ex.what());
        return;
    }

    Eigen::Affine3d to_world_eigen = tf2::transformToEigen(world_to_baselink_tf);
    processCloud(*cloud_in_baselink_frame, to_world_eigen);
}

float GridMap2DMapperNodelet::probToLogOdds(float prob)
{
    float odds = prob / (1.0f - prob);
    return log(odds);
}

}  // namespace grid_map_2d_mapper

PLUGINLIB_EXPORT_CLASS(grid_map_2d_mapper::GridMap2DMapperNodelet, nodelet::Nodelet);
