#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "bounding_box_msg/msg/bounding_box_array.hpp"
#include "bounding_box_msg/srv/bounding_box_service.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace gazebo
{
    class BoundingBoxPlugin : public WorldPlugin
    {
    public:
        BoundingBoxPlugin() : WorldPlugin() {}

        virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
            // Initialize ROS node
            node_ = gazebo_ros::Node::Get(_sdf);
            world_ = _world;

            // Create a ROS 2 publisher for bounding box markers (for RViz)
            pub_marker_ = node_->create_publisher<visualization_msgs::msg::Marker>("bounding_box_marker", 10);

            // Create a TF broadcaster for each model
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

            // Set update rate
            update_rate_ = _sdf->HasElement("update_rate") ? _sdf->Get<double>("update_rate") : 1.0;

            // Get frame_id from the SDF
            frame_id_ = _sdf->HasElement("frame_id") ? _sdf->Get<std::string>("frame_id") : "map";

            // Connect to the world update event
            update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&BoundingBoxPlugin::OnUpdate, this));

            // Subscribe to the Velodyne point cloud topic
            sub_velodyne_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/velodyne_points", 10, std::bind(&BoundingBoxPlugin::OnVelodynePoints, this, std::placeholders::_1));

            last_update_time_ = _world->SimTime().Double();
        }

    private:
        void OnVelodynePoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            pcl::fromROSMsg(*msg, pcl_cloud);

            // Clear the list of models with points inside their bounding boxes
            models_with_points_.clear();

            for (auto model : world_->Models())
            {
                if (!model) continue;

                ignition::math::AxisAlignedBox bbox = model->BoundingBox();

                // Check if any point lies within the bounding box
                for (const auto& point : pcl_cloud.points)
                {
                    if (bbox.Contains(ignition::math::Vector3d(point.x, point.y, point.z)))
                    {
                        models_with_points_.insert(model->GetName());
                        break; // If at least one point is inside, no need to check further
                    }
                }
            }
        }

        void OnUpdate()
        {
            double current_time = world_->SimTime().Double();
            if (current_time - last_update_time_ < (1.0 / update_rate_)) return;

            int id = 0;
            float color_step = 0.1f;

            for (auto model : world_->Models())
            {
                if (!model) continue;

                std::string model_name = model->GetName();

                // Only process and publish bounding boxes for models with points in their bounding box
                if (models_with_points_.find(model_name) == models_with_points_.end()) continue;

                ignition::math::AxisAlignedBox bbox = model->BoundingBox();

                // Publish bounding box marker for visualization
                visualization_msgs::msg::Marker marker;
                marker.header.stamp = node_->now();
                marker.header.frame_id = frame_id_;
                marker.ns = "bounding_box";
                marker.id = id++;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose.position.x = bbox.Center().X();
                marker.pose.position.y = bbox.Center().Y();
                marker.pose.position.z = bbox.Center().Z();
                marker.scale.x = bbox.XLength();
                marker.scale.y = bbox.YLength();
                marker.scale.z = bbox.ZLength();

                // Assign different colors
                marker.color.r = fmod(color_step * id, 1.0f);
                marker.color.g = fmod(0.5f + color_step * (id + 1), 1.0f);
                marker.color.b = fmod(0.3f + color_step * (id + 2), 1.0f);
                marker.color.a = 0.5f;
                pub_marker_->publish(marker);

                // Publish the TF
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = node_->now();
                transformStamped.header.frame_id = frame_id_;
                transformStamped.child_frame_id = model_name + "_bounding_box";
                transformStamped.transform.translation.x = bbox.Center().X();
                transformStamped.transform.translation.y = bbox.Center().Y();
                transformStamped.transform.translation.z = bbox.Center().Z();
                transformStamped.transform.rotation.w = 1.0;
                tf_broadcaster_->sendTransform(transformStamped);
            }

            last_update_time_ = current_time;
        }

        // Class members
        physics::WorldPtr world_;
        event::ConnectionPtr update_connection_;
        gazebo_ros::Node::SharedPtr node_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_velodyne_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unordered_set<std::string> models_with_points_; // Set to store models with points in bounding box
        std::string frame_id_;
        double last_update_time_;
        double update_rate_;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(BoundingBoxPlugin)
}
