#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>  // For publishing TF
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace gazebo
{
    class BoundingBoxPlugin : public ModelPlugin
    {
    public:
        BoundingBoxPlugin() : ModelPlugin() {}

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            // Initialize ROS node
            node_ = gazebo_ros::Node::Get(_sdf);

            model_ = _model;

            // Create a ROS 2 publisher to publish bounding box marker
            pub_marker_ = node_->create_publisher<visualization_msgs::msg::Marker>("bounding_box_marker", 10);

            // Create a TransformBroadcaster for publishing TF
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

            // Set update rate if specified
            if (_sdf->HasElement("update_rate"))
            {
                update_rate_ = _sdf->Get<double>("update_rate");
            }
            else
            {
                update_rate_ = 1.0; // Default 1 Hz
            }

            // Parse frame_id from the SDF
            if (_sdf->HasElement("frame_id"))
            {
                frame_id_ = _sdf->Get<std::string>("frame_id");
            }
            else
            {
                frame_id_ = "map";  // Default to "map"
            }

            // Listen to the update event
            update_connection_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&BoundingBoxPlugin::OnUpdate, this));

            last_update_time_ = _model->GetWorld()->SimTime().Double();
        }

    private:
        void OnUpdate()
        {
            // Get current simulation time
            double current_time = model_->GetWorld()->SimTime().Double();

            // Check if it's time to update based on the update rate
            if (current_time - last_update_time_ < (1.0 / update_rate_))
            {
                return;
            }

            // Get bounding box
            ignition::math::AxisAlignedBox bbox = model_->BoundingBox();

            // Create and publish a marker for the bounding box
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = node_->now();
            marker.header.frame_id = frame_id_;  // Set frame of reference from SDF

            marker.ns = "bounding_box";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::CUBE;  // Use CUBE to represent the box
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Set the pose of the marker (center of the bounding box)
            marker.pose.position.x = bbox.Center().X();
            marker.pose.position.y = bbox.Center().Y();
            marker.pose.position.z = bbox.Center().Z();
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Set the size of the bounding box
            marker.scale.x = bbox.XLength();  // Width
            marker.scale.y = bbox.YLength();  // Depth
            marker.scale.z = bbox.ZLength();  // Height

            // Set color and transparency of the marker (optional)
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;  // Semi-transparent

            // Publish the marker
            pub_marker_->publish(marker);

            // Publish the TF for the bounding box relative to the specified frame
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = node_->now();
            transformStamped.header.frame_id = frame_id_;
            transformStamped.child_frame_id = "bounding_box_frame";
            transformStamped.transform.translation.x = bbox.Center().X();
            transformStamped.transform.translation.y = bbox.Center().Y();
            transformStamped.transform.translation.z = bbox.Center().Z();
            transformStamped.transform.rotation.x = 0.0;
            transformStamped.transform.rotation.y = 0.0;
            transformStamped.transform.rotation.z = 0.0;
            transformStamped.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(transformStamped);

            // Update the last update time
            last_update_time_ = current_time;
        }

        physics::ModelPtr model_;
        event::ConnectionPtr update_connection_;
        gazebo_ros::Node::SharedPtr node_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  // TF broadcaster
        std::string frame_id_;  // Frame ID parsed from SDF
        double last_update_time_;
        double update_rate_;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(BoundingBoxPlugin)
}
