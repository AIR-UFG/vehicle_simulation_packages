#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "bounding_box_plugin/msg/bounding_box_array.hpp"
#include "bounding_box_plugin/srv/bounding_box_service.hpp"

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
            model_name_ = model_->GetName();  // Automatically get model name

            // Create a ROS 2 publisher for bounding box markers (for RViz)
            pub_marker_ = node_->create_publisher<visualization_msgs::msg::Marker>("bounding_box_marker", 10);

            // Create a TF broadcaster for each model
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

            // Create a service to return bounding boxes
            bbox_service_ = node_->create_service<bounding_box_plugin::srv::BoundingBoxService>(
                "get_bounding_boxes", std::bind(&BoundingBoxPlugin::HandleBoundingBoxService, this, std::placeholders::_1, std::placeholders::_2));

            // Set update rate
            update_rate_ = _sdf->HasElement("update_rate") ? _sdf->Get<double>("update_rate") : 1.0;

            // Get frame_id and base_link from the SDF
            frame_id_ = _sdf->HasElement("frame_id") ? _sdf->Get<std::string>("frame_id") : "map";
            base_link_frame_ = _sdf->HasElement("base_link_frame") ? _sdf->Get<std::string>("base_link_frame") : "base_link";

            // Connect to the world update event
            update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&BoundingBoxPlugin::OnUpdate, this));

            last_update_time_ = _model->GetWorld()->SimTime().Double();
        }

    private:
        void OnUpdate()
        {
            double current_time = model_->GetWorld()->SimTime().Double();
            if (current_time - last_update_time_ < (1.0 / update_rate_)) return;

            // Get bounding box
            ignition::math::AxisAlignedBox bbox = model_->BoundingBox();

            // Publish marker (for RViz on demand)
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = node_->now();
            marker.header.frame_id = frame_id_;
            marker.ns = "bounding_box";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = bbox.Center().X();
            marker.pose.position.y = bbox.Center().Y();
            marker.pose.position.z = bbox.Center().Z();
            marker.scale.x = bbox.XLength();
            marker.scale.y = bbox.YLength();
            marker.scale.z = bbox.ZLength();
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f;
            pub_marker_->publish(marker);

            // Publish the TF
            geometry_msgs::msg::TransformStamped transformStamped;
            transformStamped.header.stamp = node_->now();
            transformStamped.header.frame_id = frame_id_;
            transformStamped.child_frame_id = model_name_ + "_bounding_box";
            transformStamped.transform.translation.x = bbox.Center().X();
            transformStamped.transform.translation.y = bbox.Center().Y();
            transformStamped.transform.translation.z = bbox.Center().Z();
            transformStamped.transform.rotation.w = 1.0;
            tf_broadcaster_->sendTransform(transformStamped);

            last_update_time_ = current_time;
        }

        void HandleBoundingBoxService(
            const std::shared_ptr<bounding_box_plugin::srv::BoundingBoxService::Request> request,
            std::shared_ptr<bounding_box_plugin::srv::BoundingBoxService::Response> response)
        {
            // Get bounding box information
            ignition::math::AxisAlignedBox bbox = model_->BoundingBox();

            bounding_box_plugin::msg::BoundingBox box;
            box.model_name = model_name_;
            box.pose.position.x = bbox.Center().X();
            box.pose.position.y = bbox.Center().Y();
            box.pose.position.z = bbox.Center().Z();
            box.dimensions.x = bbox.XLength();
            box.dimensions.y = bbox.YLength();
            box.dimensions.z = bbox.ZLength();

            // Create a BoundingBoxArray message and assign the bounding box to it
            bounding_box_plugin::msg::BoundingBoxArray bbox_array;
            bbox_array.boxes.push_back(box);  // Add the bounding box to the array

            // Assign the BoundingBoxArray to the response
            response->boxes = bbox_array;
        }



        // Class members
        physics::ModelPtr model_;
        event::ConnectionPtr update_connection_;
        gazebo_ros::Node::SharedPtr node_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Service<bounding_box_plugin::srv::BoundingBoxService>::SharedPtr bbox_service_;
        std::string frame_id_;
        std::string base_link_frame_;
        std::string model_name_;
        double last_update_time_;
        double update_rate_;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(BoundingBoxPlugin)
}
