//Define the vehicle control class 
#ifndef VEHICLE_CONTROL__VEHICLE_CONTROL_HPP_
#define VEHICLE_CONTROL__VEHICLE_CONTROL_HPP_

#include <gazebo-11/gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

double WHEELBASE            = 2.65;     // distance between front and rear wheels 
double TRACK_WIDTH          = 1.638;    // distance between left and right wheels
double MAX_SPEED            = 10;       // maximum speed of the vehicle  
double MAX_STEERING_angle   = 0.52;     // maximum steering angle of the vehicle

namespace vehicle_control_gazebo {
    class VehiclePlugin : public gazebo::ModelPlugin { 
        public:
            VehiclePlugin(); //Constructor 
            virtual ~VehiclePlugin() = default; // Destructor
        protected:
            virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
            virtual void Reset();
        private:
            void OnUpdate(const gazebo::common::UpdateInfo& _info);
            void onDrive(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr); // Callback function for the ackermann drive topic
            void updateSteering(double);
            void updateSpeed(double);

            gazebo::event::ConnectionPtr update_connection_;
            
            gazebo::physics:: JointPtr front_left_wheel_joint;
            gazebo::physics:: JointPtr front_right_wheel_joint;
            gazebo::physics:: JointPtr rear_left_wheel_joint;
            gazebo::physics:: JointPtr rear_right_wheel_joint;

            gazebo::physics::JointPtr front_left_steer_joint;
            gazebo::physics::JointPtr front_right_steer_joint;

            gazebo::physics::LinkPtr footprint_link; 

            gazebo_ros::Node::SharedPtr node_; // ROS node handle

            rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_; // Subscriber to the ackermann drive topic

            // PID controllers for the steering and wheel joints
            gazebo::common::PID left_steering_pid;
            gazebo::common::PID right_steering_pid;

            gazebo::common::PID rear_right_wheel_pid;
            gazebo::common::PID rear_left_wheel_pid;
            
            gazebo::common::PID front_right_wheel_pid;
            gazebo::common::PID front_left_wheel_pid;            

            // Target values for the steering and wheel joints 
            double target_steering_angle;
            double target_speed;

            gazebo::common::Time last_update_time; // Time of the last update

            std::string drive_command_topic; // Name of the ackermann drive topic

    };

    GZ_REGISTER_MODEL_PLUGIN(VehiclePlugin)
}

#endif // VEHICLE_CONTROL__VEHICLE_CONTROL_HPP_VehiclePlugin