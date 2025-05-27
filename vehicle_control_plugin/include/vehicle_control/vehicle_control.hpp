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
#include <sensor_msgs/msg/joint_state.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>


#include <rclcpp/rclcpp.hpp>

const double WHEELBASE            = 1.686;     // distance between front and rear wheels 
const double TRACK_WIDTH          = 1.08;    // distance between left and right wheels
const double MAX_SPEED            = 10;       // maximum speed of the vehicle  
const double MAX_STEERING_angle   = 0.648228;     // maximum steering angle of the vehicle 
const double MAX_TORQUE           = 100;

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
            void getOdom();
            void imu_callback(const sensor_msgs::msg::Imu::SharedPtr);

            gazebo::event::ConnectionPtr update_connection_;
            
            gazebo::physics:: JointPtr front_left_wheel_joint;
            gazebo::physics:: JointPtr front_right_wheel_joint;
            gazebo::physics:: JointPtr rear_left_wheel_joint;
            gazebo::physics:: JointPtr rear_right_wheel_joint;

            gazebo::physics::JointPtr front_left_steer_joint;
            gazebo::physics::JointPtr front_right_steer_joint;

            gazebo::physics::LinkPtr footprint_link; 
            gazebo::physics::LinkPtr base_link; 

            gazebo_ros::Node::SharedPtr node_; // ROS node handle

            //timer
            rclcpp::TimerBase::SharedPtr timer_;

            rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_; // Subscriber to the ackermann drive topic
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub; 

            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub; //odom publisher
            rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub; //velocity publisher
            std::array<double, 2> diff_joint_positions_;

            // PID controllers for the steering and wheel joints
            gazebo::common::PID left_steering_pid;
            gazebo::common::PID right_steering_pid;

            gazebo::common::PID rear_right_wheel_pid;
            gazebo::common::PID rear_left_wheel_pid;
            
            gazebo::common::PID front_right_wheel_pid;
            gazebo::common::PID front_left_wheel_pid;   
            
            gazebo::common::PID angular_vel_pid;
            double p;
            double i;
            double d;
            double cur_ang_vel;
            // Target values for the steering and wheel joints 
            double target_steering_angle;
            double target_speed;
            // double torque;
            double chassis_aero_force_gain_ = 0.63045;

            gazebo::common::Time last_update_time; // Time of the last update

            std::string drive_command_topic; // Name of the ackermann drive topic

    };

    GZ_REGISTER_MODEL_PLUGIN(VehiclePlugin)
}

#endif // VEHICLE_CONTROL__VEHICLE_CONTROL_HPP_VehiclePlugin