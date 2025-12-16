#include <vehicle_control/vehicle_control.hpp>

#include <iostream>

using namespace std::chrono_literals;

namespace vehicle_control_gazebo {

    VehiclePlugin::VehiclePlugin(){

        target_steering_angle = 0.0;
        target_speed = 0.0;

        left_steering_pid = gazebo::common::PID(2e3, 0.0, 3e2);
        left_steering_pid.SetCmdMin(-5000.0);
        left_steering_pid.SetCmdMax(5000.0);

        right_steering_pid = gazebo::common::PID(2e3, 0.0, 3e2);
        right_steering_pid.SetCmdMin(-5000.0);
        right_steering_pid.SetCmdMax(5000.0);
        
        angular_vel_pid = gazebo::common::PID(4.0, 12.0, 0.0, 0.5, -0.5);
        angular_vel_pid.SetCmdMin(-MAX_STEERING_angle);
        angular_vel_pid.SetCmdMax(MAX_STEERING_angle);

    }

    void VehiclePlugin::Reset() {
        // Empty function body
    }

    void VehiclePlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf){

        // Get the joints from the model
        front_left_wheel_joint = model->GetJoint("front_left_wheel_joint");
        front_right_wheel_joint = model->GetJoint("front_right_wheel_joint");
        rear_left_wheel_joint = model->GetJoint("rear_left_wheel_joint");
        rear_right_wheel_joint = model->GetJoint("rear_right_wheel_joint");

        front_left_steer_joint = model->GetJoint("front_left_steer_joint");
        front_right_steer_joint = model->GetJoint("front_right_steer_joint");

        footprint_link = model->GetLink("footprint_link");
        base_link = model->GetLink("base_link");

        // Connect to the update event
        update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&VehiclePlugin::OnUpdate, this, std::placeholders::_1)
        );

        // ROS initialization
        node_ = gazebo_ros::Node::Get(sdf);

        node_->declare_parameter<std::string>("drive_command_topic", "/sd_control/cmd_vel");
        node_->get_parameter("drive_command_topic", drive_command_topic);

        node_->declare_parameter<double>("p", 4.0);
        node_->get_parameter("p", p);

        node_->declare_parameter<double>("i", 12.0);
        node_->get_parameter("i", i);

        node_->declare_parameter<double>("d", 0.0);
        node_->get_parameter("d", d);


        // Create a subscriber to the ackermann drive topic
        drive_sub_ = node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_command_topic, 1, std::bind(&VehiclePlugin::onDrive, this, std::placeholders::_1)
        );

        //create imu sub
        imu_sub = node_->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_raw", 10,
            std::bind(&VehiclePlugin::imu_callback, this, std::placeholders::_1)
          );
        
        //Create a publisher to odom topic -- leaving the odometry to RTAB 
        // odom_pub = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);
        vel_pub = node_->create_publisher<geometry_msgs::msg::TwistStamped>("/current_velocity", 10);

        // Get timer
        // timer_ = node_->create_wall_timer(
        //     50ms, std::bind(&VehiclePlugin::getOdom, this)
        // );

        //std::cout << "drive_commmand_topic" << drive_command_topic << std::endl;
    }

    void VehiclePlugin::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu){
        const double alpha = 0.6;
        static double filtered_w = 0.0;

        filtered_w = alpha * imu->angular_velocity.z + (1 - alpha) * filtered_w;
        cur_ang_vel = filtered_w;
    }

    void VehiclePlugin::onDrive(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr drive_msg){

        target_steering_angle   = drive_msg->drive.steering_angle;
        target_speed            = drive_msg->drive.speed; // this must be treated as a torque signal

        if (target_speed > MAX_SPEED){
            target_speed = MAX_SPEED;
        }

        if (target_speed < -MAX_SPEED){
            target_speed = -MAX_SPEED;
        }

    }

    void VehiclePlugin::OnUpdate(const gazebo::common::UpdateInfo& info){
        if (last_update_time == gazebo::common::Time(0)){
            last_update_time = info.simTime;
            return;
        }

        double dt = (info.simTime - last_update_time).Double();
        last_update_time = info.simTime;

        updateSteering(dt);
        updateSpeed(dt);
    }

    void VehiclePlugin::updateSteering(double dt){

        node_->get_parameter("p", p);
        node_->get_parameter("i", i);
        node_->get_parameter("d", d);

        angular_vel_pid.SetPGain(p);
        angular_vel_pid.SetIGain(i);
        angular_vel_pid.SetDGain(d);

        double w_error = cur_ang_vel - target_steering_angle;
        double target_steering_angle_ = angular_vel_pid.Update(w_error, dt);
        
        auto current_front_left_steering_angle  = front_left_steer_joint->Position(0); 
        auto current_front_right_steering_angle  = front_right_steer_joint->Position(0); 

        double t_alph = tan(target_steering_angle_);
        double target_front_right_steering_angle = atan(WHEELBASE * t_alph / (WHEELBASE + 0.5 * TRACK_WIDTH * t_alph));
        double target_front_left_steering_angle = atan(WHEELBASE * t_alph / (WHEELBASE - 0.5 * TRACK_WIDTH * t_alph));

        double front_right_steering_angle_error = current_front_right_steering_angle - target_front_right_steering_angle;
        double front_left_steering_angle_error = current_front_left_steering_angle - target_front_left_steering_angle;

        double front_right_steering_torque = right_steering_pid.Update(front_right_steering_angle_error, dt);
        double front_left_steering_torque = left_steering_pid.Update(front_left_steering_angle_error, dt);
        
        front_right_steer_joint->SetForce(0, front_right_steering_torque);
        front_left_steer_joint->SetForce(0, front_left_steering_torque);

        // Verification logging
        //RCLCPP_INFO(this->node_->get_logger(), "Steering checkup | left=(%lf : %lf) middle=(%lf) right=(%lf : %lf)", target_front_left_steering_angle, current_front_left_steering_angle, target_steering_angle, target_front_right_steering_angle, current_front_right_steering_angle);
    }

    void VehiclePlugin::updateSpeed(double dt){ 
        
        // Set the target speed for the rear wheels
        auto target_speed_rads = target_speed/0.281;
        rear_left_wheel_joint->SetVelocity(0, target_speed_rads);
        rear_right_wheel_joint->SetVelocity(0, target_speed_rads);

    }

    void VehiclePlugin::getOdom(){ // timer callback 
         
        double rl_wheel_w  = rear_left_wheel_joint->GetVelocity(0); // left rear wheel angular velocity
        
        double rr_wheel_w  = rear_right_wheel_joint->GetVelocity(0); // right rear wheel angular velocity
        
        double rl_wheel_v = rl_wheel_w * 0.281; // left rear wheel linear velocity
        double rr_wheel_v = rr_wheel_w * 0.281; // right rear wheel linear velocity
 
        double v = ((rl_wheel_v + rr_wheel_v) / 2.0); // vehicle linear velocity
    
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "footprint_link";
        odom_msg->header.stamp = node_->now();

        odom_msg->twist.twist.linear.x = v;

        odom_pub->publish(std::move(odom_msg));

        auto vel_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        vel_msg->twist.linear.x = v;
        vel_msg->twist.angular.z = cur_ang_vel;
        vel_pub->publish(std::move(vel_msg));

    }

} // namespace vehicle_control