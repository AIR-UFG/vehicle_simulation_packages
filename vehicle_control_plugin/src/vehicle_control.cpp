#include <vehicle_control/vehicle_control.hpp>

#include <iostream>

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

        rear_right_wheel_pid = gazebo::common::PID(1000, 0.0, 1.0);
        rear_right_wheel_pid = gazebo::common::PID(1000, 0.0, 1.0);
        front_left_wheel_pid = gazebo::common::PID(1000, 0.0, 1.0);
        front_right_wheel_pid = gazebo::common::PID(1000, 0.0, 1.0);

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

        // Connect to the update event
        update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&VehiclePlugin::OnUpdate, this, std::placeholders::_1)
        );

        // ROS initialization
        node_ = gazebo_ros::Node::Get(sdf);

        node_->declare_parameter<std::string>("drive_command_topic", "/sd_control/cmd_vel");
        node_->get_parameter("drive_command_topic", drive_command_topic);


        // Create a subscriber to the ackermann drive topic
        drive_sub_ = node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_command_topic, 1, std::bind(&VehiclePlugin::onDrive, this, std::placeholders::_1)
        );

        std::cout << "drive_commmand_topic" << drive_command_topic << std::endl;
    }

    void VehiclePlugin::onDrive(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr drive_msg){

        target_steering_angle   = drive_msg->drive.steering_angle;
        target_speed            = drive_msg->drive.speed;

        if (target_speed > MAX_SPEED){
            target_speed = MAX_SPEED;
        }

        if (target_speed < -MAX_SPEED){
            target_speed = -MAX_SPEED;
        }

        if (target_steering_angle > MAX_STEERING_angle){
            target_steering_angle = MAX_STEERING_angle;
        }

        if (target_steering_angle < -MAX_STEERING_angle){
            target_steering_angle = -MAX_STEERING_angle;
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
        
        auto current_front_left_steering_angle  = front_left_steer_joint->Position(0);
        auto current_front_right_steering_angle  = front_right_steer_joint->Position(0);

        double t_alph = tan(target_steering_angle);
        double target_front_right_steering_angle = atan(WHEELBASE * t_alph / (WHEELBASE + 0.5 * TRACK_WIDTH * t_alph));
        double target_front_left_steering_angle = atan(WHEELBASE * t_alph / (WHEELBASE - 0.5 * TRACK_WIDTH * t_alph));

        double front_right_steering_angle_error = current_front_right_steering_angle - target_front_right_steering_angle;
        double front_right_steering_torque = right_steering_pid.Update(front_right_steering_angle_error, dt);
        front_right_steer_joint->SetForce(0, front_right_steering_torque);

        double front_left_steering_angle_error = current_front_left_steering_angle - target_front_left_steering_angle;
        double front_left_steering_torque = left_steering_pid.Update(front_left_steering_angle_error, dt);
        front_left_steer_joint->SetForce(0, front_left_steering_torque);

    }

    void VehiclePlugin::updateSpeed(double dt){

        double rear_left_wheel_speed  = rear_left_wheel_joint->GetVelocity(0);
        double rear_left_wheel_speed_error = rear_left_wheel_speed - target_speed;
        double rear_left_wheel_velocity_torque = rear_left_wheel_pid.Update(rear_left_wheel_speed_error, dt);
        rear_left_wheel_joint->SetForce(0, rear_left_wheel_velocity_torque);

        double rear_right_wheel_speed  = rear_right_wheel_joint->GetVelocity(0);
        double rear_right_wheel_speed_error = rear_right_wheel_speed - target_speed;
        double rear_right_wheel_velocity_torque = rear_right_wheel_pid.Update(rear_right_wheel_speed_error, dt);
        rear_right_wheel_joint->SetForce(0, rear_right_wheel_velocity_torque);

    }

} // namespace vehicle_control