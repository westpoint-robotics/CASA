//
// Created by dbkim-ros on 2/22/23.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;
px4_msgs::msg::VehicleOdometry::_timestamp_type dbk_timestamp, dbk_timestamp_sample;
double pos_x, pos_y, pos_z, q_w,q_x,q_y,q_z,
        cr,cp,cy,sr,sp,sy, qw,qx,qy,qz;
double sinr_cosp, cosr_cosp, sinp, siny_cosp, cosy_cosp;
double roll, pitch, yaw;

class Mocap2px4pub : public rclcpp::Node
{

public:

    Mocap2px4pub()
            : Node("mocap2px4_publisher"), count_(0)
    {

        rmw_qos_profile_t qos_profile2 = rmw_qos_profile_sensor_data;
        auto qos2 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile2.history,5), qos_profile2);

        publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry",qos2);

        timer_ = this->create_wall_timer(10ms, std::bind(&Mocap2px4pub::timer_callback,this));
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),qos_profile);

        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/vrpn_mocap/RigidBody/pose", qos, std::bind(&Mocap2px4pub::topic_callback2, this, std::placeholders::_1)
        );

        subscription2_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry", qos, std::bind(&Mocap2px4pub::topic_callback3, this, std::placeholders::_1)
        );
    }


private:

    void timer_callback()
    {
        auto message_test = px4_msgs::msg::VehicleOdometry();

        message_test.timestamp = dbk_timestamp;
        message_test.timestamp_sample = dbk_timestamp_sample;
        message_test.pose_frame = message_test.POSE_FRAME_FRD;
        message_test.position = {(float)pos_x,(float)pos_y,(float)pos_z};
        message_test.q = {(float)q_w,(float)q_x,(float)q_y,(float)q_z };
        message_test.reset_counter = 7;
        message_test.quality = 0;

        publisher_->publish(message_test);

    }
    void topic_callback2(const geometry_msgs::msg::PoseStamped::SharedPtr dbk) const{
//        std::cout<<dbk->pose.position.x<<std::endl;

        pos_x = dbk->pose.position.x;
        pos_y = -dbk->pose.position.y;
        pos_z = -dbk->pose.position.z;

        qw = dbk->pose.orientation.w;
        qx = dbk->pose.orientation.x;
        qy = dbk->pose.orientation.y;
        qz = dbk->pose.orientation.z;

        sinr_cosp = 2 * (qw * qx + qy * qz);
        cosr_cosp = 1 - 2 * (qx * qx + qy *qy);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        sinp = 2 * (qw * qy - qz * qx);
        pitch = std::asin(sinp);

        siny_cosp = 2 * (qw * qz + qx * qy);
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        yaw = std::atan2(siny_cosp, cosy_cosp);

        std::cout<<"RPY: "<<roll*180/M_PI<<" "<< pitch*180/M_PI<<" "<<yaw*180/M_PI<<std::endl;

        cy = std::cos(-yaw * 0.5);
        sy = std::sin(-yaw * 0.5);
        cp = std::cos(-pitch * 0.5);
        sp = std::sin(-pitch * 0.5);
        cr = std::cos(roll * 0.5);
        sr = std::sin(roll * 0.5);

        q_w = cy * cp * cr + sy * sp * sr;
        q_x = cy * cp * sr - sy * sp * cr;
        q_y = sy * cp * sr + cy * sp * cr;
        q_z = sy * cp * cr - cy * sp * sr;
        std::cout<<"Qwxyz: "<<q_w<<" "<< q_x<<" "<<q_y<<" " << q_z<<std::endl;

    }
    void topic_callback3(const px4_msgs::msg::VehicleOdometry::SharedPtr dbk) const{
        dbk_timestamp = dbk->timestamp;
        dbk_timestamp_sample = dbk->timestamp_sample;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription2_;
    size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Mocap2px4pub>());
    rclcpp::shutdown();
    return 0;

}

