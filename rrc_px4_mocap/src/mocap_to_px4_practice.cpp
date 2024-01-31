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

class Mocap2px4pub : public rclcpp::Node
{

public:

    Mocap2px4pub()
            : Node("mocap2px4_publisher"), count_(0)
    {

//        rmw_qos_profile_t qos_profile2 = {
//                RMW_QOS_POLICY_HISTORY_KEEP_LAST,
//                5,
//                RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
//                RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
//                RMW_QOS_DEADLINE_DEFAULT,
//                RMW_QOS_LIFESPAN_DEFAULT,
//                RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
//                RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
//                false
//        };
        rmw_qos_profile_t qos_profile2 = rmw_qos_profile_sensor_data;
        auto qos2 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile2.history,5), qos_profile2);

        publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry",qos2);

//        timer_ = this->create_wall_timer(10ms, std::bind(&Mocap2px4pub::timer_callback,this));
//        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
//        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),qos_profile);
//
//        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
//                "/fmu/out/vehicle_odometry", qos, std::bind(&Mocap2px4pub::topic_callback2, this, std::placeholders::_1)
//        );
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/vrpn_mocap/RigidBody/pose", 10, std::bind(&Mocap2px4pub::topic_callback2, this, std::placeholders::_1)
        );

    }


private:

    void timer_callback()
    {
        auto message_test = px4_msgs::msg::VehicleOdometry();

//        message_test.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
//        message_test.timestamp_sample = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
        message_test.timestamp = dbk_timestamp;
        message_test.timestamp_sample = dbk_timestamp_sample;
        message_test.pose_frame = message_test.POSE_FRAME_NED;
        message_test.position = {0,0,10};
        message_test.q = {0,0,0,0};
//        message_test.velocity = {NAN,NAN,NAN};
//        message_test.position_variance = {0,0,0};
//        message_test.position_variance = {0,0,0};
//        message_test.position_variance = {0,0,0};
        message_test.reset_counter = 7;
        message_test.quality = 0;

        publisher_->publish(message_test);
//        std::cout<< message_test.timestamp<<" "<<message_test.position[2]<<std::endl;
    }
    void topic_callback2(const geometry_msgs::msg::PoseStamped::SharedPtr dbk) const{
        std::cout<<dbk->pose.position.x<<std::endl;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Mocap2px4pub>());
    rclcpp::shutdown();
    return 0;

}

