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
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
px4_msgs::msg::VehicleOdometry::_timestamp_type dbk_timestamp, dbk_timestamp_sample;


class flightMode : public rclcpp::Node
{

public:

    flightMode()
            : Node("rrc_flight_mode"), count_(0)
    {
        mode_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/fmu/flight_mode",10);
//        uav_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/fmu/uav_number",10);
        timer_ = this->create_wall_timer(10ms, std::bind(&flightMode::timer_callback,this));
    }


private:

    void timer_callback()
    {
        auto flight_mode_msg = std_msgs::msg::Int32();
//        auto uav_number_msg = std_msgs::msg::Int64 ();

//        std::cout<<"Enter the UAV number"<<std::endl;
//        std::cin>>uav_number;

        std::cout<<"Enter the flight mode "<<std::endl;
        std::cin>>flight_mode;

        flight_mode_msg.data = flight_mode;
//        uav_number_msg.data = uav_number;
        mode_publisher_->publish(flight_mode_msg);
//        uav_publisher_->publish(uav_number_msg);

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_publisher_;
//    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr uav_publisher_;
    size_t count_;
    int flight_mode;
    int uav_number;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<flightMode>());
    rclcpp::shutdown();
    return 0;

}

