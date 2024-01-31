//
// Created by dbkim-ros on 2/23/23.
//
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "px4_msgs/msg/vehicle_odometry.hpp"

class Px4ddsSub : public rclcpp::Node
{
public:
    Px4ddsSub()
    :Node("PX4_DDS_Sub")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5),qos_profile);

        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry", qos, std::bind(&Px4ddsSub::topic_callback, this, std::placeholders::_1)
                );
    }

private:
    void topic_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr dbk) const{
        std::cout<<dbk->position[0]<<std::endl;
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4ddsSub>());
    rclcpp::shutdown();

    return 0;
}