//
// Created by dbkim-ros on 2/22/23.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;

class flightMode : public rclcpp::Node
{

public:

    flightMode()
            : Node("rrc_flight_mode"), count_(0)
    {
        task_update_vehicle_pub = this->create_publisher<std_msgs::msg::Int64>("/casa/task_update_vehicle",10);
        task_update_num_pub = this->create_publisher<std_msgs::msg::Int64>("/casa/task_update_num",10);

        timer_ = this->create_wall_timer(10ms, std::bind(&flightMode::timer_callback,this));
    }


private:

    void timer_callback()
    {
        auto task_update_vehicle_msg = std_msgs::msg::Int64();
        auto task_update_num_msg = std_msgs::msg::Int64();

        std::cout<<"Enter the agent number "<<std::endl;
        std::cin>>task_update_vehicle_input;

        std::cout<<"Enter the new task number "<<std::endl;
        std::cin>>task_update_num_input;

        task_update_vehicle_msg.data = task_update_vehicle_input;
        task_update_num_msg.data = task_update_num_input;

        task_update_vehicle_pub->publish(task_update_vehicle_msg);
        task_update_num_pub->publish(task_update_num_msg);

//        task_update_vehicle_input = 0;
//
//        task_update_vehicle_msg.data = task_update_vehicle_input;
//
//        task_update_vehicle_pub->publish(task_update_vehicle_msg);

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr task_update_vehicle_pub;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr task_update_num_pub;

    size_t count_;
    int task_update_vehicle_input;
    int task_update_num_input;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<flightMode>());
    rclcpp::shutdown();
    return 0;

}

