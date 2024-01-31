
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <std_msgs/msg/int64.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <functional>


using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
int flight_mode;
int uav_number;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control")
    {
        rmw_qos_profile_t qos_profile2 = rmw_qos_profile_sensor_data;
        auto qos2 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile2.history,5), qos_profile2);

        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/px4_" + std::to_string(uav_number) + "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/px4_" + std::to_string(uav_number) + "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/px4_" + std::to_string(uav_number) + "/fmu/in/vehicle_command", 10);
        flight_mode_subscriber_ = this->create_subscription<std_msgs::msg::Int64>("/fmu/flight_mode",10,std::bind(&OffboardControl::flight_mode_callback, this, std::placeholders::_1));
        uav_number_subscriber_ = this->create_subscription<std_msgs::msg::Int64>("/fmu/uav_number",10,std::bind(&OffboardControl::uav_number_callback, this, std::placeholders::_1));

        auto timer_callback = [this]() -> void {

            if(flight_mode==1){
                arm();
                flight_mode = 0;
            }else if(flight_mode==2){
                this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
                flight_mode = 0;
            }else if(flight_mode==3){
                land();
                flight_mode = 0;
            }else if(flight_mode==4){
                disarm();
                flight_mode = 0;
            }
            publish_offboard_control_mode();
            publish_trajectory_setpoint();

        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm();
    void disarm();
    void land();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr flight_mode_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr uav_number_subscriber_;

    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void flight_mode_callback(const std_msgs::msg::Int64::SharedPtr msg);
    void uav_number_callback(const std_msgs::msg::Int64::SharedPtr msg);
};

void OffboardControl::flight_mode_callback(const std_msgs::msg::Int64::SharedPtr msg){

    flight_mode = msg->data;
}

void OffboardControl::uav_number_callback(const std_msgs::msg::Int64::SharedPtr msg){

    uav_number = msg->data;
}

void OffboardControl::publish_offboard_control_mode()
{
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
    TrajectorySetpoint msg{};
    msg.position = {-0.94, 0.1, -5.0};
//    msg.yaw = 0; // [-PI:PI]
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = uav_number + 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void OffboardControl::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::disarm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardControl::land(){
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND,0.0);
    RCLCPP_INFO(this->get_logger(), "Land command send");
}

int main(int argc, char *argv[])
{
    std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}