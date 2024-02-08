/* Author: Jason Hughes
 * Date: June 2023
 * About: Node to take in task info and send it to the pixhawk
 */

#include <cmath>
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
int flight_mode;

class GoToTask : public rclcpp::Node
{
public:
    GoToTask();
    void arm();
    void disarm();
    void land();
private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr task_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr flight_mode_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    int my_id_;
    float task_x_in_, task_y_in_, alt_;
    float current_pose_x_, current_pose_y_, current_pose_z_;
    uint64_t offboard_setpoint_counter_;

    px4_msgs::msg::VehicleStatus::SharedPtr status_;

    void cycleCallback();
    void taskCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void flight_mode_callback(const std_msgs::msg::Int32::SharedPtr msg);

    void publishControlMode();
    void publishTrajectory();
    void publishTrajectory2();
    void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0,float param3 = 0.0, float param4 = 0.0, float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);

    double calculateHeading();
};

GoToTask::GoToTask() : Node("goto_task")
{
    // set QoS
    rclcpp::QoS qos(10);
    qos.keep_last(10);;
    qos.best_effort();
    qos.transient_local();

    // get sys_id
    this -> declare_parameter("sys_id", 1);
    this -> declare_parameter("altitude", 10.1);
    my_id_ = this -> get_parameter("sys_id").get_parameter_value().get<int>();
    alt_ = this -> get_parameter("altitude").get_parameter_value().get<float>();

    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "test " << my_id_ << " connected to pixhawk");

    std::string pub_namespace = "/px4_" + std::to_string(my_id_);
    std::string sub_namespace = "/casa" + std::to_string(my_id_);

    std::cout<<pub_namespace<<std::endl;
    std::cout<<sub_namespace<<std::endl;

    control_mode_pub_ = this -> create_publisher<px4_msgs::msg::OffboardControlMode>(pub_namespace+"/fmu/in/offboard_control_mode", 10);
    vehicle_command_pub_ = this -> create_publisher<px4_msgs::msg::VehicleCommand>(pub_namespace+"/fmu/in/vehicle_command", 10);
    trajectory_pub_ = this -> create_publisher<px4_msgs::msg::TrajectorySetpoint>(pub_namespace+"/fmu/in/trajectory_setpoint", 10);

    task_sub_ = this -> create_subscription<geometry_msgs::msg::PoseStamped>(sub_namespace+"/internal/task",
                                                                             qos,
                                                                             std::bind(&GoToTask::taskCallback,
                                                                                       this,
                                                                                       std::placeholders::_1));

    status_sub_ = this -> create_subscription<px4_msgs::msg::VehicleStatus>(pub_namespace+"/fmu/out/vehicle_status",
                                                                            qos,
                                                                            std::bind(&GoToTask::statusCallback,
                                                                                      this,
                                                                                      std::placeholders::_1));
    pose_sub_ = this -> create_subscription<geometry_msgs::msg::PoseStamped>(sub_namespace+"/internal/local_position",
                                                                             qos,
                                                                             std::bind(&GoToTask::poseCallback,
                                                                                       this,
                                                                                       std::placeholders::_1));
    flight_mode_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("/fmu/flight_mode",10,
                                                                              std::bind(&GoToTask::flight_mode_callback,
                                                                                        this, std::placeholders::_1));


    offboard_setpoint_counter_ = 0;
    auto cycleCallback = [this]() -> void {
        if(flight_mode==1){
            arm();
            flight_mode = 0;
        }else if(flight_mode==2){
//            this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, 0,0,0,0,0,0,10.0);
            this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            publishTrajectory2();
            flight_mode = 0;
        }else if(flight_mode==3){
            publishTrajectory();
        }else if(flight_mode==4){
            land();
            flight_mode = 0;
        }else if(flight_mode==5){
            this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 2);
            disarm();
            flight_mode = 0;
        }
        this->publishControlMode();
    };
    timer_ = this -> create_wall_timer(10ms, cycleCallback);
}

void GoToTask::flight_mode_callback(const std_msgs::msg::Int32::SharedPtr msg){

    flight_mode = msg->data;
}

void GoToTask::arm()
{
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}
void GoToTask::disarm() {
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,0.0);
    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void GoToTask::land(){
    publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "Land command send");
}
void GoToTask::taskCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    task_x_in_ = msg->pose.position.x;
    task_y_in_ = msg->pose.position.y;
    RCLCPP_INFO_ONCE(this->get_logger(), "Pixhawk recieved command from task");
}


void GoToTask::statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    status_ = msg;
}

void GoToTask::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    current_pose_x_ = msg->pose.position.x;
    current_pose_y_ = msg->pose.position.y;
    current_pose_z_ = msg->pose.position.z;
}


void GoToTask::publishControlMode()
{
    px4_msgs::msg::OffboardControlMode msg;
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    control_mode_pub_ -> publish(msg);
}


void GoToTask::publishTrajectory()
{
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.position = {task_x_in_, task_y_in_, -1.3};
//    msg.velocity = {2.0,2.0,2.0};
    msg.yaw = calculateHeading();
    msg.timestamp = this -> get_clock() -> now().nanoseconds()/1000;
    // TODO: calculate heading in radians

    trajectory_pub_ -> publish(msg);
}
void GoToTask::publishTrajectory2()
{
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.position = {0, 0, -2.0};
//    msg.yaw = calculateHeading();
    msg.timestamp = this -> get_clock() -> now().nanoseconds()/1000;
    // TODO: calculate heading in radians

    trajectory_pub_ -> publish(msg);
}


void GoToTask::publishVehicleCommand(uint16_t command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    px4_msgs::msg::VehicleCommand msg;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;
    msg.command = command;
    msg.target_system = 0;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this -> get_clock()->now().nanoseconds()/1000;

    vehicle_command_pub_ -> publish(msg);
}

double GoToTask::calculateHeading()
{
    double heading;
    heading = atan2(task_y_in_-current_pose_y_, task_x_in_-current_pose_x_);
    return heading;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoToTask>());
    rclcpp::shutdown();

    return 0;
}
