/*
 * Jason Hughes
 * Date: Feb. 2023
 * Description: Node to interface with the pixhawk for global and local position. 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"
#include "utils/casa_utils.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

class PixhawkInterface : public rclcpp::Node
{
public:
  PixhawkInterface() : Node("pixhawk_position_interface")
  {
    // set the Quality of Service
    // RELIABILITY: best effort
    // DURABILITY: transient local
    // everything else is kept default
    rclcpp::QoS qos(10);
    qos.keep_last(10);
    qos.best_effort();
    qos.transient_local();

    this -> declare_parameter("sys_id", 1);

    sys_id_ = this -> get_parameter("sys_id").get_parameter_value().get<int>();

    std::string sys_namespace = "px4_"+ std::to_string(sys_id_);
    internal_namespace_ = "casa"+ std::to_string(sys_id_);
    
    local_sub_ = this -> create_subscription<px4_msgs::msg::VehicleLocalPosition>(sys_namespace+"/fmu/out/vehicle_local_position",
										qos,
										std::bind(&PixhawkInterface::localCallback, this, std::placeholders::_1));
    gps_sub_ = this -> create_subscription<px4_msgs::msg::VehicleGlobalPosition>(sys_namespace+"/fmu/out/vehicle_global_position",
									 qos,
									 std::bind(&PixhawkInterface::gpsCallback, this, std::placeholders::_1));

    attitude_sub_ = this -> create_subscription<px4_msgs::msg::VehicleAttitude>(sys_namespace+"/fmu/out/vehicle_attitude",
										qos,
										std::bind(&PixhawkInterface::attitudeCallback, this, std::placeholders::_1));
									     
    
    local_pub_ = this -> create_publisher<geometry_msgs::msg::PoseStamped>(internal_namespace_+"/internal/local_position", qos);
    gps_pub_ = this -> create_publisher<sensor_msgs::msg::NavSatFix>(internal_namespace_+"/internal/global_position", qos);
    heading_pub_ = this -> create_publisher<std_msgs::msg::Float32>(internal_namespace_+"/internal/heading", qos);
    
    timer_ = this -> create_wall_timer(100ms, std::bind(&PixhawkInterface::timerCallback, this));

  }

private:

  int sys_id_;
  std::string internal_namespace_;
  
  float lat_in_, lon_in_, alt_in_;

  float local_x_in_, local_y_in_, local_z_in_;

  float quat_x_in_, quat_y_in_, quat_z_in_, quat_w_in_;

  float heading_in_;
  
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr gps_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr heading_pub_;
  
  rclcpp::TimerBase::SharedPtr timer_;

  void localCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void gpsCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
  void attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  
  void localPosePublisher(float x, float y, float z, float qx, float qy, float qz, float qw);
  void globalPosePublisher(float lat, float lon, float alt, int sys_id);
  void headingPublisher(float heading);
  
  float int_to_float_conversion(int input, int flag);
  
  void timerCallback();
};


void PixhawkInterface::timerCallback()
{
  localPosePublisher(local_x_in_, local_y_in_, local_z_in_, quat_x_in_, quat_y_in_, quat_z_in_, quat_w_in_);
  globalPosePublisher(lat_in_, lon_in_, alt_in_, sys_id_);
  headingPublisher(heading_in_);
}

void PixhawkInterface::attitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
  quat_x_in_ = msg->q[0];
  quat_y_in_ = msg->q[1];
  quat_z_in_ = msg->q[2];
  quat_w_in_ = msg->q[3];
}

void PixhawkInterface::localCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
{
  local_x_in_ = msg->x;
  local_y_in_ = msg->y;
  local_z_in_ = msg->z;
  heading_in_ = msg->heading;
  RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "ID " << sys_id_ << " connected to pixhawk");
}

void PixhawkInterface::gpsCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg)
{
  lat_in_ = msg->lat;
  lon_in_ = msg->lon;
  alt_in_ = msg->alt;
  //RCLCPP_INFO_STREAM(this->get_logger(), "recieved gps message at: "<< lat_in_ << ", " << lon_in_);
}

void PixhawkInterface::localPosePublisher(float x, float y, float z, float qx, float qy, float qz, float qw)
{
  geometry_msgs::msg::PoseStamped msg{};

  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;

  msg.pose.orientation.x = qx;
  msg.pose.orientation.y = qy;
  msg.pose.orientation.z = qz;
  msg.pose.orientation.w = qw;

  msg.header.stamp = this -> get_clock() -> now();

  local_pub_->publish(msg);
}

void PixhawkInterface::globalPosePublisher(float lat, float lon, float alt, int sys_id)
{
  sensor_msgs::msg::NavSatFix msg{};

  msg.latitude = lat;
  msg.longitude = lon;
  msg.altitude = alt;

  msg.header.stamp = this -> get_clock() -> now();
  msg.header.frame_id = std::to_string(sys_id);

  gps_pub_->publish(msg);
}

void PixhawkInterface::headingPublisher(float heading)
{
  std_msgs::msg::Float32 msg{};

  msg.data = heading;

  heading_pub_ -> publish(msg);
}

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PixhawkInterface>());
  rclcpp::shutdown();
  
  return 0;
}


  
