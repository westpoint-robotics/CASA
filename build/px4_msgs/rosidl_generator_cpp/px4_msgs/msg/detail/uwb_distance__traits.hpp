// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/UwbDistance.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__UWB_DISTANCE__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__UWB_DISTANCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "px4_msgs/msg/detail/uwb_distance__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace px4_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UwbDistance & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: time_uwb_ms
  {
    out << "time_uwb_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.time_uwb_ms, out);
    out << ", ";
  }

  // member: counter
  {
    out << "counter: ";
    rosidl_generator_traits::value_to_yaml(msg.counter, out);
    out << ", ";
  }

  // member: sessionid
  {
    out << "sessionid: ";
    rosidl_generator_traits::value_to_yaml(msg.sessionid, out);
    out << ", ";
  }

  // member: time_offset
  {
    out << "time_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.time_offset, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: anchor_distance
  {
    if (msg.anchor_distance.size() == 0) {
      out << "anchor_distance: []";
    } else {
      out << "anchor_distance: [";
      size_t pending_items = msg.anchor_distance.size();
      for (auto item : msg.anchor_distance) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: nlos
  {
    if (msg.nlos.size() == 0) {
      out << "nlos: []";
    } else {
      out << "nlos: [";
      size_t pending_items = msg.nlos.size();
      for (auto item : msg.nlos) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: aoafirst
  {
    if (msg.aoafirst.size() == 0) {
      out << "aoafirst: []";
    } else {
      out << "aoafirst: [";
      size_t pending_items = msg.aoafirst.size();
      for (auto item : msg.aoafirst) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: position
  {
    if (msg.position.size() == 0) {
      out << "position: []";
    } else {
      out << "position: [";
      size_t pending_items = msg.position.size();
      for (auto item : msg.position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UwbDistance & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: time_uwb_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_uwb_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.time_uwb_ms, out);
    out << "\n";
  }

  // member: counter
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "counter: ";
    rosidl_generator_traits::value_to_yaml(msg.counter, out);
    out << "\n";
  }

  // member: sessionid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sessionid: ";
    rosidl_generator_traits::value_to_yaml(msg.sessionid, out);
    out << "\n";
  }

  // member: time_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.time_offset, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: anchor_distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.anchor_distance.size() == 0) {
      out << "anchor_distance: []\n";
    } else {
      out << "anchor_distance:\n";
      for (auto item : msg.anchor_distance) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: nlos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.nlos.size() == 0) {
      out << "nlos: []\n";
    } else {
      out << "nlos:\n";
      for (auto item : msg.nlos) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: aoafirst
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.aoafirst.size() == 0) {
      out << "aoafirst: []\n";
    } else {
      out << "aoafirst:\n";
      for (auto item : msg.aoafirst) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.position.size() == 0) {
      out << "position: []\n";
    } else {
      out << "position:\n";
      for (auto item : msg.position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UwbDistance & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace px4_msgs

namespace rosidl_generator_traits
{

[[deprecated("use px4_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const px4_msgs::msg::UwbDistance & msg,
  std::ostream & out, size_t indentation = 0)
{
  px4_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use px4_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const px4_msgs::msg::UwbDistance & msg)
{
  return px4_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<px4_msgs::msg::UwbDistance>()
{
  return "px4_msgs::msg::UwbDistance";
}

template<>
inline const char * name<px4_msgs::msg::UwbDistance>()
{
  return "px4_msgs/msg/UwbDistance";
}

template<>
struct has_fixed_size<px4_msgs::msg::UwbDistance>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::UwbDistance>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::UwbDistance>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__UWB_DISTANCE__TRAITS_HPP_
