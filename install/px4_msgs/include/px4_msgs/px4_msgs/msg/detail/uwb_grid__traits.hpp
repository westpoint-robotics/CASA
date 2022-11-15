// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/UwbGrid.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__UWB_GRID__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__UWB_GRID__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "px4_msgs/msg/detail/uwb_grid__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace px4_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UwbGrid & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: initator_time
  {
    out << "initator_time: ";
    rosidl_generator_traits::value_to_yaml(msg.initator_time, out);
    out << ", ";
  }

  // member: num_anchors
  {
    out << "num_anchors: ";
    rosidl_generator_traits::value_to_yaml(msg.num_anchors, out);
    out << ", ";
  }

  // member: target_gps
  {
    if (msg.target_gps.size() == 0) {
      out << "target_gps: []";
    } else {
      out << "target_gps: [";
      size_t pending_items = msg.target_gps.size();
      for (auto item : msg.target_gps) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: target_pos
  {
    if (msg.target_pos.size() == 0) {
      out << "target_pos: []";
    } else {
      out << "target_pos: [";
      size_t pending_items = msg.target_pos.size();
      for (auto item : msg.target_pos) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: anchor_pos_0
  {
    if (msg.anchor_pos_0.size() == 0) {
      out << "anchor_pos_0: []";
    } else {
      out << "anchor_pos_0: [";
      size_t pending_items = msg.anchor_pos_0.size();
      for (auto item : msg.anchor_pos_0) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: anchor_pos_1
  {
    if (msg.anchor_pos_1.size() == 0) {
      out << "anchor_pos_1: []";
    } else {
      out << "anchor_pos_1: [";
      size_t pending_items = msg.anchor_pos_1.size();
      for (auto item : msg.anchor_pos_1) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: anchor_pos_2
  {
    if (msg.anchor_pos_2.size() == 0) {
      out << "anchor_pos_2: []";
    } else {
      out << "anchor_pos_2: [";
      size_t pending_items = msg.anchor_pos_2.size();
      for (auto item : msg.anchor_pos_2) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: anchor_pos_3
  {
    if (msg.anchor_pos_3.size() == 0) {
      out << "anchor_pos_3: []";
    } else {
      out << "anchor_pos_3: [";
      size_t pending_items = msg.anchor_pos_3.size();
      for (auto item : msg.anchor_pos_3) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: anchor_pos_4
  {
    if (msg.anchor_pos_4.size() == 0) {
      out << "anchor_pos_4: []";
    } else {
      out << "anchor_pos_4: [";
      size_t pending_items = msg.anchor_pos_4.size();
      for (auto item : msg.anchor_pos_4) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: anchor_pos_5
  {
    if (msg.anchor_pos_5.size() == 0) {
      out << "anchor_pos_5: []";
    } else {
      out << "anchor_pos_5: [";
      size_t pending_items = msg.anchor_pos_5.size();
      for (auto item : msg.anchor_pos_5) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: anchor_pos_6
  {
    if (msg.anchor_pos_6.size() == 0) {
      out << "anchor_pos_6: []";
    } else {
      out << "anchor_pos_6: [";
      size_t pending_items = msg.anchor_pos_6.size();
      for (auto item : msg.anchor_pos_6) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: anchor_pos_7
  {
    if (msg.anchor_pos_7.size() == 0) {
      out << "anchor_pos_7: []";
    } else {
      out << "anchor_pos_7: [";
      size_t pending_items = msg.anchor_pos_7.size();
      for (auto item : msg.anchor_pos_7) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: anchor_pos_8
  {
    if (msg.anchor_pos_8.size() == 0) {
      out << "anchor_pos_8: []";
    } else {
      out << "anchor_pos_8: [";
      size_t pending_items = msg.anchor_pos_8.size();
      for (auto item : msg.anchor_pos_8) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: anchor_pos_9
  {
    if (msg.anchor_pos_9.size() == 0) {
      out << "anchor_pos_9: []";
    } else {
      out << "anchor_pos_9: [";
      size_t pending_items = msg.anchor_pos_9.size();
      for (auto item : msg.anchor_pos_9) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: anchor_pos_10
  {
    if (msg.anchor_pos_10.size() == 0) {
      out << "anchor_pos_10: []";
    } else {
      out << "anchor_pos_10: [";
      size_t pending_items = msg.anchor_pos_10.size();
      for (auto item : msg.anchor_pos_10) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: anchor_pos_11
  {
    if (msg.anchor_pos_11.size() == 0) {
      out << "anchor_pos_11: []";
    } else {
      out << "anchor_pos_11: [";
      size_t pending_items = msg.anchor_pos_11.size();
      for (auto item : msg.anchor_pos_11) {
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
  const UwbGrid & msg,
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

  // member: initator_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "initator_time: ";
    rosidl_generator_traits::value_to_yaml(msg.initator_time, out);
    out << "\n";
  }

  // member: num_anchors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_anchors: ";
    rosidl_generator_traits::value_to_yaml(msg.num_anchors, out);
    out << "\n";
  }

  // member: target_gps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.target_gps.size() == 0) {
      out << "target_gps: []\n";
    } else {
      out << "target_gps:\n";
      for (auto item : msg.target_gps) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: target_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.target_pos.size() == 0) {
      out << "target_pos: []\n";
    } else {
      out << "target_pos:\n";
      for (auto item : msg.target_pos) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: anchor_pos_0
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.anchor_pos_0.size() == 0) {
      out << "anchor_pos_0: []\n";
    } else {
      out << "anchor_pos_0:\n";
      for (auto item : msg.anchor_pos_0) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: anchor_pos_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.anchor_pos_1.size() == 0) {
      out << "anchor_pos_1: []\n";
    } else {
      out << "anchor_pos_1:\n";
      for (auto item : msg.anchor_pos_1) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: anchor_pos_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.anchor_pos_2.size() == 0) {
      out << "anchor_pos_2: []\n";
    } else {
      out << "anchor_pos_2:\n";
      for (auto item : msg.anchor_pos_2) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: anchor_pos_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.anchor_pos_3.size() == 0) {
      out << "anchor_pos_3: []\n";
    } else {
      out << "anchor_pos_3:\n";
      for (auto item : msg.anchor_pos_3) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: anchor_pos_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.anchor_pos_4.size() == 0) {
      out << "anchor_pos_4: []\n";
    } else {
      out << "anchor_pos_4:\n";
      for (auto item : msg.anchor_pos_4) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: anchor_pos_5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.anchor_pos_5.size() == 0) {
      out << "anchor_pos_5: []\n";
    } else {
      out << "anchor_pos_5:\n";
      for (auto item : msg.anchor_pos_5) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: anchor_pos_6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.anchor_pos_6.size() == 0) {
      out << "anchor_pos_6: []\n";
    } else {
      out << "anchor_pos_6:\n";
      for (auto item : msg.anchor_pos_6) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: anchor_pos_7
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.anchor_pos_7.size() == 0) {
      out << "anchor_pos_7: []\n";
    } else {
      out << "anchor_pos_7:\n";
      for (auto item : msg.anchor_pos_7) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: anchor_pos_8
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.anchor_pos_8.size() == 0) {
      out << "anchor_pos_8: []\n";
    } else {
      out << "anchor_pos_8:\n";
      for (auto item : msg.anchor_pos_8) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: anchor_pos_9
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.anchor_pos_9.size() == 0) {
      out << "anchor_pos_9: []\n";
    } else {
      out << "anchor_pos_9:\n";
      for (auto item : msg.anchor_pos_9) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: anchor_pos_10
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.anchor_pos_10.size() == 0) {
      out << "anchor_pos_10: []\n";
    } else {
      out << "anchor_pos_10:\n";
      for (auto item : msg.anchor_pos_10) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: anchor_pos_11
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.anchor_pos_11.size() == 0) {
      out << "anchor_pos_11: []\n";
    } else {
      out << "anchor_pos_11:\n";
      for (auto item : msg.anchor_pos_11) {
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

inline std::string to_yaml(const UwbGrid & msg, bool use_flow_style = false)
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
  const px4_msgs::msg::UwbGrid & msg,
  std::ostream & out, size_t indentation = 0)
{
  px4_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use px4_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const px4_msgs::msg::UwbGrid & msg)
{
  return px4_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<px4_msgs::msg::UwbGrid>()
{
  return "px4_msgs::msg::UwbGrid";
}

template<>
inline const char * name<px4_msgs::msg::UwbGrid>()
{
  return "px4_msgs/msg/UwbGrid";
}

template<>
struct has_fixed_size<px4_msgs::msg::UwbGrid>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::UwbGrid>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::UwbGrid>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__UWB_GRID__TRAITS_HPP_
