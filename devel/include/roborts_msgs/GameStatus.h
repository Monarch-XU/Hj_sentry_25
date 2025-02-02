// Generated by gencpp from file roborts_msgs/GameStatus.msg
// DO NOT EDIT!


#ifndef ROBORTS_MSGS_MESSAGE_GAMESTATUS_H
#define ROBORTS_MSGS_MESSAGE_GAMESTATUS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace roborts_msgs
{
template <class ContainerAllocator>
struct GameStatus_
{
  typedef GameStatus_<ContainerAllocator> Type;

  GameStatus_()
    : header()
    , area(0)
    , shoot_number(0.0)
    , health_point(0.0)
    , game_state(0)
    , heat_rest(0.0)  {
    }
  GameStatus_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , area(0)
    , shoot_number(0.0)
    , health_point(0.0)
    , game_state(0)
    , heat_rest(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _area_type;
  _area_type area;

   typedef double _shoot_number_type;
  _shoot_number_type shoot_number;

   typedef double _health_point_type;
  _health_point_type health_point;

   typedef uint8_t _game_state_type;
  _game_state_type game_state;

   typedef double _heat_rest_type;
  _heat_rest_type heat_rest;





  typedef boost::shared_ptr< ::roborts_msgs::GameStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roborts_msgs::GameStatus_<ContainerAllocator> const> ConstPtr;

}; // struct GameStatus_

typedef ::roborts_msgs::GameStatus_<std::allocator<void> > GameStatus;

typedef boost::shared_ptr< ::roborts_msgs::GameStatus > GameStatusPtr;
typedef boost::shared_ptr< ::roborts_msgs::GameStatus const> GameStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roborts_msgs::GameStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roborts_msgs::GameStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::roborts_msgs::GameStatus_<ContainerAllocator1> & lhs, const ::roborts_msgs::GameStatus_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.area == rhs.area &&
    lhs.shoot_number == rhs.shoot_number &&
    lhs.health_point == rhs.health_point &&
    lhs.game_state == rhs.game_state &&
    lhs.heat_rest == rhs.heat_rest;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::roborts_msgs::GameStatus_<ContainerAllocator1> & lhs, const ::roborts_msgs::GameStatus_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace roborts_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::roborts_msgs::GameStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roborts_msgs::GameStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roborts_msgs::GameStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roborts_msgs::GameStatus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roborts_msgs::GameStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roborts_msgs::GameStatus_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roborts_msgs::GameStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f256835487a7c2a1aeb09f0176114118";
  }

  static const char* value(const ::roborts_msgs::GameStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf256835487a7c2a1ULL;
  static const uint64_t static_value2 = 0xaeb09f0176114118ULL;
};

template<class ContainerAllocator>
struct DataType< ::roborts_msgs::GameStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roborts_msgs/GameStatus";
  }

  static const char* value(const ::roborts_msgs::GameStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roborts_msgs::GameStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"\n"
"uint8 area\n"
"float64 shoot_number\n"
"float64 health_point\n"
"uint8 game_state\n"
"float64 heat_rest\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::roborts_msgs::GameStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roborts_msgs::GameStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.area);
      stream.next(m.shoot_number);
      stream.next(m.health_point);
      stream.next(m.game_state);
      stream.next(m.heat_rest);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GameStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roborts_msgs::GameStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roborts_msgs::GameStatus_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "area: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.area);
    s << indent << "shoot_number: ";
    Printer<double>::stream(s, indent + "  ", v.shoot_number);
    s << indent << "health_point: ";
    Printer<double>::stream(s, indent + "  ", v.health_point);
    s << indent << "game_state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.game_state);
    s << indent << "heat_rest: ";
    Printer<double>::stream(s, indent + "  ", v.heat_rest);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBORTS_MSGS_MESSAGE_GAMESTATUS_H
