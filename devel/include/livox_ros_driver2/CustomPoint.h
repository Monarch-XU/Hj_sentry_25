// Generated by gencpp from file livox_ros_driver2/CustomPoint.msg
// DO NOT EDIT!


#ifndef LIVOX_ROS_DRIVER2_MESSAGE_CUSTOMPOINT_H
#define LIVOX_ROS_DRIVER2_MESSAGE_CUSTOMPOINT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace livox_ros_driver2
{
template <class ContainerAllocator>
struct CustomPoint_
{
  typedef CustomPoint_<ContainerAllocator> Type;

  CustomPoint_()
    : offset_time(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , reflectivity(0)
    , tag(0)
    , line(0)  {
    }
  CustomPoint_(const ContainerAllocator& _alloc)
    : offset_time(0)
    , x(0.0)
    , y(0.0)
    , z(0.0)
    , reflectivity(0)
    , tag(0)
    , line(0)  {
  (void)_alloc;
    }



   typedef uint32_t _offset_time_type;
  _offset_time_type offset_time;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;

   typedef uint8_t _reflectivity_type;
  _reflectivity_type reflectivity;

   typedef uint8_t _tag_type;
  _tag_type tag;

   typedef uint8_t _line_type;
  _line_type line;





  typedef boost::shared_ptr< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> const> ConstPtr;

}; // struct CustomPoint_

typedef ::livox_ros_driver2::CustomPoint_<std::allocator<void> > CustomPoint;

typedef boost::shared_ptr< ::livox_ros_driver2::CustomPoint > CustomPointPtr;
typedef boost::shared_ptr< ::livox_ros_driver2::CustomPoint const> CustomPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::livox_ros_driver2::CustomPoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::livox_ros_driver2::CustomPoint_<ContainerAllocator1> & lhs, const ::livox_ros_driver2::CustomPoint_<ContainerAllocator2> & rhs)
{
  return lhs.offset_time == rhs.offset_time &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.z == rhs.z &&
    lhs.reflectivity == rhs.reflectivity &&
    lhs.tag == rhs.tag &&
    lhs.line == rhs.line;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::livox_ros_driver2::CustomPoint_<ContainerAllocator1> & lhs, const ::livox_ros_driver2::CustomPoint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace livox_ros_driver2

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "109a3cc548bb1f96626be89a5008bd6d";
  }

  static const char* value(const ::livox_ros_driver2::CustomPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x109a3cc548bb1f96ULL;
  static const uint64_t static_value2 = 0x626be89a5008bd6dULL;
};

template<class ContainerAllocator>
struct DataType< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "livox_ros_driver2/CustomPoint";
  }

  static const char* value(const ::livox_ros_driver2::CustomPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Livox costom pointcloud format.\n"
"\n"
"uint32 offset_time      # offset time relative to the base time\n"
"float32 x               # X axis, unit:m\n"
"float32 y               # Y axis, unit:m\n"
"float32 z               # Z axis, unit:m\n"
"uint8 reflectivity      # reflectivity, 0~255\n"
"uint8 tag               # livox tag\n"
"uint8 line              # laser number in lidar\n"
"\n"
;
  }

  static const char* value(const ::livox_ros_driver2::CustomPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.offset_time);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
      stream.next(m.reflectivity);
      stream.next(m.tag);
      stream.next(m.line);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CustomPoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::livox_ros_driver2::CustomPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::livox_ros_driver2::CustomPoint_<ContainerAllocator>& v)
  {
    s << indent << "offset_time: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.offset_time);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
    s << indent << "reflectivity: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reflectivity);
    s << indent << "tag: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.tag);
    s << indent << "line: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.line);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIVOX_ROS_DRIVER2_MESSAGE_CUSTOMPOINT_H
