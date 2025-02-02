// Generated by gencpp from file roborts_msgs/LocationInfo.msg
// DO NOT EDIT!


#ifndef ROBORTS_MSGS_MESSAGE_LOCATIONINFO_H
#define ROBORTS_MSGS_MESSAGE_LOCATIONINFO_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace roborts_msgs
{
template <class ContainerAllocator>
struct LocationInfo_
{
  typedef LocationInfo_<ContainerAllocator> Type;

  LocationInfo_()
    : if_relocation(false)
    , point_cloud_quantity(0.0)
    , tranDist(0.0)
    , angleDist(0.0)
    , angle_apeed(0.0)
    , score(0.0)
    , if_match_success(false)  {
    }
  LocationInfo_(const ContainerAllocator& _alloc)
    : if_relocation(false)
    , point_cloud_quantity(0.0)
    , tranDist(0.0)
    , angleDist(0.0)
    , angle_apeed(0.0)
    , score(0.0)
    , if_match_success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _if_relocation_type;
  _if_relocation_type if_relocation;

   typedef double _point_cloud_quantity_type;
  _point_cloud_quantity_type point_cloud_quantity;

   typedef double _tranDist_type;
  _tranDist_type tranDist;

   typedef double _angleDist_type;
  _angleDist_type angleDist;

   typedef double _angle_apeed_type;
  _angle_apeed_type angle_apeed;

   typedef double _score_type;
  _score_type score;

   typedef uint8_t _if_match_success_type;
  _if_match_success_type if_match_success;





  typedef boost::shared_ptr< ::roborts_msgs::LocationInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::roborts_msgs::LocationInfo_<ContainerAllocator> const> ConstPtr;

}; // struct LocationInfo_

typedef ::roborts_msgs::LocationInfo_<std::allocator<void> > LocationInfo;

typedef boost::shared_ptr< ::roborts_msgs::LocationInfo > LocationInfoPtr;
typedef boost::shared_ptr< ::roborts_msgs::LocationInfo const> LocationInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::roborts_msgs::LocationInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::roborts_msgs::LocationInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::roborts_msgs::LocationInfo_<ContainerAllocator1> & lhs, const ::roborts_msgs::LocationInfo_<ContainerAllocator2> & rhs)
{
  return lhs.if_relocation == rhs.if_relocation &&
    lhs.point_cloud_quantity == rhs.point_cloud_quantity &&
    lhs.tranDist == rhs.tranDist &&
    lhs.angleDist == rhs.angleDist &&
    lhs.angle_apeed == rhs.angle_apeed &&
    lhs.score == rhs.score &&
    lhs.if_match_success == rhs.if_match_success;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::roborts_msgs::LocationInfo_<ContainerAllocator1> & lhs, const ::roborts_msgs::LocationInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace roborts_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::roborts_msgs::LocationInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::roborts_msgs::LocationInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roborts_msgs::LocationInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::roborts_msgs::LocationInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roborts_msgs::LocationInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::roborts_msgs::LocationInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::roborts_msgs::LocationInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4480d6179e334a6455057a4ec084d3ef";
  }

  static const char* value(const ::roborts_msgs::LocationInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4480d6179e334a64ULL;
  static const uint64_t static_value2 = 0x55057a4ec084d3efULL;
};

template<class ContainerAllocator>
struct DataType< ::roborts_msgs::LocationInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "roborts_msgs/LocationInfo";
  }

  static const char* value(const ::roborts_msgs::LocationInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::roborts_msgs::LocationInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool if_relocation\n"
"float64 point_cloud_quantity\n"
"float64 tranDist\n"
"float64 angleDist\n"
"float64 angle_apeed\n"
"float64 score\n"
"bool if_match_success\n"
;
  }

  static const char* value(const ::roborts_msgs::LocationInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::roborts_msgs::LocationInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.if_relocation);
      stream.next(m.point_cloud_quantity);
      stream.next(m.tranDist);
      stream.next(m.angleDist);
      stream.next(m.angle_apeed);
      stream.next(m.score);
      stream.next(m.if_match_success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct LocationInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::roborts_msgs::LocationInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::roborts_msgs::LocationInfo_<ContainerAllocator>& v)
  {
    s << indent << "if_relocation: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.if_relocation);
    s << indent << "point_cloud_quantity: ";
    Printer<double>::stream(s, indent + "  ", v.point_cloud_quantity);
    s << indent << "tranDist: ";
    Printer<double>::stream(s, indent + "  ", v.tranDist);
    s << indent << "angleDist: ";
    Printer<double>::stream(s, indent + "  ", v.angleDist);
    s << indent << "angle_apeed: ";
    Printer<double>::stream(s, indent + "  ", v.angle_apeed);
    s << indent << "score: ";
    Printer<double>::stream(s, indent + "  ", v.score);
    s << indent << "if_match_success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.if_match_success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBORTS_MSGS_MESSAGE_LOCATIONINFO_H
