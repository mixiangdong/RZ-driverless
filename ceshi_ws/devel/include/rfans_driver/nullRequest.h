// Generated by gencpp from file rfans_driver/nullRequest.msg
// DO NOT EDIT!


#ifndef RFANS_DRIVER_MESSAGE_NULLREQUEST_H
#define RFANS_DRIVER_MESSAGE_NULLREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rfans_driver
{
template <class ContainerAllocator>
struct nullRequest_
{
  typedef nullRequest_<ContainerAllocator> Type;

  nullRequest_()
    : state(false)  {
    }
  nullRequest_(const ContainerAllocator& _alloc)
    : state(false)  {
  (void)_alloc;
    }



   typedef uint8_t _state_type;
  _state_type state;





  typedef boost::shared_ptr< ::rfans_driver::nullRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rfans_driver::nullRequest_<ContainerAllocator> const> ConstPtr;

}; // struct nullRequest_

typedef ::rfans_driver::nullRequest_<std::allocator<void> > nullRequest;

typedef boost::shared_ptr< ::rfans_driver::nullRequest > nullRequestPtr;
typedef boost::shared_ptr< ::rfans_driver::nullRequest const> nullRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rfans_driver::nullRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rfans_driver::nullRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rfans_driver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'rfans_driver': ['/home/mixiangdong/ceshi_ws/src/rfans_driver/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rfans_driver::nullRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rfans_driver::nullRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rfans_driver::nullRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rfans_driver::nullRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rfans_driver::nullRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rfans_driver::nullRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rfans_driver::nullRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "001fde3cab9e313a150416ff09c08ee4";
  }

  static const char* value(const ::rfans_driver::nullRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x001fde3cab9e313aULL;
  static const uint64_t static_value2 = 0x150416ff09c08ee4ULL;
};

template<class ContainerAllocator>
struct DataType< ::rfans_driver::nullRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rfans_driver/nullRequest";
  }

  static const char* value(const ::rfans_driver::nullRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rfans_driver::nullRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
bool state\n\
";
  }

  static const char* value(const ::rfans_driver::nullRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rfans_driver::nullRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.state);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct nullRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rfans_driver::nullRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rfans_driver::nullRequest_<ContainerAllocator>& v)
  {
    s << indent << "state: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.state);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RFANS_DRIVER_MESSAGE_NULLREQUEST_H
