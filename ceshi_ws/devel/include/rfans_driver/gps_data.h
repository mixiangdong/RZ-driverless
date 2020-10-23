// Generated by gencpp from file rfans_driver/gps_data.msg
// DO NOT EDIT!


#ifndef RFANS_DRIVER_MESSAGE_GPS_DATA_H
#define RFANS_DRIVER_MESSAGE_GPS_DATA_H


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
struct gps_data_
{
  typedef gps_data_<ContainerAllocator> Type;

  gps_data_()
    : latitude(0.0)
    , longitude(0.0)
    , yaw(0.0)  {
    }
  gps_data_(const ContainerAllocator& _alloc)
    : latitude(0.0)
    , longitude(0.0)
    , yaw(0.0)  {
  (void)_alloc;
    }



   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef double _yaw_type;
  _yaw_type yaw;





  typedef boost::shared_ptr< ::rfans_driver::gps_data_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rfans_driver::gps_data_<ContainerAllocator> const> ConstPtr;

}; // struct gps_data_

typedef ::rfans_driver::gps_data_<std::allocator<void> > gps_data;

typedef boost::shared_ptr< ::rfans_driver::gps_data > gps_dataPtr;
typedef boost::shared_ptr< ::rfans_driver::gps_data const> gps_dataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rfans_driver::gps_data_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rfans_driver::gps_data_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::rfans_driver::gps_data_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rfans_driver::gps_data_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rfans_driver::gps_data_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rfans_driver::gps_data_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rfans_driver::gps_data_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rfans_driver::gps_data_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rfans_driver::gps_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1eb7a48b066869c948b4808bcc575127";
  }

  static const char* value(const ::rfans_driver::gps_data_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1eb7a48b066869c9ULL;
  static const uint64_t static_value2 = 0x48b4808bcc575127ULL;
};

template<class ContainerAllocator>
struct DataType< ::rfans_driver::gps_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rfans_driver/gps_data";
  }

  static const char* value(const ::rfans_driver::gps_data_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rfans_driver::gps_data_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
float64 latitude\n\
float64 longitude\n\
float64 yaw\n\
";
  }

  static const char* value(const ::rfans_driver::gps_data_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rfans_driver::gps_data_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.yaw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct gps_data_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rfans_driver::gps_data_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rfans_driver::gps_data_<ContainerAllocator>& v)
  {
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "yaw: ";
    Printer<double>::stream(s, indent + "  ", v.yaw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RFANS_DRIVER_MESSAGE_GPS_DATA_H
