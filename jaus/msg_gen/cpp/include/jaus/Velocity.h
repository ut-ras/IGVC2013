/* Auto-generated by genmsg_cpp for file /home/ras/IGVC2013/jaus/msg/Velocity.msg */
#ifndef JAUS_MESSAGE_VELOCITY_H
#define JAUS_MESSAGE_VELOCITY_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace jaus
{
template <class ContainerAllocator>
struct Velocity_ {
  typedef Velocity_<ContainerAllocator> Type;

  Velocity_()
  : xrate(0.0)
  , yrate(0.0)
  , zrate(0.0)
  , rollrate(0.0)
  , pitchrate(0.0)
  , yawrate(0.0)
  {
  }

  Velocity_(const ContainerAllocator& _alloc)
  : xrate(0.0)
  , yrate(0.0)
  , zrate(0.0)
  , rollrate(0.0)
  , pitchrate(0.0)
  , yawrate(0.0)
  {
  }

  typedef double _xrate_type;
  double xrate;

  typedef double _yrate_type;
  double yrate;

  typedef double _zrate_type;
  double zrate;

  typedef double _rollrate_type;
  double rollrate;

  typedef double _pitchrate_type;
  double pitchrate;

  typedef double _yawrate_type;
  double yawrate;


  typedef boost::shared_ptr< ::jaus::Velocity_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jaus::Velocity_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Velocity
typedef  ::jaus::Velocity_<std::allocator<void> > Velocity;

typedef boost::shared_ptr< ::jaus::Velocity> VelocityPtr;
typedef boost::shared_ptr< ::jaus::Velocity const> VelocityConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::jaus::Velocity_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::jaus::Velocity_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace jaus

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::jaus::Velocity_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::jaus::Velocity_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::jaus::Velocity_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8de9a9274155344335ec8e399f30e54a";
  }

  static const char* value(const  ::jaus::Velocity_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8de9a92741553443ULL;
  static const uint64_t static_value2 = 0x35ec8e399f30e54aULL;
};

template<class ContainerAllocator>
struct DataType< ::jaus::Velocity_<ContainerAllocator> > {
  static const char* value() 
  {
    return "jaus/Velocity";
  }

  static const char* value(const  ::jaus::Velocity_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::jaus::Velocity_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 xrate\n\
float64 yrate\n\
float64 zrate\n\
\n\
float64 rollrate\n\
float64 pitchrate\n\
float64 yawrate\n\
\n\
";
  }

  static const char* value(const  ::jaus::Velocity_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::jaus::Velocity_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::jaus::Velocity_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.xrate);
    stream.next(m.yrate);
    stream.next(m.zrate);
    stream.next(m.rollrate);
    stream.next(m.pitchrate);
    stream.next(m.yawrate);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Velocity_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jaus::Velocity_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::jaus::Velocity_<ContainerAllocator> & v) 
  {
    s << indent << "xrate: ";
    Printer<double>::stream(s, indent + "  ", v.xrate);
    s << indent << "yrate: ";
    Printer<double>::stream(s, indent + "  ", v.yrate);
    s << indent << "zrate: ";
    Printer<double>::stream(s, indent + "  ", v.zrate);
    s << indent << "rollrate: ";
    Printer<double>::stream(s, indent + "  ", v.rollrate);
    s << indent << "pitchrate: ";
    Printer<double>::stream(s, indent + "  ", v.pitchrate);
    s << indent << "yawrate: ";
    Printer<double>::stream(s, indent + "  ", v.yawrate);
  }
};


} // namespace message_operations
} // namespace ros

#endif // JAUS_MESSAGE_VELOCITY_H
