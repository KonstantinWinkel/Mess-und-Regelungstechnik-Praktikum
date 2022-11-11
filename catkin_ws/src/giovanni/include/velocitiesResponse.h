// Generated by gencpp from file volksbot/velocitiesResponse.msg
// DO NOT EDIT!


#ifndef VOLKSBOT_MESSAGE_VELOCITIESRESPONSE_H
#define VOLKSBOT_MESSAGE_VELOCITIESRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace volksbot
{
template <class ContainerAllocator>
struct velocitiesResponse_
{
  typedef velocitiesResponse_<ContainerAllocator> Type;

  velocitiesResponse_()
    {
    }
  velocitiesResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::volksbot::velocitiesResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::volksbot::velocitiesResponse_<ContainerAllocator> const> ConstPtr;

}; // struct velocitiesResponse_

typedef ::volksbot::velocitiesResponse_<std::allocator<void> > velocitiesResponse;

typedef boost::shared_ptr< ::volksbot::velocitiesResponse > velocitiesResponsePtr;
typedef boost::shared_ptr< ::volksbot::velocitiesResponse const> velocitiesResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::volksbot::velocitiesResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::volksbot::velocitiesResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace volksbot

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::volksbot::velocitiesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::volksbot::velocitiesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::volksbot::velocitiesResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::volksbot::velocitiesResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::volksbot::velocitiesResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::volksbot::velocitiesResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::volksbot::velocitiesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::volksbot::velocitiesResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::volksbot::velocitiesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "volksbot/velocitiesResponse";
  }

  static const char* value(const ::volksbot::velocitiesResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::volksbot::velocitiesResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::volksbot::velocitiesResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::volksbot::velocitiesResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct velocitiesResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::volksbot::velocitiesResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::volksbot::velocitiesResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // VOLKSBOT_MESSAGE_VELOCITIESRESPONSE_H
