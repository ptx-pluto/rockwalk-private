// Generated by gencpp from file rosserial_arduino/TestRequest.msg
// DO NOT EDIT!


#ifndef ROSSERIAL_ARDUINO_MESSAGE_TESTREQUEST_H
#define ROSSERIAL_ARDUINO_MESSAGE_TESTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace rosserial_arduino
{
template <class ContainerAllocator>
struct TestRequest_
{
  typedef TestRequest_<ContainerAllocator> Type;

  TestRequest_()
    : input()  {
    }
  TestRequest_(const ContainerAllocator& _alloc)
    : input(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _input_type;
  _input_type input;




  typedef boost::shared_ptr< ::rosserial_arduino::TestRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rosserial_arduino::TestRequest_<ContainerAllocator> const> ConstPtr;

}; // struct TestRequest_

typedef ::rosserial_arduino::TestRequest_<std::allocator<void> > TestRequest;

typedef boost::shared_ptr< ::rosserial_arduino::TestRequest > TestRequestPtr;
typedef boost::shared_ptr< ::rosserial_arduino::TestRequest const> TestRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rosserial_arduino::TestRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rosserial_arduino::TestRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rosserial_arduino

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'rosserial_arduino': ['/home/nazir/ws_moveit/src/rosserial/rosserial_arduino/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rosserial_arduino::TestRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rosserial_arduino::TestRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosserial_arduino::TestRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rosserial_arduino::TestRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosserial_arduino::TestRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rosserial_arduino::TestRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rosserial_arduino::TestRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "39e92f1778057359c64c7b8a7d7b19de";
  }

  static const char* value(const ::rosserial_arduino::TestRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x39e92f1778057359ULL;
  static const uint64_t static_value2 = 0xc64c7b8a7d7b19deULL;
};

template<class ContainerAllocator>
struct DataType< ::rosserial_arduino::TestRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rosserial_arduino/TestRequest";
  }

  static const char* value(const ::rosserial_arduino::TestRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rosserial_arduino::TestRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string input\n\
";
  }

  static const char* value(const ::rosserial_arduino::TestRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rosserial_arduino::TestRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.input);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TestRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rosserial_arduino::TestRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rosserial_arduino::TestRequest_<ContainerAllocator>& v)
  {
    s << indent << "input: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.input);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROSSERIAL_ARDUINO_MESSAGE_TESTREQUEST_H
