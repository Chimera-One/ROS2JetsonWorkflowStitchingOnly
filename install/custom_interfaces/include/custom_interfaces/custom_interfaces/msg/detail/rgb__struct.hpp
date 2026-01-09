// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:msg/RGB.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__RGB__STRUCT_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__RGB__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interfaces__msg__RGB __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__msg__RGB __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RGB_
{
  using Type = RGB_<ContainerAllocator>;

  explicit RGB_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->gps_latitude = 0.0;
      this->gps_longitude = 0.0;
      this->gps_altitude = 0.0;
      this->gps_signal_level = 0l;
      this->compass = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
      this->gimbal_roll = 0.0;
      this->gimbal_pitch = 0.0;
      this->gimbal_yaw = 0.0;
    }
  }

  explicit RGB_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    image(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->gps_latitude = 0.0;
      this->gps_longitude = 0.0;
      this->gps_altitude = 0.0;
      this->gps_signal_level = 0l;
      this->compass = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
      this->gimbal_roll = 0.0;
      this->gimbal_pitch = 0.0;
      this->gimbal_yaw = 0.0;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _gps_latitude_type =
    double;
  _gps_latitude_type gps_latitude;
  using _gps_longitude_type =
    double;
  _gps_longitude_type gps_longitude;
  using _gps_altitude_type =
    double;
  _gps_altitude_type gps_altitude;
  using _gps_signal_level_type =
    int32_t;
  _gps_signal_level_type gps_signal_level;
  using _compass_type =
    double;
  _compass_type compass;
  using _roll_type =
    double;
  _roll_type roll;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _gimbal_roll_type =
    double;
  _gimbal_roll_type gimbal_roll;
  using _gimbal_pitch_type =
    double;
  _gimbal_pitch_type gimbal_pitch;
  using _gimbal_yaw_type =
    double;
  _gimbal_yaw_type gimbal_yaw;
  using _image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _image_type image;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__gps_latitude(
    const double & _arg)
  {
    this->gps_latitude = _arg;
    return *this;
  }
  Type & set__gps_longitude(
    const double & _arg)
  {
    this->gps_longitude = _arg;
    return *this;
  }
  Type & set__gps_altitude(
    const double & _arg)
  {
    this->gps_altitude = _arg;
    return *this;
  }
  Type & set__gps_signal_level(
    const int32_t & _arg)
  {
    this->gps_signal_level = _arg;
    return *this;
  }
  Type & set__compass(
    const double & _arg)
  {
    this->compass = _arg;
    return *this;
  }
  Type & set__roll(
    const double & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__gimbal_roll(
    const double & _arg)
  {
    this->gimbal_roll = _arg;
    return *this;
  }
  Type & set__gimbal_pitch(
    const double & _arg)
  {
    this->gimbal_pitch = _arg;
    return *this;
  }
  Type & set__gimbal_yaw(
    const double & _arg)
  {
    this->gimbal_yaw = _arg;
    return *this;
  }
  Type & set__image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->image = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::msg::RGB_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::msg::RGB_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::msg::RGB_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::msg::RGB_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::RGB_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::RGB_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::RGB_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::RGB_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::msg::RGB_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::msg::RGB_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__msg__RGB
    std::shared_ptr<custom_interfaces::msg::RGB_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__msg__RGB
    std::shared_ptr<custom_interfaces::msg::RGB_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RGB_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->gps_latitude != other.gps_latitude) {
      return false;
    }
    if (this->gps_longitude != other.gps_longitude) {
      return false;
    }
    if (this->gps_altitude != other.gps_altitude) {
      return false;
    }
    if (this->gps_signal_level != other.gps_signal_level) {
      return false;
    }
    if (this->compass != other.compass) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->gimbal_roll != other.gimbal_roll) {
      return false;
    }
    if (this->gimbal_pitch != other.gimbal_pitch) {
      return false;
    }
    if (this->gimbal_yaw != other.gimbal_yaw) {
      return false;
    }
    if (this->image != other.image) {
      return false;
    }
    return true;
  }
  bool operator!=(const RGB_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RGB_

// alias to use template instance with default allocator
using RGB =
  custom_interfaces::msg::RGB_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__RGB__STRUCT_HPP_
