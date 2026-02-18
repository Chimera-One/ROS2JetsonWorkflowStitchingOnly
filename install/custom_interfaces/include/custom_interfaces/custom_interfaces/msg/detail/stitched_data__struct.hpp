// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:msg/StitchedData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__STITCHED_DATA__STRUCT_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__STITCHED_DATA__STRUCT_HPP_

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
# define DEPRECATED__custom_interfaces__msg__StitchedData __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__msg__StitchedData __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StitchedData_
{
  using Type = StitchedData_<ContainerAllocator>;

  explicit StitchedData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rotation_degree = 0.0;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->numberofimages = 0ll;
    }
  }

  explicit StitchedData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rotation_degree = 0.0;
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->numberofimages = 0ll;
    }
  }

  // field types and members
  using _rotation_degree_type =
    double;
  _rotation_degree_type rotation_degree;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    double;
  _altitude_type altitude;
  using _numberofimages_type =
    int64_t;
  _numberofimages_type numberofimages;
  using _image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _image_type image;

  // setters for named parameter idiom
  Type & set__rotation_degree(
    const double & _arg)
  {
    this->rotation_degree = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__numberofimages(
    const int64_t & _arg)
  {
    this->numberofimages = _arg;
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
    custom_interfaces::msg::StitchedData_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::msg::StitchedData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::msg::StitchedData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::msg::StitchedData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::StitchedData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::StitchedData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::StitchedData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::StitchedData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::msg::StitchedData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::msg::StitchedData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__msg__StitchedData
    std::shared_ptr<custom_interfaces::msg::StitchedData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__msg__StitchedData
    std::shared_ptr<custom_interfaces::msg::StitchedData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StitchedData_ & other) const
  {
    if (this->rotation_degree != other.rotation_degree) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->numberofimages != other.numberofimages) {
      return false;
    }
    if (this->image != other.image) {
      return false;
    }
    return true;
  }
  bool operator!=(const StitchedData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StitchedData_

// alias to use template instance with default allocator
using StitchedData =
  custom_interfaces::msg::StitchedData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__STITCHED_DATA__STRUCT_HPP_
