#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

class F249Driver : public rclcpp::Node
{
public:
  F249Driver();

  // This construct of static members is required because wiringPiISR can't
  // receive a non-static member function using std::bind. Static member
  // functions do not have a this parameter so we have to keep a copy of a
  // pointer to itself after creating an instance to be able to use member
  // functions in the callback.
  // See also: https://isocpp.org/wiki/faq/pointers-to-members#memfnptr-vs-fnptr
  static size_t count_;
  static F249Driver* pointer_to_self_;

private:
  static void handleInput();
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
};