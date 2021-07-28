#ifndef F249_DRIVER_H
#define F249_DRIVER_H

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;

class F249Driver : public rclcpp::Node {
 public:
  F249Driver();

  // This construct of static members is required because wiringPiISR can't
  // receive a non-static member function using std::bind. Static member
  // functions do not have a "this" parameter so we have to keep a copy of a
  // pointer to itself after creating an instance to be able to use member
  // functions in the callback.
  // See also: https://isocpp.org/wiki/faq/pointers-to-members#memfnptr-vs-fnptr
  static size_t count_;
  static size_t previous_count_;
  static TimePoint time_;
  static TimePoint previous_time_;
  static F249Driver* pointer_to_self_;

 private:
  static void handleInput();
  int calculateRpm(int dcount, double dt);
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
  int pin_;
  int num_slots_;
  double min_publish_time_;
};

#endif  // F249_DRIVER_H