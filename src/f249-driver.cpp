#include <chrono>
#include <memory>
#include <string>
#include <wiringPi.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

// Broadcom GPIO pin number
#define PIN 4

using namespace std::chrono_literals;

class F249Driver : public rclcpp::Node
{
public:
  F249Driver()
    : Node("f249publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int64>("f249", 10);

    // GPIO and interrupt setup
    wiringPiSetupGpio();
    pinMode(PIN, INPUT);
    wiringPiISR(PIN, INT_EDGE_FALLING, F249Driver::handleInput);
  }

  // This construct of static members is required because wiringPiISR can't
  // receive a non-static member function using std::bind. Static member
  // functions do not have a this parameter so we have to keep a copy of a
  // pointer to itself after creating an instance to be able to use member
  // functions in the callback.
  // See also: https://isocpp.org/wiki/faq/pointers-to-members#memfnptr-vs-fnptr
  static size_t count_;
  static F249Driver* pointer_to_self_;

private:
  static void handleInput()
  {
    auto message = std_msgs::msg::Int64();
    message.data = F249Driver::count_++;
    if (F249Driver::count_ % 10 == 0) {
      RCLCPP_INFO(F249Driver::pointer_to_self_->get_logger(),
                  "Publishing: '%d'",
                  message.data);
    }
    F249Driver::pointer_to_self_->publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
};

// Static variables and functions definition
size_t F249Driver::count_{ 0 };
F249Driver* F249Driver::pointer_to_self_{ nullptr };

int
main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<F249Driver> driver{ std::make_shared<F249Driver>() };
  F249Driver::pointer_to_self_ = driver.get();
  RCLCPP_INFO(F249Driver::pointer_to_self_->get_logger(), "Start spinning...");
  rclcpp::spin(driver);
  rclcpp::shutdown();
  return 0;
}