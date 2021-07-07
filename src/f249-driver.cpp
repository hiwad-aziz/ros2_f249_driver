#include <memory>
#include <wiringPi.h>

#include "f249-driver/f249-driver.h"

// Broadcom GPIO pin number
#define PIN 4

// Static variables and functions definition
size_t F249Driver::count_{ 0 };
F249Driver* F249Driver::pointer_to_self_{ nullptr };

F249Driver::F249Driver()
  : Node("f249publisher")
{
  publisher_ = this->create_publisher<std_msgs::msg::Int64>("f249", 10);

  // GPIO and interrupt setup
  wiringPiSetupGpio();
  pinMode(PIN, INPUT);
  wiringPiISR(PIN, INT_EDGE_FALLING, F249Driver::handleInput);
}

void
F249Driver::handleInput()
{
  auto message = std_msgs::msg::Int64();
  message.data = ++F249Driver::count_;
  if (F249Driver::count_ % 10 == 0) {
    RCLCPP_INFO(F249Driver::pointer_to_self_->get_logger(),
                "Publishing: '%d'",
                message.data);
  }
  F249Driver::pointer_to_self_->publisher_->publish(message);
}

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