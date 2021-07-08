#include "f249-driver/f249-driver.h"

#include <wiringPi.h>

#include <iostream>
#include <memory>

// Static variables and functions definition
size_t F249Driver::count_{0};
size_t F249Driver::previous_count_{0};
TimePoint F249Driver::time_{};
TimePoint F249Driver::previous_time_{};
F249Driver* F249Driver::pointer_to_self_{nullptr};

// Constructor
F249Driver::F249Driver() : Node("f249publisher")
{
  publisher_ = this->create_publisher<std_msgs::msg::Int64>("f249", 10);

  // GPIO and interrupt setup
  wiringPiSetupGpio();
  pinMode(kPin, INPUT);
  wiringPiISR(kPin, INT_EDGE_FALLING, F249Driver::handleInput);
}

// Callback function for interrupt
void F249Driver::handleInput()
{
  auto message = std_msgs::msg::Int64();
  // Update timestamps
  F249Driver::time_ = Clock::now();
  std::chrono::duration<double> dt = F249Driver::time_ - F249Driver::previous_time_;
  ++F249Driver::count_;
  // Only publish every 0.5s to get a better average estimate of the RPM
  if (dt.count() > kMinPublishTime) {
    // Fill message and publish
    message.data = pointer_to_self_->calculateRpm(F249Driver::count_ - F249Driver::previous_count_,
                                                  dt.count());
    RCLCPP_INFO(F249Driver::pointer_to_self_->get_logger(), "Publishing: '%d'", message.data);
    F249Driver::pointer_to_self_->publisher_->publish(message);
    // Update previous time to current time
    F249Driver::previous_time_ = F249Driver::time_;
    // Update tick count
    F249Driver::previous_count_ = F249Driver::count_;
  }
}

int F249Driver::calculateRpm(int dcount, double dt)
{
  if (dt != 0) {
    return static_cast<double>(dcount) / (kNumSlots * (dt / 60));
  }
  return 0;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<F249Driver> driver{std::make_shared<F249Driver>()};
  F249Driver::pointer_to_self_ = driver.get();
  RCLCPP_INFO(F249Driver::pointer_to_self_->get_logger(), "Start spinning...");
  rclcpp::spin(driver);
  rclcpp::shutdown();
  return 0;
}