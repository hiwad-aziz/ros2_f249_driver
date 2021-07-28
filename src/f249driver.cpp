#include "f249driver/f249driver.h"

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
  // Declare parameters and create publisher
  this->declare_parameter<int>("pin", 4);
  this->declare_parameter<int>("num_slots", 20);
  this->declare_parameter<double>("min_publish_time", 0.5);
  pin_ = this->get_parameter("pin").as_int();
  num_slots_ = this->get_parameter("num_slots").as_int();
  min_publish_time_ = this->get_parameter("min_publish_time").as_double();
  publisher_ = this->create_publisher<std_msgs::msg::Int64>("f249", 10);

  // GPIO and interrupt setup
  wiringPiSetupGpio();
  pinMode(pin_, INPUT);
  wiringPiISR(pin_, INT_EDGE_FALLING, F249Driver::handleInput);
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
  if (dt.count() > pointer_to_self_->min_publish_time_) {
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
    return static_cast<double>(dcount) / (num_slots_ * (dt / 60));
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