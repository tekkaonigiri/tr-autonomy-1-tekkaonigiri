#ifndef YOUR_SOLUTION_SRC_SPIN_SLOW_UPDATE_H_
#define YOUR_SOLUTION_SRC_SPIN_SLOW_UPDATE_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using ArrayMsg = std_msgs::msg::Float64MultiArray;

class SlowSolution : public rclcpp::Node {
 public:
  SlowSolution();
 private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;

void callback_function(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

#endif //YOUR_SOLUTION_SRC_SPIN_SLOW_UPDATE_H_
