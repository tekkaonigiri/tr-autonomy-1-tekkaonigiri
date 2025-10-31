#ifndef YOUR_SOLUTION_SRC_SPIN_SOL_H_
#define YOUR_SOLUTION_SRC_SPIN_SOL_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using ArrayMsg = std_msgs::msg::Float64MultiArray;

class SpinSolution : public rclcpp::Node {
 public:
   SpinSolution();
 private:
   double x_pos_last = 0.0;
   double y_pos_last = 0.0;
   double x_vel_last = 0.0;
   double y_vel_last = 0.0;
   
   double x_old = 0.0;
   double y_old = 0.0;

   rclcpp::Publisher<ArrayMsg>::SharedPtr publisher_;
   rclcpp::Subscription<ArrayMsg>::SharedPtr subscription_pos_;
   rclcpp::Subscription<ArrayMsg>::SharedPtr subscription_vel_;

   rclcpp::Time last_pred_timestamp_;
   rclcpp::TimerBase::SharedPtr timer_;
   
   void callback_pos(const ArrayMsg::SharedPtr msg);
   void callback_vel(const ArrayMsg::SharedPtr msg);
   void callback_timer();
};

#endif //YOUR_SOLUTION_SRC_SPIN_SOL_H_
