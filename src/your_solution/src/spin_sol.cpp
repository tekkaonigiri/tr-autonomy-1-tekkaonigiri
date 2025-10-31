#include "spin_sol.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpinSolution>());
  rclcpp::shutdown();
  return 0;
}

SpinSolution::SpinSolution() : Node("spinsolution") {
  publisher_ = this->create_publisher<ArrayMsg>("predictedpos", 10);
  
  // need two subscribers -- /measuredpos and /measuredvel
  // need two callbacks, one for each subscriber
  subscription_pos_ = this->create_subscription<ArrayMsg>("/measuredpos", 10, std::bind(&SpinSolution::callback_pos, this, std::placeholders::_1)
  );
  subscription_vel_ = this->create_subscription<ArrayMsg>("/measuredvel", 10, std::bind(&SpinSolution::callback_vel, this, std::placeholders::_1)
  );
  
  // ros2 function timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1),
    std::bind(&SpinSolution::callback_timer, this)
  );
  
  // member var last_pred_timestamp that gets current ROS time from clock
  // need for Delta t
  last_pred_timestamp_ = this->get_clock()->now();
}

void SpinSolution::callback_pos(const ArrayMsg::SharedPtr msg) {
    // store latest position of x and y coordinates
    x_pos_last = msg->data[0];
    y_pos_last = msg->data[1];

    x_old = x_pos_last;
    y_old = y_pos_last;
    last_pred_timestamp_ = this->get_clock()->now();
    
}

void SpinSolution::callback_vel(const ArrayMsg::SharedPtr msg) {
    // store latest velocity of x and y coords
    x_vel_last = msg->data[0];
    y_vel_last = msg->data[1];
}

// no msg params for this guy --> triggered by timer
void SpinSolution::callback_timer() {
    auto now = this->get_clock()->now();
    double delta = (now - last_pred_timestamp_).seconds();
    double x_pred = x_old + delta * x_vel_last;
    double y_pred = y_old + delta * y_vel_last;
    
    last_pred_timestamp_ = now;
    x_old = x_pred;
    y_old = y_pred;
    
    // create new msg for float64multiarray
    // same as part 1 kinda
    auto predicted_msg = std::make_shared<ArrayMsg>();
    predicted_msg->data.push_back(x_pred);
    predicted_msg->data.push_back(y_pred);
    publisher_->publish(*predicted_msg);
    
    // debug
    RCLCPP_INFO(this->get_logger(), "Publishing predictedpos: x=%.2f y=%.2f", x_pred, y_pred);
}
