#include "spin_slow_update.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlowSolution>());
  rclcpp::shutdown();
  return 0;
}

// constructor -- anything inside here runs when nodes starts (add publisher + subscriber)
SlowSolution::SlowSolution() : Node("slowsolution") {
  // publisher
  publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("predictedpos", 10);
  
  // subscriber
  subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/measuredpos", 10, std::bind(&SlowSolution::callback_function, this, std::placeholders::_1)
);
}

void SlowSolution::callback_function(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    // callback function
    // extract x and y coordinates -- read x and y from incoming msg
    double x = msg->data[0];
    double y = msg->data[1];

    // create new msg for float64multiarray
    auto predicted_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
    
    // assign + set predicted msg values
    predicted_msg->data.push_back(x);
    predicted_msg->data.push_back(y);

    publisher_->publish(*predicted_msg);

    RCLCPP_INFO(this->get_logger(), "Republish x: %f, y: %f", x, y);
    // debugger print statement
}
