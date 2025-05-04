#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include "std_msgs/msg/int32.hpp"

class Listener : public rclcpp::Node {
public:
  Listener()
  : Node("ros2_listener")
  {
    // Costruisco un profilo QoS con Best Effort
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                              .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "micro_ros_pub_topic",
      qos_profile,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Ricevuto dall'ESP32: %d", msg->data);
      }
    );
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}

