#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node
{
public:
  SimpleSubscriber() : Node("simple_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&SimpleSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String &msg) const
  {
    RCLCPP_INFO(get_logger(), "I heard: %s", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}