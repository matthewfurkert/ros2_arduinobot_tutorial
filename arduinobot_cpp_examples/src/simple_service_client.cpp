#include "rclcpp/rclcpp.hpp"
#include "arduinobot_interfaces/srv/add_two_ints.hpp"

using namespace std::placeholders;

class SimpleServiceClient : public rclcpp::Node
{
public:
  SimpleServiceClient(int a, int b) : Node("simple_service_client")
  {
    // Create a client that will use the 'add_two_ints' service type
    client_ = this->create_client<arduinobot_interfaces::srv::AddTwoInts>("add_two_ints");
    
    // Create a request message
    auto request = std::make_shared<arduinobot_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
      }
    
    // Call the service
    auto result = client_->async_send_request(request, std::bind(&SimpleServiceClient::responseCallback, this, _1));
  }
private:
    void responseCallback(rclcpp::Client<arduinobot_interfaces::srv::AddTwoInts>::SharedFuture future)
    {
        if (future.valid())
        {
            RCLCPP_INFO(this->get_logger(), "Service response: %ld", future.get()->sum);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get response from service");
        }
    }

    rclcpp::Client<arduinobot_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

    if (argc != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run arduinobot_cpp_examples simple_service_client <int> <int>");
        return 1;
    }

  std::make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2]));
  rclcpp::shutdown();
  return 0;
}