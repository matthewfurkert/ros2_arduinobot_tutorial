#include "rclcpp/rclcpp.hpp"
#include "arduinobot_interfaces/srv/add_two_ints.hpp"

using namespace std::placeholders;

class SimpleServiceServer : public rclcpp::Node
{
public:
  SimpleServiceServer() : Node("simple_service_server")
  {
    // Create a service that will use the 'add_two_ints' service type
    service_ = this->create_service<arduinobot_interfaces::srv::AddTwoInts>(
      "add_two_ints", std::bind(&SimpleServiceServer::service_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Add two ints service is ready.");
  }
private:
    void service_callback(const std::shared_ptr<arduinobot_interfaces::srv::AddTwoInts::Request> request,
                    std::shared_ptr<arduinobot_interfaces::srv::AddTwoInts::Response> response)
    {
        // Add the two integers and store the result in the response
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "Incoming request\na: %ld\nb: %ld", request->a, request->b);
        RCLCPP_INFO(this->get_logger(), "Sending back response (sum): [%ld]", response->sum);
    }


    rclcpp::Service<arduinobot_interfaces::srv::AddTwoInts>::SharedPtr service_;
    };

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleServiceServer>());
  rclcpp::shutdown();
  return 0;
}