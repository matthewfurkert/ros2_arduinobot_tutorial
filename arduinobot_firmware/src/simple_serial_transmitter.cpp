#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "libserial/SerialPort.h"

using std::placeholders::_1;

class SimpleSerialTransmitter : public rclcpp::Node
{
public:
    SimpleSerialTransmitter() : Node("simple_serial_transmitter")
    {
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        std::string port_ = get_parameter("port").as_string();
        
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "serial_transmitter", 10, std::bind(&SimpleSerialTransmitter::topic_callback, this, _1));
        
        arudino_.Open(port_);
        arudino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    ~SimpleSerialTransmitter()
    {
        arudino_.Close();
    }

private:
  void topic_callback(const std_msgs::msg::String &msg)
  {
    RCLCPP_INFO(get_logger(), "New message received, publishing on serial port: %s", msg.data.c_str());
    arudino_.Write(msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  LibSerial::SerialPort arudino_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSerialTransmitter>());
  rclcpp::shutdown();
  return 0;
}