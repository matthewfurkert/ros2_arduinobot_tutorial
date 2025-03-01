#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "libserial/SerialPort.h"

using namespace std::chrono_literals;

class SimpleSerialReceiver : public rclcpp::Node
{
public:
    SimpleSerialReceiver() : Node("simple_serial_receiver")
    {
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        std::string port_ = get_parameter("port").as_string();

        publisher_ = this->create_publisher<std_msgs::msg::String>("serial_receiver", 10);
        timer_ = this->create_wall_timer(0.01s, std::bind(&SimpleSerialReceiver::timer_callback, this));
        arudino_.Open(port_);
        arudino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }
    ~SimpleSerialReceiver()
    {
        arudino_.Close();
    }
private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        if (rclcpp::ok() && arudino_.IsDataAvailable())
        {
            arudino_.ReadLine(message.data);
        }
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    LibSerial::SerialPort arudino_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSerialReceiver>());
  rclcpp::shutdown();
  return 0;
}