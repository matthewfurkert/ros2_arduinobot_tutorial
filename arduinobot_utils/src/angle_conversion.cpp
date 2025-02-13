#include "rclcpp/rclcpp.hpp"
#include "arduinobot_interfaces/srv/euler_to_quaternion.hpp"
#include "arduinobot_interfaces/srv/quaternion_to_euler.hpp"
#include <tf2/utils.hpp>

using namespace std::placeholders;

class AnglesConverter : public rclcpp::Node
{
public:
  AnglesConverter() : Node("angle_conversion_service")
  {
    // Create a service that will use the 'add_two_ints' service type
    euler_to_quaternion_ = this->create_service<arduinobot_interfaces::srv::EulerToQuaternion>("euler_to_quaternion", std::bind(&AnglesConverter::eulerToQuaternionCallback, this, _1, _2));
    quaternion_to_euler_ = this->create_service<arduinobot_interfaces::srv::QuaternionToEuler>("quaternion_to_euler", std::bind(&AnglesConverter::quaternionToEulerCallback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Angle conversion services ready.");
  }
private:
    void eulerToQuaternionCallback(const std::shared_ptr<arduinobot_interfaces::srv::EulerToQuaternion::Request> request,
                                    const std::shared_ptr<arduinobot_interfaces::srv::EulerToQuaternion::Response> response)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received request to convert euler angles roll: " << request->roll << " pitch: " << request->pitch << " yaw: " << request->yaw);
        tf2::Quaternion q;
        q.setRPY(request->roll, request->pitch, request->yaw);
        response->x = q.x();
        response->y = q.y();
        response->z = q.z();
        response->w = q.w();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Converted quaternion x: " << response->x << " y: " << response->y << " z: " << response->z << " w: " << response->w);
    }
    
    void quaternionToEulerCallback(const std::shared_ptr<arduinobot_interfaces::srv::QuaternionToEuler::Request> request,
                                    const std::shared_ptr<arduinobot_interfaces::srv::QuaternionToEuler::Response> response)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received request to convert quaternion x: " << request->x << " y: " << request->y << " z: " << request->z << " w: " << request->w);
        tf2::Quaternion q(request->x, request->y, request->z, request->w);
        tf2::Matrix3x3 m(q);
        m.getRPY(response->roll, response->pitch, response->yaw);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Converted euler angles roll: " << response->roll << " pitch: " << response->pitch << " yaw: " << response->yaw);
    }


    rclcpp::Service<arduinobot_interfaces::srv::EulerToQuaternion>::SharedPtr euler_to_quaternion_;
    rclcpp::Service<arduinobot_interfaces::srv::QuaternionToEuler>::SharedPtr quaternion_to_euler_;
    };

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AnglesConverter>());
  rclcpp::shutdown();
  return 0;
}