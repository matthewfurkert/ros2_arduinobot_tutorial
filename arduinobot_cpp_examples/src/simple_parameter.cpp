#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using std::placeholders::_1;

class SimpleParameter : public rclcpp::Node
{
public:
  SimpleParameter() : Node("simple_parameter")
  {
    this->declare_parameter<int>("simple_int_param", 42);
    this->declare_parameter<std::string>("simple_string_param", "Matthew");

    param_callback_handle_ = add_on_set_parameters_callback(std::bind(&SimpleParameter::parameterChangeCallback, this, _1));

  }
private:
    rcl_interfaces::msg::SetParametersResult parameterChangeCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &parameter : parameters)
        {
            if (parameter.get_name() == "simple_int_param" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                my_parameter = parameter.as_int();
                result.successful = true;
                RCLCPP_INFO(get_logger(), "Parameter changed. New value is: %d", my_parameter);
            }
            else if (parameter.get_name() == "simple_string_param" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
            {
                result.successful = true;
                RCLCPP_INFO(get_logger(), "Parameter changed. New value is: %s", parameter.as_string().c_str());
            }
            else
            {
                result.successful = false;
                RCLCPP_ERROR(get_logger(), "Parameter not found or type mismatch");
            }
        }
        return result;
    }

    int my_parameter;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleParameter>());
  rclcpp::shutdown();
  return 0;
}