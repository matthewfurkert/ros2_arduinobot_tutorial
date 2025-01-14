import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__("simple_parameter")
        self.declare_parameter("simple_int_param", 28)
        self.declare_parameter("simple_string_param", "Matthew")
        
        self.add_on_set_parameters_callback(self.param_change_callback)
        
    def param_change_callback(self, params):
        result = SetParametersResult()
        
        for param in params:
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f"Param simple_int_param changed! New value is {param.value}")
                result.successful = True
            if param.name == "simple_string_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info(f"Param simple_string_param changed! New value is {param.value}")
                result.successful = True
        
        return result

def main():
    rclpy.init()
    node = SimpleParameter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()