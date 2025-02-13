import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from arduinobot_interfaces.action import Fibonacci

class SimpleActionClient(Node):
    def __init__(self):
        super().__init__("simple_action_client")
        self.action_client_ = ActionClient(self, Fibonacci, "fibonacci")

        self.action_client_.wait_for_server()

        self.goal = Fibonacci.Goal()
        self.goal.order = 5

        self.future = self.action_client_.send_goal_async(self.goal, feedback_callback=self.feedbackCallback)
        self.future.add_done_callback(self.responseCallback)
    
    def responseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal Rejected")
            return
        
        self.get_logger().info("Goal Accepted")
        self.future = goal_handle.get_result_async()
        self.future.add_done_callback(self.resultCallback)
    
    def resultCallback(self, future):
        result = future.result().result
        self.get_logger().info("Result: {0}".format(result.sequence))
        rclpy.shutdown()
    

    def feedbackCallback(self, feedback_msg):
        self.get_logger().info("Feedback: {0}".format(feedback_msg.feedback.partial_sequence))

def main():
    rclpy.init()
    node = SimpleActionClient()
    rclpy.spin(node)

if __name__ == "__main__":
    main()