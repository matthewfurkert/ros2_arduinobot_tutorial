#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "arduinobot_interfaces/action/fibonacci.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace arduinobot_cpp_examples
{
class SimpleActionClient : public rclcpp::Node
{
public:
  using Fibonacci = arduinobot_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit SimpleActionClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("simple_action_client", options)
  {
    action_client_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    timer_ = create_wall_timer(1s, std::bind(&SimpleActionClient::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "Simple Action Client has been started.");
  }
private:
  rclcpp_action::Client<Fibonacci>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timerCallback()
  {
    timer_->cancel();
    if (!action_client_->wait_for_action_server(10s)) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
    }
    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 8;
    RCLCPP_INFO(this->get_logger(), "Sending goal request with order %d", goal_msg.order);

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&SimpleActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&SimpleActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&SimpleActionClient::result_callback, this, _1);

    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

    void feedback_callback(
        GoalHandleFibonacci::SharedPtr,
        const std::shared_ptr<const Fibonacci::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Next number in sequence received: %d", feedback->partial_sequence.back());
    }

    void result_callback(const GoalHandleFibonacci::WrappedResult &result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Result received: %d", result.result->sequence.back());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        std::stringstream ss;
        for (auto number : result.result->sequence) {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "Result sequence: %s", ss.str().c_str());
        

        rclcpp::shutdown();
    }
};

}  // namespace arduinobot_cpp_examples

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_cpp_examples::SimpleActionClient)