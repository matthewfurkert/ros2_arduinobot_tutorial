#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "arduinobot_interfaces/action/fibonacci.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::placeholders;

namespace arduinobot_cpp_examples
{

class SimpleActionServer : public rclcpp::Node
{
public:
  using Fibonacci = arduinobot_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  explicit SimpleActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("simple_action_server", options)
  {
    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&SimpleActionServer::handle_goal, this, _1, _2),
      std::bind(&SimpleActionServer::handle_cancel, this, _1),
      std::bind(&SimpleActionServer::handle_accepted, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "Simple Action Server has been started.");
  }

private:
    rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;
    
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        std::thread{std::bind(&SimpleActionServer::execute, this, _1), goal_handle}.detach();
    }
    
    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto &sequence = feedback->partial_sequence;
        sequence.push_back(0);
        sequence.push_back(1);
    
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = sequence;

        for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
        {
            if (goal_handle->is_canceling())
            {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
    
            sequence.push_back(sequence[i] + sequence[i - 1]);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");

            loop_rate.sleep();
        }
    
        if (rclcpp::ok())
        {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Result %d", result->sequence.back());
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }

   
    }
   
};

}  // namespace arduinobot_cpp_examples

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_cpp_examples::SimpleActionServer);