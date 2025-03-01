#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "arduinobot_interfaces/action/arduinobot_task.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

using namespace std::placeholders;

namespace arduinobot_remote
{

class TaskServer : public rclcpp::Node
{
public:
  using Task = arduinobot_interfaces::action::ArduinobotTask;
  using GoalHandleTask = rclcpp_action::ServerGoalHandle<Task>;

  explicit TaskServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("task_server", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    action_server_ = rclcpp_action::create_server<Task>(
      this,
      "task_server",
      std::bind(&TaskServer::handle_goal, this, _1, _2),
      std::bind(&TaskServer::handle_cancel, this, _1),
      std::bind(&TaskServer::handle_accepted, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "Task Server has been started.");
  }

private:
    rclcpp_action::Server<Task>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_;
    std::vector<double> arm_joint_goal_, gripper_joint_goal_;
    
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Task::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with task number: %d", goal->task_number);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        if (arm_move_group_)
        {
            arm_move_group_->stop();
        }
        if (gripper_move_group_)
        {
            gripper_move_group_->stop();
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        std::thread{std::bind(&TaskServer::execute, this, _1), goal_handle}.detach();
    }
    
    void execute(const std::shared_ptr<GoalHandleTask> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        auto result = std::make_shared<Task::Result>();

        if (!arm_move_group_)
        {
            arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        }
        if (!gripper_move_group_)
        {
            gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
        }
        

        if (goal_handle->get_goal()->task_number == 0){
            arm_joint_goal_ ={0.0, 0.0, 0.0};
            gripper_joint_goal_ = {-0.7, 0.7};
        } else if (goal_handle->get_goal()->task_number == 1){
            arm_joint_goal_ = {-1.14, -0.6, -0.07};
            gripper_joint_goal_ = {0.0, 0.0};
        } else if (goal_handle->get_goal()->task_number == 2){
            arm_joint_goal_ = {-1.57, 0.0, -0.9};
            gripper_joint_goal_ = {0.7, -0.7};
        } else if (goal_handle->get_goal()->task_number == 3){
            arm_joint_goal_ = {0.0, 0.0, 0.0};
            gripper_joint_goal_ = {0.0, 0.0};
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid task number");
            return;
        }

        arm_move_group_->setStartState(*arm_move_group_->getCurrentState());
        gripper_move_group_->setStartState(*gripper_move_group_->getCurrentState());

        bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
        bool gripper_within_bounds = gripper_move_group_->setJointValueTarget(gripper_joint_goal_);

        if (!arm_within_bounds | !gripper_within_bounds)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal is out of bounds");
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan, gripper_plan;
        bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        bool gripper_plan_success = (gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (arm_plan_success && gripper_plan_success)
        {
            RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arme and the gripper");
            arm_move_group_->move();
            gripper_move_group_->move();
        } else {
            RCLCPP_ERROR(this->get_logger(), "One or more planners failed");
            return;
        }

        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Goal succeeded");
    }
   
};

}  // namespace arduinobot_remote

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::TaskServer);