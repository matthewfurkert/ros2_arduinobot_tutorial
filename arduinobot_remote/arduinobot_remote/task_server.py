#! /usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from arduinobot_interfaces.action import ArduinobotTask
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

class TaskServer(Node):
    def __init__(self):
        super().__init__("task_server")
        self.action_server_ = ActionServer(self, ArduinobotTask, "task_server", self.goalCallback)
        self.arduinobot_ = MoveItPy(node_name="moveit_py")
        self.arduinobot_arm_ = self.arduinobot_.get_planning_component("arm")
        self.arduinobot_gripper_ = self.arduinobot_.get_planning_component("gripper")
        self.get_logger().info("Starting the server")

    def goalCallback(self, goal_handle):
        self.get_logger().info("Received goal request with task_number %d" % goal_handle.request.task_number)
        arm_state = RobotState(self.arduinobot_.get_robot_model())
        gripper_state = RobotState(self.arduinobot_.get_robot_model())

        arm_joint_goal = []
        gripper_joint_goal = []

        if goal_handle.request.task_number == 0:
            arm_joint_goal = np.array([0.0, 0.0, 0.0])
            gripper_joint_goal = np.array([-0.7, 0.7])
        elif goal_handle.request.task_number == 1:
            arm_joint_goal = np.array([-1.14, -0.6, -0.07])
            gripper_joint_goal = np.array([0.0, 0.0])
        elif goal_handle.request.task_number == 2:
            arm_joint_goal = np.array([-1.57, 0.0, -0.9])
            gripper_joint_goal = np.array([0.0, 0.0])
        else:
            self.get_logger().error("Invalid task number")
            goal_handle.abort()
            return
        
        arm_state.set_joint_group_positions("arm", arm_joint_goal)
        arm_state.set_joint_group_positions("gripper", arm_joint_goal)

        self.arduinobot_arm_.set_start_state_to_current_state()
        self.arduinobot_gripper_.set_start_state_to_current_state()

        self.arduinobot_arm_.set_goal_state(robot_state=arm_state)
        self.arduinobot_gripper_.set_goal_state(robot_state=gripper_state)

        arm_plan_result = self.arduinobot_arm_.plan()
        gripper_plan_result = self.arduinobot_gripper_.plan()

        if arm_plan_result and gripper_plan_result:
            self.arduinobot_arm_.execute(arm_plan_result.trajectory, controllers=[])
            self.arduinobot_arm_.execute(gripper_plan_result.trajectory, controllers=[])
        else:
            self.get_logger().warn("Planning failed")
        
        goal_handle.succeed()
        result = ArduinobotTask.Result()
        result.success = True
        return result


def main():
    rclpy.init()
    node = TaskServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()