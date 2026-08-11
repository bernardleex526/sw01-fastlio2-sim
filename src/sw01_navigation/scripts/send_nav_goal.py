#!/usr/bin/env python3
"""向 Nav2 发送默认迷宫终点或命令行指定的 NavigateToPose 目标。"""

import argparse
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


EXIT_SUCCESS = 0
EXIT_ERROR = 1
EXIT_REJECTED = 2
EXIT_SERVER_UNAVAILABLE = 3
EXIT_CANCELED = 4
EXIT_ABORTED = 5
EXIT_UNKNOWN_STATUS = 6
EXIT_INTERRUPTED = 130
CANCEL_TIMEOUT_SECONDS = 5.0

STATUS_LABELS = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def parse_arguments(argv=None):
    """解析 map 坐标系中的 Nav2 终点参数。"""
    parser = argparse.ArgumentParser(
        description="Send a NavigateToPose goal to Nav2.",
    )
    # 来源：任务定义的 maze 目标默认坐标为 (12, 12, 0)。
    parser.add_argument("--x", type=float, default=12.0, help="Goal X coordinate in meters.")
    parser.add_argument("--y", type=float, default=12.0, help="Goal Y coordinate in meters.")
    parser.add_argument("--yaw", type=float, default=0.0, help="Goal yaw in radians.")
    # 来源：任务定义要求目标位姿发布在 map 坐标系。
    parser.add_argument("--frame", default="map", help="Goal pose frame ID (default: map).")
    parser.add_argument(
        "--server-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for /navigate_to_pose action server (default: 10).",
    )
    return parser.parse_args(argv)


def duration_seconds(duration):
    """将 ROS Duration 消息转换为秒，便于稳定地显示反馈。"""
    return duration.sec + duration.nanosec / 1_000_000_000.0


class NavGoalSender(Node):
    """封装 /navigate_to_pose 的发送、反馈和取消操作。"""

    def __init__(self):
        super().__init__("sw01_nav_goal_sender")
        # 话题来源：Nav2 Humble NavigateToPose action 端点 /navigate_to_pose。
        self._client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def build_goal(self, arguments):
        """创建指定 frame 中的 NavigateToPose 目标，并由 yaw 计算四元数。"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = arguments.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = arguments.x
        goal.pose.pose.position.y = arguments.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(arguments.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(arguments.yaw / 2.0)
        return goal

    def feedback_callback(self, feedback_message):
        """报告 NavigateToPose 的关键运行反馈。"""
        feedback = feedback_message.feedback
        self.get_logger().info(
            "feedback: distance_remaining=%.2f m, "
            "estimated_time_remaining=%.2f s, navigation_time=%.2f s, "
            "number_of_recoveries=%d"
            % (
                feedback.distance_remaining,
                duration_seconds(feedback.estimated_time_remaining),
                duration_seconds(feedback.navigation_time),
                feedback.number_of_recoveries,
            )
        )

    def cancel_goal(self, goal_handle):
        """请求取消已接收目标，并有界等待取消请求完成。"""
        self.get_logger().warning("KeyboardInterrupt received; canceling accepted goal.")
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(
            self, cancel_future, timeout_sec=CANCEL_TIMEOUT_SECONDS
        )
        if not cancel_future.done():
            self.get_logger().warning(
                "Timed out waiting %.1f seconds for cancellation acknowledgement.",
                CANCEL_TIMEOUT_SECONDS,
            )
            return

        cancel_response = cancel_future.result()
        if cancel_response.goals_canceling:
            self.get_logger().info("Cancellation request acknowledged.")
        else:
            self.get_logger().warning("Cancellation request was not acknowledged.")


def exit_code_for_status(status):
    """将 action_msgs GoalStatus 终态映射为进程退出码。"""
    if status == GoalStatus.STATUS_SUCCEEDED:
        return EXIT_SUCCESS
    if status == GoalStatus.STATUS_CANCELED:
        return EXIT_CANCELED
    if status == GoalStatus.STATUS_ABORTED:
        return EXIT_ABORTED
    return EXIT_UNKNOWN_STATUS


def main(argv=None):
    """等待 Nav2、发送目标、等待终态，并返回明确的退出码。"""
    arguments = parse_arguments(argv)
    rclpy.init(args=None)
    node = None
    goal_handle = None

    try:
        node = NavGoalSender()
        node.get_logger().info(
            "Waiting up to %.1f seconds for /navigate_to_pose.",
            arguments.server_timeout,
        )
        if not node._client.wait_for_server(timeout_sec=arguments.server_timeout):
            node.get_logger().error("Nav2 action server /navigate_to_pose is unavailable.")
            return EXIT_SERVER_UNAVAILABLE

        goal = node.build_goal(arguments)
        node.get_logger().info(
            "Sending goal: frame=%s, x=%.2f, y=%.2f, yaw=%.2f rad.",
            arguments.frame,
            arguments.x,
            arguments.y,
            arguments.yaw,
        )
        send_future = node._client.send_goal_async(
            goal, feedback_callback=node.feedback_callback
        )
        rclpy.spin_until_future_complete(node, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            node.get_logger().error("Goal was rejected by /navigate_to_pose.")
            return EXIT_REJECTED

        node.get_logger().info("Goal accepted; waiting for terminal result.")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        wrapped_result = result_future.result()
        status = wrapped_result.status
        status_label = STATUS_LABELS.get(status, "UNRECOGNIZED")
        node.get_logger().info("Navigation finished with status %s (%d).", status_label, status)
        return exit_code_for_status(status)
    except KeyboardInterrupt:
        if node is not None and goal_handle is not None and goal_handle.accepted:
            node.cancel_goal(goal_handle)
        return EXIT_INTERRUPTED
    except Exception as error:  # ROS futures can surface transport exceptions.
        message = "Navigation goal sender failed: %s" % error
        if node is None:
            print(message, file=sys.stderr)
        else:
            node.get_logger().error(message)
        return EXIT_ERROR
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
