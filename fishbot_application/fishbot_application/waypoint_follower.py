from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    goal_poses = []
    goal_poses.append(PoseStamped())
    goal_poses.append(PoseStamped())
    goal_poses.append(PoseStamped())
    x = [0.0, 2.0, 2.0]
    y = [0.0, 0.0, 2.0]
    w = [1.0, 1.0, 1.0]
    for i in range(len(goal_poses)):
        goal_poses[i].header.frame_id = "map"
        goal_poses[i].pose.position.x = x[i]
        goal_poses[i].pose.position.y = y[i]
        goal_poses[i].pose.orientation.w = w[i]
        goal_poses[i].header.stamp = navigator.get_clock().now().to_msg()
    navigator.followWaypoints(goal_poses)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(f'当前目标编号：{feedback.current_waypoint}')
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('导航结果：成功')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info('导航结果：取消')
    elif result == TaskResult.FAILED:
        navigator.get_logger().info('导航结果：失败')
    else:
        navigator.get_logger().info('导航结果：返回状态无效')