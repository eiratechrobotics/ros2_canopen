from geometry_msgs.msg import PoseStamped
from robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to poses.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.w = 1.0
    initial_pose.pose.orientation.z = 0.0

    goal_poses = []

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 2.0
    goal_pose1.pose.position.y = 0.0
    goal_pose1.pose.orientation.w = 1.0
    goal_pose1.pose.orientation.z = 0.0

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 2.0
    goal_pose2.pose.position.y = 2.0
    goal_pose2.pose.orientation.w = 0.71
    goal_pose2.pose.orientation.z = 0.71

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id = 'map'
    goal_pose3.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x = 0.0
    goal_pose3.pose.position.y = 2.0
    goal_pose3.pose.orientation.w = 0.0
    goal_pose3.pose.orientation.z = 1.0

    goal_pose4 = PoseStamped()
    goal_pose4.header.frame_id = 'map'
    goal_pose4.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose4.pose.position.x = 0.0
    goal_pose4.pose.position.y = 0.0
    goal_pose4.pose.orientation.w = -0.71
    goal_pose4.pose.orientation.z = 0.71

    for i in range(10):
        goal_poses.append(goal_pose1)
        goal_poses.append(goal_pose2)
        goal_poses.append(goal_pose3)
        goal_poses.append(goal_pose4)

    # sanity check a valid path exists
    path = navigator.getPath(initial_pose, goal_pose1)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)
    #navigator.goThroughPoses(goal_poses)
    #navigator.followPath(path)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

    #     # Do something with the feedback
        i = i + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and i % 5 == 0:
    #         print('Executing current waypoint: ' +
    #               str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
    #         now = navigator.get_clock().now()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')


    exit(0)


if __name__ == '__main__':
    main()