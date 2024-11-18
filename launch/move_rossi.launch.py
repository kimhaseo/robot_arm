import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.robot = MoveGroupCommander('robot_arm')
        self.robot.set_planning_time(10)

    def move_to_goal(self, goal_pose):
        self.robot.set_pose_target(goal_pose)
        plan = self.robot.go(wait=True)

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()

    goal_pose = Pose()
    goal_pose.position.x = 0.5  # Example x-coordinate
    goal_pose.position.y = 0.5  # Example y-coordinate
    goal_pose.position.z = 0.5  # Example z-coordinate
    # Set the orientation as needed
    goal_pose.orientation.x = 0.0  # Example x-orientation
    goal_pose.orientation.y = 0.0  # Example y-orientation
    goal_pose.orientation.z = 0.0  # Example z-orientation
    goal_pose.orientation.w = 1.0  # Example w-orientation

    node.move_to_goal(goal_pose)

    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()