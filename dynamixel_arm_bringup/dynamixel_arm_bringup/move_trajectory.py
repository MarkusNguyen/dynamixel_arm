#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from control_msgs.action import FollowJointTrajectory


class ArmCartesianActionNode(Node):
    def __init__(self):
        super().__init__('arm_cartesian_action_node')

        # 1. Service Client for MoveIt Cartesian Path Planner
        self.cartesian_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path'
        )
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_cartesian_path service...')

        # 2. Action Client for ros2_control Trajectory Execution
        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for /arm_controller/follow_joint_trajectory action server...')

        # Trigger trajectory planning after spin() starts (0.5s delay)
        self.timer = self.create_timer(0.5, self.plan_and_execute_trajectory)

    def plan_and_execute_trajectory(self):
        # Destroy timer so it only executes once
        self.destroy_timer(self.timer)

        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.header.stamp = self.get_clock().now().to_msg()

        # FIX 1: Set is_diff = True so MoveIt uses the current live joint positions
        req.start_state.is_diff = True

        # Velocity & Acceleration scaling
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        
        # SRDF Group and Link Matching
        req.group_name = 'arm'
        req.link_name = 'link_5_1'

        req.max_step = 0.001          # 5 mm step resolution
        req.jump_threshold = 0.0       # IK jump threshold scaling factor
        req.avoid_collisions = True    # Keep collision checking enabled
        
        x1, y1, z1 = 0.03, 0.03, 0.03     # Target Cartesian coordinates

        # Define Cartesian Waypoints
        w1 = Pose()
        w1.position.x = x1
        w1.position.y = y1
        w1.position.z = z1
        
        w1.orientation.x = 0.0
        w1.orientation.y = 0.0
        w1.orientation.z = 0.0
        w1.orientation.w = 1.0         # Valid identity unit quaternion

        req.waypoints = [w1]

        # Request path from MoveIt
        self.get_logger().info(f'Requesting Cartesian path from MoveIt x={x1}, y={y1}, z={z1}...')
        future = self.cartesian_client.call_async(req)
        future.add_done_callback(self.cartesian_callback)

    def cartesian_callback(self, future):
        try:
            response = future.result()
            # Reject execution if path is less than 95% complete
            if response.fraction >= 0.95:
                trajectory = response.solution.joint_trajectory

                # FIX 2: Set timestamp to 0 so ros2_control executes immediately upon arrival
                trajectory.header.stamp.sec = 0
                trajectory.header.stamp.nanosec = 0

                # FIX 3: Ensure velocity and acceleration fields match joint count to avoid rejection
                num_joints = len(trajectory.joint_names)
                for pt in trajectory.points:
                    if not pt.velocities or len(pt.velocities) != num_joints:
                        pt.velocities = [0.0] * num_joints
                    if not pt.accelerations or len(pt.accelerations) != num_joints:
                        pt.accelerations = [0.0] * num_joints

                self.get_logger().info(
                    f'Path planned successfully ({response.fraction * 100:.1f}%). Sending action goal...'
                )

                # Construct Action Goal
                goal_msg = FollowJointTrajectory.Goal()
                goal_msg.trajectory = trajectory

                # Send goal asynchronously
                send_goal_future = self.action_client.send_goal_async(
                    goal_msg, 
                    feedback_callback=self.feedback_callback
                )
                send_goal_future.add_done_callback(self.goal_response_callback)
            else:
                self.get_logger().warn(
                    f'Path incomplete: Only {response.fraction * 100:.1f}% reachable. Execution aborted.'
                )
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Action goal rejected by controller!')
            return

        self.get_logger().info('Action goal accepted! Executing trajectory...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        current_positions = feedback_msg.feedback.actual.positions
        self.get_logger().debug(f'Arm moving... positions: {current_positions}')

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('Trajectory execution completed successfully!')
        else:
            self.get_logger().error(f'Trajectory execution failed with error code: {result.error_code}')


def main(args=None):
    rclpy.init(args=args)
    node = ArmCartesianActionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()