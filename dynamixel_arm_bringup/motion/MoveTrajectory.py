#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from moveit_configs_utils import MoveItConfigsBuilder
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped, Point

# MoveIt & Message imports
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_msgs.msg import Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive


class MoveTrajectory(Node):

    def __init__(self):
        super().__init__("move_trajectory")

        self.cb_group = ReentrantCallbackGroup()

        # Build MoveIt parameters
        moveit_config = (
            MoveItConfigsBuilder("dynamixel_arm", package_name="dynamixel_arm_moveit_config")
            .to_moveit_configs()
        )

        config_dict = moveit_config.to_dict()
        config_dict["planning_pipelines"] = {"pipeline_names": ["ompl"]}
        config_dict["plan_request_params"] = {
            "planning_pipeline": "ompl",
            "planner_id": "RRTConnectkConfigDefault",
            "planning_time": 5.0,
            "planning_attempts": 10,
            "max_velocity_scaling_factor": 0.025,
            "max_acceleration_scaling_factor": 0.025,
        }

        self.robot = MoveItPy(
            node_name="headless_planner", 
            config_dict=config_dict
        )
        self.arm = self.robot.get_planning_component("arm")

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            callback_group=self.cb_group
        )

        self._target_sub = self.create_subscription(
            Point,
            "/target_xyz",
            self.target_xyz_callback,
            10,
            callback_group=self.cb_group
        )

        self.is_busy = False
        self.get_logger().info("MoveTrajectory node ready! Listening on topic: /target_xyz")

    async def target_xyz_callback(self, msg: Point):
        """Callback triggered when a new target point is received."""
        if self.is_busy:
            self.get_logger().warn("Currently executing another motion. Target ignored.")
            return

        self.is_busy = True
        self.get_logger().info(f"Received new target point: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}")
        
        await self.plan_and_execute_xyz(msg.x, msg.y, msg.z)
        
        self.is_busy = False

    def _create_position_constraint(
        self, link_name: str, frame_id: str, x: float, y: float, z: float, tolerance: float = 0.015
    ) -> Constraints:
        """Creates a pure position goal constraint without orientation restriction."""
        goal_constraints = Constraints()
        goal_constraints.name = "position_only_goal"

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = frame_id
        pos_constraint.link_name = link_name

        box_primitive = SolidPrimitive()
        box_primitive.type = SolidPrimitive.BOX
        box_primitive.dimensions = [tolerance * 2, tolerance * 2, tolerance * 2]

        target_pose = PoseStamped()
        target_pose.header.frame_id = frame_id
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)

        pos_constraint.constraint_region.primitives.append(box_primitive)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
        pos_constraint.weight = 1.0

        goal_constraints.position_constraints.append(pos_constraint)
        return goal_constraints

    async def send_trajectory_goal(self, joint_trajectory: JointTrajectory):
        """Sends the joint trajectory goal asynchronously."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = joint_trajectory

        self.get_logger().info("Waiting for action server...")
        
        # FIXED: Removed 'await' because wait_for_server() returns a boolean
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server '/arm_controller/follow_joint_trajectory' not available!")
            return None

        self.get_logger().info("Sending trajectory goal to controller...")
        return await self._action_client.send_goal_async(goal_msg)

    async def plan_and_execute_xyz(self, x: float, y: float, z: float) -> bool:
        """Plans and executes trajectory asynchronously."""
        self.arm.set_start_state_to_current_state()

        position_goal = self._create_position_constraint(
            link_name="end_effector",
            frame_id="base_link",
            x=x,
            y=y,
            z=z,
            tolerance=0.015
        )

        self.arm.set_goal_state(motion_plan_constraints=[position_goal])

        plan_params = PlanRequestParameters(self.robot, "ompl")
        plan_params.planning_pipeline = "ompl"
        plan_params.planner_id = "RRTConnectkConfigDefault"
        plan_params.planning_time = 5.0
        plan_params.planning_attempts = 10

        plan_result = self.arm.plan(single_plan_parameters=plan_params)

        if plan_result:
            robot_traj_msg = plan_result.trajectory.get_robot_trajectory_msg()
            joint_trajectory_msg = robot_traj_msg.joint_trajectory

            goal_handle = await self.send_trajectory_goal(joint_trajectory_msg)

            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("Trajectory goal rejected or server unavailable.")
                return False

            self.get_logger().info("Goal accepted! Executing motion...")
            
            result = await goal_handle.get_result_async()
            self.get_logger().info(f"Successfully reached target: ({x}, {y}, {z})")
            return True
        else:
            self.get_logger().error(f"Planning failed for target: ({x}, {y}, {z})")
            return False

def main(args=None):
    rclpy.init(args=args)

    node = MoveTrajectory()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()