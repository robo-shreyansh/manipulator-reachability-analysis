from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur", package_name="ur_10_desc_moveit").to_moveit_configs()
    robo_node = Node(
        package="reachability_analysis",
        executable="reach_node",
        output = "screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    ld = generate_rsp_launch(moveit_config)
    ld.add_action(robo_node)
    return LaunchDescription([robo_node])