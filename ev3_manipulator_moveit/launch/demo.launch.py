# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder

# def generate_launch_description():

#     moveit_config = (
#         MoveItConfigsBuilder(
#             "manipulator_ev3_brick",
#             package_name="ev3_manipulator_moveit"
#         )
#         .robot_description(
#             file_path=os.path.join(
#                 get_package_share_directory("manipulator_ev3_brick"),
#                 "urdf", "manipulator.urdf.xacro"
#             )
#         )
#         .robot_description_semantic(
#             file_path="config/manipulator_ev3_brick.srdf"
#         )
#         .robot_description_kinematics(
#             file_path="config/kinematics.yaml"
#         )
#         .trajectory_execution(
#             file_path="config/moveit_controllers.yaml"
#         )
#         .joint_limits(
#             file_path="config/joint_limits.yaml"
#         )
#         .to_moveit_configs()
#     )

#     # move_group node
#     move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[
#             moveit_config.to_dict(),
#             {"use_sim_time": True},
#         ],
#     )

#     # RViz with MoveIt
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         output="log",
#         arguments=[
#             "-d",
#             os.path.join(
#                 get_package_share_directory(
#                     "ev3_manipulator_moveit"
#                 ),
#                 "config", "moveit.rviz"
#             )
#         ],
#         parameters=[
#             moveit_config.to_dict(),
#             {"use_sim_time": True},
#         ],
#     )

#     return LaunchDescription([
#         move_group_node,
#         rviz_node,
#     ])


# import os
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import TimerAction
# from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder

# def generate_launch_description():

#     pkg_moveit = get_package_share_directory(
#         "ev3_manipulator_moveit")
#     pkg_robot = get_package_share_directory(
#         "manipulator_ev3_brick")

#     # Read SRDF file
#     srdf_file = os.path.join(
#         pkg_moveit, "config", "manipulator_ev3_brick.srdf")
#     with open(srdf_file, 'r') as f:
#         srdf_content = f.read()

#     # Read URDF via xacro
#     import xacro
#     urdf_file = os.path.join(
#         pkg_robot, "urdf", "manipulator.urdf.xacro")
#     robot_description_content = xacro.process_file(
#         urdf_file).toxml()

#     moveit_config = (
#         MoveItConfigsBuilder(
#             "manipulator_ev3_brick",
#             package_name="ev3_manipulator_moveit"
#         )
#         .robot_description(
#             file_path=urdf_file
#         )
#         .robot_description_semantic(
#             file_path="config/manipulator_ev3_brick.srdf"
#         )
#         .robot_description_kinematics(
#             file_path="config/kinematics.yaml"
#         )
#         .trajectory_execution(
#             file_path="config/moveit_controllers.yaml"
#         )
#         .joint_limits(
#             file_path="config/joint_limits.yaml"
#         )
#         .to_moveit_configs()
#     )

#     moveit_dict = moveit_config.to_dict()

#     # move_group node
#     move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[
#             moveit_dict,
#             {
#                 "use_sim_time": True,
#                 "robot_description_semantic": srdf_content,
#                 "robot_description": robot_description_content,
#                 "publish_robot_description_semantic": True,
#             },
#         ],
#     )

#     # RViz node
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=[
#             "-d",
#             os.path.join(pkg_moveit, "config", "moveit.rviz")
#         ],
#         parameters=[
#             moveit_dict,
#             {
#                 "use_sim_time": True,
#                 "robot_description_semantic": srdf_content,
#                 "robot_description": robot_description_content,
#             },
#         ],
#     )

#     return LaunchDescription([
#         move_group_node,
#         TimerAction(period=3.0, actions=[rviz_node]),
#     ])


import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    pkg_robot = get_package_share_directory("manipulator_ev3_brick")
    pkg_moveit = get_package_share_directory("ev3_manipulator_moveit")

    urdf_file = os.path.join(
        pkg_robot,
        "urdf",
        "manipulator.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "manipulator_ev3_brick",
            package_name="ev3_manipulator_moveit"
        )
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(
            file_path="config/manipulator_ev3_brick.srdf"
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .joint_limits(
            file_path="config/joint_limits.yaml"
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
                "moveit_manage_controllers": False,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(pkg_moveit, "config", "moveit.rviz")
        ],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        move_group_node,
        TimerAction(period=3.0, actions=[rviz_node]),
    ])