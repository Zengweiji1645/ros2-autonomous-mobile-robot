
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 获取各个ROS2软件包的位置
    my_robot_dir = get_package_share_directory("my_robot")
    slam_dir = get_package_share_directory("slam_toolbox")
    nav2_dir = get_package_share_directory("nav2_bringup")

    # 获取配置文件路径
    slam_config = os.path.join(
        my_robot_dir, "config", "slam.yaml"
    )

    nav2_config = os.path.join(
        my_robot_dir, "config", "nav2_params.yaml"
    )

    # 1. 启动机器人仿真
    robot_node = Node(
        package="my_robot",
        executable="robot_node",
        name="robot_node",
        output="screen"
    )

    # 2. 启动SLAM定位与建图
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                slam_dir, "launch", "online_async_launch.py"
            )
        ),
        launch_arguments={
            "slam_params_file": slam_config,
            "use_sim_time": "false"
        }.items()
    )

    # 3. 启动Nav2导航
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_dir, "launch", "navigation_launch.py"
            )
        ),
        launch_arguments={
            "params_file": nav2_config,
            "use_sim_time": "false",
            "autostart": "true"
        }.items()
    )

    # 4. 启动任务管理节点
    mission_node = Node(
        package="my_robot",
        executable="mission_node",
        name="mission_node",
        output="screen",
        parameters=[{
            "waypoints_file": os.path.join(
                my_robot_dir,
                "config",
                "waypoints.yaml"
            )
        }]
    )

    # 5. 启动RViz可视化
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    return LaunchDescription([
        robot_node,
        slam_launch,
        nav2_launch,
        mission_node,
        rviz_node
    ])
