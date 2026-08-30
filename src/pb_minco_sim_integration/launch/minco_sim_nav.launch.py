import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import SetRemap


def generate_launch_description():
    integration_share = get_package_share_directory("pb_minco_sim_integration")
    bringup_share = get_package_share_directory("pb2025_nav_bringup")

    namespace = LaunchConfiguration("namespace")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_composition = LaunchConfiguration("use_composition")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    slam = LaunchConfiguration("slam")

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="red_standard_robot1",
        description="Single simulated robot namespace.")
    declare_world = DeclareLaunchArgument(
        "world", default_value="rmuc_2025",
        description="PBSimulation world name.")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="True",
        description="Use the Gazebo clock.")
    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            integration_share, "config", "minco_sim_nav2_params.yaml"),
        description="Mode A Nav2/ROGMap/MINCO parameter file.")
    declare_use_composition = DeclareLaunchArgument(
        "use_composition", default_value="False",
        description="Use composition after the standalone path is verified.")
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="True",
        description="Start RViz with the MINCO/ROGMap-aware view.")
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(
            integration_share, "rviz", "nav2_default_view_minco.rviz"),
        description="RViz config with MINCO/ROGMap/MPC displays (baseline + extras).")
    declare_slam = DeclareLaunchArgument(
        "slam", default_value="False",
        description="Mode A uses the existing localization/relocalization path.")

    # The existing bringup owns Gazebo-facing sensors, Point-LIO, small_gicp,
    # costmaps, behaviors, and the velocity conversion chain.  This wrapper
    # only supplies the new parameter file and the absolute odom remap needed
    # by the vendored controller plugin.
    current_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "rm_navigation_simulation_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "world": world,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "use_composition": use_composition,
            "use_rviz": use_rviz,
            "rviz_config_file": rviz_config_file,
            "slam": slam,
        }.items(),
    )

    # MincoMpcController subscribes to /aft_mapped_to_init as an absolute
    # name, while PBSimulation publishes it below the robot namespace.
    remapped_bringup = GroupAction(
        actions=[
            SetRemap(
                src="/aft_mapped_to_init",
                dst=[TextSubstitution(text="/"), namespace,
                     TextSubstitution(text="/aft_mapped_to_init")],
            ),
            current_bringup,
        ]
    )

    return LaunchDescription([
        declare_namespace,
        declare_world,
        declare_use_sim_time,
        declare_params,
        declare_use_composition,
        declare_use_rviz,
        declare_rviz_config,
        declare_slam,
        remapped_bringup,
    ])
