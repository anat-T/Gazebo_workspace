"""Entry point for scene 2: the single-robot ball-sort demo with one plain
gray basket instead of scene 1's red/green pair.

Nearly identical to ball_sort_demo.launch.py (scene 1) - same xacro wiring,
same controllers_ball_sort.yaml, same controller-spawner-gated mission
start - just pointed at ball_sort_demo2.sdf and passing this scene's
world_name/basket_x/basket_y to ball_region_node/ball_sort_node (both now
parametrized rather than hardcoded, precisely so a second scene like this
one doesn't need its own copy of that Python).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Same network-isolation reasoning as demo.launch.py - this lab's network
# carries other people's real ROS2 systems on the default domain.
ROS_DOMAIN_ID = "33"

PKG = "box_pick_place_demo"
NAME = "robot"
TF_PREFIX = "robot_"
WORLD_NAME = "ball_sort_demo2"

# Robot is mounted on top of its table (z=0.75, matching the table height
# in worlds/ball_sort_demo2.sdf), not standing on the ground beside it.
TABLE_HEIGHT_M = 0.75

# gray_basket's position in ball_sort_demo2.sdf - must match its <pose>.
BASKET_X = 0.65
BASKET_Y = 0.0


def generate_launch_description():
    set_ros_domain_id = SetEnvironmentVariable("ROS_DOMAIN_ID", ROS_DOMAIN_ID)

    declared_arguments = [
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start gazebo with GUI?"),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution([FindPackageShare(PKG), "worlds", "ball_sort_demo2.sdf"]),
            description="Gazebo world file (absolute path).",
        ),
        DeclareLaunchArgument("launch_mission", default_value="true",
                               description="Start ball_region_node + ball_sort_node."),
        # Narrow margin, matching robot_b's "help" case in the box-pick
        # demo - this pick is meant to reliably succeed, the interesting
        # part of this demo is the color-based place, not a search contrast.
        DeclareLaunchArgument("margin", default_value="0.03",
                               description="Belief-region half-extent (m) for the green ball."),
    ]

    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": [" -r -v 4 ", world_file]}.items(),
        condition=IfCondition(gazebo_gui),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments={"gz_args": [" -s -r -v 4 ", world_file]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    bridge_topics = [
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        f"/world/{WORLD_NAME}/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        f"/{NAME}/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
        f"/{NAME}/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
        f"/{NAME}/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        f"/{NAME}/gripper/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
        f"/{NAME}/gripper/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
        f"/{NAME}/gripper/state@std_msgs/msg/String[ignition.msgs.StringMsg",
    ]
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_topics,
        output="screen",
    )

    ball_region_node = Node(
        package=PKG,
        executable="ball_region_node",
        name="ball_region_node",
        parameters=[{
            "use_sim_time": True,
            "margin": LaunchConfiguration("margin"),
            "world_name": WORLD_NAME,
        }],
        condition=IfCondition(LaunchConfiguration("launch_mission")),
    )

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare(PKG), "config", "controllers_ball_sort.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(PKG), "urdf", "box_pick_robot.urdf.xacro"]),
            " name:=", NAME,
            " ur_type:=ur5e",
            " tf_prefix:=", TF_PREFIX,
            " simulation_controllers:=", controllers_yaml,
            # DetachableJoint's child is hardwired to green_ball - the
            # robot's gripper can only ever pick that one, architecturally
            # (not just by mission logic choosing not to touch red_ball).
            " child_model:=green_ball",
            " attach_topic:=/", NAME, "/gripper/attach",
            " detach_topic:=/", NAME, "/gripper/detach",
            " output_topic:=/", NAME, "/gripper/state",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=f"{TF_PREFIX}robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content,
            "-name", NAME,
            "-x", "0.0", "-y", "0.0", "-z", str(TABLE_HEIGHT_M),
            "-allow_renaming", "true",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[f"{TF_PREFIX}joint_state_broadcaster", "-c", f"{TF_PREFIX}controller_manager",
                   "--param-file", controllers_yaml],
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[f"{TF_PREFIX}joint_trajectory_controller", "-c", f"{TF_PREFIX}controller_manager",
                   "--param-file", controllers_yaml],
    )
    delay_jtc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    mission_node = Node(
        package=PKG,
        executable="ball_sort_node",
        name="ball_sort_node",
        parameters=[{
            "use_sim_time": True,
            "world_name": WORLD_NAME,
            "basket_x": BASKET_X,
            "basket_y": BASKET_Y,
        }],
        condition=IfCondition(LaunchConfiguration("launch_mission")),
    )
    # Mission start (and its detach() call) is gated on the robot's own
    # joint_trajectory_controller_spawner having exited, not launched
    # alongside everything else - by the time it exits (reporting
    # "Configured and activated"), the robot's model and all its plugins
    # are necessarily already loaded. See search_and_pick_node.py /
    # demo.launch.py for why a timing-guess version of this broke.
    delay_mission_after_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[mission_node],
        )
    )

    nodes = [
        gz_sim_gui, gz_sim_headless, gz_bridge, ball_region_node,
        robot_state_publisher_node, spawn_entity,
        joint_state_broadcaster_spawner, delay_jtc_after_jsb, delay_mission_after_jtc,
    ]

    return LaunchDescription([set_ros_domain_id] + declared_arguments + nodes)
