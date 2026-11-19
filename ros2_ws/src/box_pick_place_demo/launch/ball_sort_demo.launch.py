"""Entry point for the single-robot ball-sort demo.

Brings up one Gazebo (Ignition Fortress) world with one UR5e arm, a red/green
ball pair and a red/green basket pair, bridges pose/gripper topics, and
starts the ball-region oracle plus the pick-and-place mission.

Modeled directly on demo.launch.py's per-robot pattern (xacro wiring,
ign_ros2_control quirks, DetachableJoint wiring, the wait-for-controller
race, and gating the mission node's start on this robot's own controller
spawner having exited) but simplified to one robot - no cross-robot
pluginlib stagger is needed since there's only one controller_manager in
this Gazebo process.
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

# Robot is mounted on top of its table (z=0.75, matching the table height
# in worlds/ball_sort_demo.sdf), not standing on the ground beside it.
TABLE_HEIGHT_M = 0.75


def generate_launch_description():
    set_ros_domain_id = SetEnvironmentVariable("ROS_DOMAIN_ID", ROS_DOMAIN_ID)

    declared_arguments = [
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start gazebo with GUI?"),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution([FindPackageShare(PKG), "worlds", "ball_sort_demo.sdf"]),
            description="Gazebo world file (absolute path).",
        ),
        DeclareLaunchArgument("launch_mission", default_value="true",
                               description="Start ball_region_node + ball_sort_node."),
        # Narrow margin, matching robot_b's "help" case in the box-pick
        # demo - this pick is meant to reliably succeed, the interesting
        # part of this demo is the color-based place, not a search contrast.
        DeclareLaunchArgument("margin", default_value="0.03",
                               description="Belief-region half-extent (m) for the green ball."),
        # Must match green_basket's actual <pose> x/y in whatever world_file
        # is loaded - ball_sort_node.py has no live pose feed for the
        # basket (unlike the ball, it never moves, so there's nothing to
        # track), so a caller swapping in a world with the basket
        # relocated (e.g. a randomized-layout sweep) needs to pass the
        # matching position here too. Defaults match this package's own
        # worlds/ball_sort_demo.sdf.
        DeclareLaunchArgument("basket_x", default_value="0.65", description="green_basket's world x (m)."),
        DeclareLaunchArgument("basket_y", default_value="0.08", description="green_basket's world y (m)."),
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
        "/world/ball_sort_demo/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
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
            "basket_x": ParameterValue(LaunchConfiguration("basket_x"), value_type=float),
            "basket_y": ParameterValue(LaunchConfiguration("basket_y"), value_type=float),
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
