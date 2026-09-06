"""Single entry point for the two-robot box pick demo.

Brings up one Gazebo (Ignition Fortress) world containing both cells, spawns
both UR5e arms into it (modeled on ur_simulation_gz's ur_sim_control.launch.py,
but with two robots sharing one Gazebo instance instead of one robot with its
own), bridges camera/pose/gripper topics, and starts the belief-region oracle
plus both missions' search_and_pick_node.

Robot commands go out as plain topics: each robot's joint_trajectory_controller
is reached at /<tf_prefix>joint_trajectory_controller/joint_trajectory.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# This lab's network carries other people's real ROS2 systems on the default
# domain (ROS_DOMAIN_ID=0) - DDS discovery is multicast and domain-scoped, so
# sharing it means this demo's nodes/topics show up in their graph and
# vice versa (found by actually launching this and seeing unrelated nodes
# like /experiment_setup show up in `ros2 node list`). Fixed here rather than
# left to whoever launches this to remember to export it.
ROS_DOMAIN_ID = "33"

PKG = "box_pick_place_demo"

# Robots are mounted on top of their table (z=0.75, matching the table
# height in worlds/box_pick_demo.sdf), not standing on the ground beside it.
TABLE_HEIGHT_M = 0.75

# (name, tf_prefix, spawn_x, spawn_y, child_model, controllers_file, no-help/help margin)
ROBOTS = [
    dict(name="robot_a", tf_prefix="robot_a_", x=0.0, y=0.0, z=TABLE_HEIGHT_M, child_model="box_a",
         controllers_file="controllers_robot_a.yaml"),
    dict(name="robot_b", tf_prefix="robot_b_", x=2.5, y=0.0, z=TABLE_HEIGHT_M, child_model="box_b",
         controllers_file="controllers_robot_b.yaml"),
]


def robot_actions(robot: dict, help_flag: bool, launch_missions) -> list:
    name = robot["name"]
    tf_prefix = robot["tf_prefix"]

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare(PKG), "config", robot["controllers_file"]]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(PKG), "urdf", "box_pick_robot.urdf.xacro"]),
            " name:=", name,
            " ur_type:=ur5e",
            " tf_prefix:=", tf_prefix,
            " simulation_controllers:=", controllers_yaml,
            " child_model:=", robot["child_model"],
            " attach_topic:=/", name, "/gripper/attach",
            " detach_topic:=/", name, "/gripper/detach",
            " output_topic:=/", name, "/gripper/state",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=f"{tf_prefix}robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content,
            "-name", name,
            "-x", str(robot["x"]), "-y", str(robot["y"]), "-z", str(robot["z"]),
            "-allow_renaming", "true",
        ],
    )

    # --param-file passed explicitly to each spawner (redundant with the
    # <parameters> tag on the ros2_control plugin itself) because without it,
    # this Fortress-era ign_ros2_control build was observed cross-contaminating
    # the two robots' controller parameters: whichever robot's plugin loaded
    # its YAML second would leave the other's joint_trajectory_controller
    # configured with empty joints/command_interfaces (found by actually
    # launching both robots and comparing `ros2 control list_controllers`
    # output - robot_b came up active, robot_a's jtc stayed "unconfigured").
    # Passing it directly to the spawner sidesteps whatever shared/global
    # parameter state the two plugin instances were racing on.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[f"{tf_prefix}joint_state_broadcaster", "-c", f"{tf_prefix}controller_manager",
                   "--param-file", controllers_yaml],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[f"{tf_prefix}joint_trajectory_controller", "-c", f"{tf_prefix}controller_manager",
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
        executable="search_and_pick_node",
        name=f"{tf_prefix}search_and_pick_node",
        parameters=[{
            "use_sim_time": True,
            "robot_name": name,
            "tf_prefix": tf_prefix,
            "use_help": help_flag,
        }],
        condition=IfCondition(launch_missions),
    )
    # Mission start (and its detach() calls) is gated on THIS robot's own
    # joint_trajectory_controller_spawner having exited, not launched
    # alongside everything else and hoping enough time has passed: a
    # timing-based guess (e.g. "wait until region+pose topics arrive") was
    # tried first and looked reliable in isolated tests, but broke again as
    # soon as this robot's own spawn+controller sequence (which the region
    # wait isn't actually coupled to) happened to run slower - the box was
    # found still dragging on the robot's very first motion because its
    # own DetachableJoint plugin/controller genuinely wasn't up yet when
    # detach() was sent. Chaining off OnProcessExit of this robot's own
    # spawner (same pattern as delay_jtc_after_jsb above) is the actual
    # guarantee: by the time the jtc spawner reports "Configured and
    # activated" and exits, this robot's model and all its plugins are
    # necessarily already loaded.
    delay_mission_after_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[mission_node],
        )
    )

    return [
        robot_state_publisher_node,
        spawn_entity,
        joint_state_broadcaster_spawner,
        delay_jtc_after_jsb,
        delay_mission_after_jtc,
    ]


def generate_launch_description():
    set_ros_domain_id = SetEnvironmentVariable("ROS_DOMAIN_ID", ROS_DOMAIN_ID)

    declared_arguments = [
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Start gazebo with GUI?"),
        DeclareLaunchArgument(
            "world_file",
            default_value=PathJoinSubstitution([FindPackageShare(PKG), "worlds", "box_pick_demo.sdf"]),
            description="Gazebo world file (absolute path).",
        ),
        DeclareLaunchArgument("launch_missions", default_value="true",
                               description="Start belief_region_node + both search_and_pick_node instances."),
        # A single random guess anywhere in the region is compared against
        # GRASP_TOLERANCE_M (0.04m, in search_and_pick_node.py) to decide
        # success. no_help_margin=0.40 makes a lucky guess unlikely (~1%);
        # help_margin=0.03 is at/below that tolerance, so every possible
        # guess in robot_b's region is automatically within tolerance -
        # help always works, no help almost never does.
        DeclareLaunchArgument("no_help_margin", default_value="0.40",
                               description="Belief-region half-extent (m) for robot_a (no help)."),
        DeclareLaunchArgument("help_margin", default_value="0.03",
                               description="Belief-region half-extent (m) for robot_b (help)."),
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
        # Per-model PosePublisher plugins were tried first (see worlds/box_pick_demo.sdf
        # history) but never actually published for these at-rest dynamic
        # bodies, regardless of <static_publisher>/<update_frequency> tags -
        # found by echoing the gz-native topic directly and seeing nothing.
        # SceneBroadcaster's always-on world pose stream works reliably.
        "/world/box_pick_demo/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
    ]
    for robot in ROBOTS:
        n = robot["name"]
        bridge_topics += [
            f"/{n}/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            f"/{n}/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            f"/{n}/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            f"/{n}/gripper/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
            f"/{n}/gripper/detach@std_msgs/msg/Empty]ignition.msgs.Empty",
            # GZ->ROS: DetachableJoint's own state feedback ("attached"/
            # "detached" strings, confirmed via `ign topic -e`), used by
            # grasp_controller.py to confirm attach()/detach() actually
            # took effect instead of firing blind.
            f"/{n}/gripper/state@std_msgs/msg/String[ignition.msgs.StringMsg",
        ]
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_topics,
        output="screen",
    )

    belief_region_node = Node(
        package=PKG,
        executable="belief_region_node",
        name="belief_region_node",
        parameters=[{
            "use_sim_time": True,
            "no_help_margin": LaunchConfiguration("no_help_margin"),
            "help_margin": LaunchConfiguration("help_margin"),
        }],
        condition=IfCondition(LaunchConfiguration("launch_missions")),
    )

    nodes = [gz_sim_gui, gz_sim_headless, gz_bridge, belief_region_node]
    # Staggered per robot (4s apart), not launched concurrently: both robots'
    # controller_manager instances live in the SAME Gazebo process, and
    # letting them load joint_trajectory_controller's plugin library at
    # nearly the same instant was observed hitting a pluginlib/class_loader
    # race - one succeeds, the other fails with "no factory exists for it"
    # and its controller never loads, silently turning that robot's motion
    # commands into no-ops (found by comparing a "successful" mission's
    # logged guess against the arm's actual controller state afterward:
    # only joint_state_broadcaster was active, joint_trajectory_controller
    # was missing entirely). A few seconds' stagger avoids the collision.
    # Each robot's mission node itself is launched later still, chained off
    # that robot's own controller spawner finishing (see robot_actions) -
    # not by this stagger delay directly.
    for i, (robot, help_flag) in enumerate(zip(ROBOTS, [False, True])):
        nodes.append(TimerAction(
            period=float(i) * 4.0,
            actions=robot_actions(robot, help_flag, LaunchConfiguration("launch_missions")),
        ))

    return LaunchDescription([set_ros_domain_id] + declared_arguments + nodes)
