"""Entry point for scene 1 (pick a ball, place it in a basket) running on
REAL hardware: a real UR5e (via ur_robot_driver) and a real OnRobot 2FG7
gripper (via onrobot_2fg7's grip/status nodes) - no Gazebo, no belief-
region guessing, no vision. See real_ball_sort_node.py's docstring for
what that means and what's still a placeholder needing your own
measurement/tuning.

Written directly against ur_robot_driver 2.13.2 and onrobot_2fg7's actual
source (both inspected on this machine/from their repos, not guessed).

Actually tested here (no real UR5e/2FG7 available, but this much didn't
need one): launched this file with use_fake_hardware:=true and
robot_ip:=0.0.0.0, and gripper_ip left at its unreachable default -
`joint_trajectory_controller` loaded and activated, and the mocked joint
states exactly tracked every commanded position (move_home() and three
IK-computed waypoints, verified by echoing /joint_states against the
logged commands), confirming the controller/topic wiring end to end. With
no gripper reachable, attach()/detach() correctly timed out and returned
False rather than a false success (see real_ball_sort_node.py's docstring
for exactly what was confirmed there), and the mission correctly aborted
before lifting rather than proceeding blind.

NOT tested, at all: anything with a real arm or a real gripper actually
connected. Dry-run with use_fake_hardware:=true again after any change
here, and watch the logs, before ever pointing this at a real arm near
people or fragile things.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Standard domain for this lab's real UR5e work (matches ~/.bashrc's
# default) - baked in here too rather than left to scripts/run_scene1_real.sh
# to set externally, so it's guaranteed regardless of how this is invoked.
ROS_DOMAIN_ID = "133"

PKG = "box_pick_place_demo"

# OnRobot's fixed XML-RPC port on the UR controller - see onrobot_2fg7's
# README ("Universal Robots default port"), not expected to ever change,
# so it's a plain constant rather than a launch argument (avoids a launch-
# argument-string vs. declared-integer-parameter type mismatch).
GRIPPER_PORT = 41414


def generate_launch_description():
    set_ros_domain_id = SetEnvironmentVariable("ROS_DOMAIN_ID", ROS_DOMAIN_ID)

    declared_arguments = [
        DeclareLaunchArgument("robot_ip", description="IP address of the real UR5e."),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "config", "ur5e", "default_kinematics.yaml"]
            ),
            description="This robot's OWN calibration, extracted via ur_calibration, for real "
                        "accuracy - the default here is the generic, uncalibrated UR5e kinematics "
                        "(also what ik_solver.py's DH parameters are derived from).",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="false",
            description="Mock the arm in software, no real robot needed - dry-run with this "
                        "true first, always.",
        ),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument(
            # Confirmed reachable at this lab's robot_ip (10.240.10.2, same
            # host as the UR controller - port 41414 open there, per an
            # earlier connectivity check) - onrobot_2fg7's own example
            # default (192.168.0.9) caused a real failure here: the grip
            # service call just hangs (no socket timeout in that package's
            # own code) until this project's 15s client-side timeout gives
            # up, which looks identical to a genuinely broken gripper.
            "gripper_ip", default_value="10.240.10.2",
            description="IP of the OnRobot compute box (mounted on the UR arm itself).",
        ),
        DeclareLaunchArgument(
            "launch_gripper", default_value="true",
            description="Start onrobot_2fg7's status/grip nodes.",
        ),
        DeclareLaunchArgument(
            "launch_mission", default_value="true",
            description="Start real_ball_sort_node - safe to leave true even before pressing Play "
                        "on the pendant, it waits for joint_trajectory_controller to actually be "
                        "active before doing anything. Set false to bring up just the arm/gripper "
                        "drivers instead, e.g. for scripts/test_arm_move.py.",
        ),
    ]

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "kinematics_params_file": LaunchConfiguration("kinematics_params_file"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            # ur_robot_driver's own default is scaled_joint_trajectory_
            # controller instead - arm_commander.py's hardcoded topic
            # pattern (/<tf_prefix>joint_trajectory_controller/
            # joint_trajectory) needs this exact one.
            "initial_joint_controller": "joint_trajectory_controller",
        }.items(),
    )

    # onrobot_2fg7's own launch.xml expects two YAML files under its
    # share/config/ that the cloned repo doesn't actually include (no
    # config/ directory exists in it at all) - launch the two nodes
    # directly with inline parameters instead of including that file.
    gripper_status_node = Node(
        package="onrobot_2fg7",
        executable="status",
        name="onrobot_2fg7_status_publisher",
        parameters=[{
            "ip": LaunchConfiguration("gripper_ip"),
            "port": GRIPPER_PORT,
            "frequency": 5.0,
            "queue_msgs": 10,
        }],
        condition=IfCondition(LaunchConfiguration("launch_gripper")),
    )
    gripper_grip_node = Node(
        package="onrobot_2fg7",
        executable="grip",
        name="onrobot_2fg7_grip_service",
        parameters=[{
            "ip": LaunchConfiguration("gripper_ip"),
            "port": GRIPPER_PORT,
        }],
        condition=IfCondition(LaunchConfiguration("launch_gripper")),
    )

    mission_node = Node(
        package=PKG,
        executable="real_ball_sort_node",
        name="real_ball_sort_node",
        parameters=[{"tf_prefix": ""}],
        condition=IfCondition(LaunchConfiguration("launch_mission")),
    )

    return LaunchDescription([set_ros_domain_id] + declared_arguments + [
        ur_control, gripper_status_node, gripper_grip_node, mission_node,
    ])
