import os
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.logging import get_logger

def load_yaml(pkg: str, relpath: str):
    path = os.path.join(get_package_share_directory(pkg), relpath)
    with open(path, "r") as file:
        return yaml.safe_load(file)

def generate_launch_description():
    declare_sigverse_port    = DeclareLaunchArgument("sigverse_ros_bridge_port", default_value="50001")
    declare_rosbridge_port   = DeclareLaunchArgument("ros_bridge_port", default_value="9090")
    declare_camera_ns        = DeclareLaunchArgument("camera_ns", default_value="/camera")
    declare_controllers_file = DeclareLaunchArgument(
        "controllers_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("sigverse_turtlebot3"),
            "config", "turtlebot3_controller_manager.yaml"
        ])
    )

    description_dir = get_package_share_directory("sigverse_turtlebot3_manipulation_description")
    urdf_path = str(Path(description_dir, "urdf",  "turtlebot3_manipulation.urdf.xacro"))

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
#        name="controller_manager",
        output="screen",
#        prefix='gnome-terminal --title="controller_manager" --',
        parameters=[
            {"robot_description": Command(["xacro ", urdf_path])},
            LaunchConfiguration("controllers_file")
        ]
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
#        name="spawn_arm_controller",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    moveit_config = (
        MoveItConfigsBuilder("sigverse_turtlebot3_manipulation")
        .robot_description           (file_path=urdf_path)
        .robot_description_semantic  (file_path="config/turtlebot3_manipulation.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits                (file_path="config/joint_limits.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .trajectory_execution        (file_path="config/moveit_controllers.yaml")
        .planning_pipelines          (pipelines=["ompl"], default_planning_pipeline="ompl")
        .sensors_3d                  (file_path="config/sensors_3d.yaml")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
#        name="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        remappings=[("/joint_states", "/tb3/joint_state")],
    )

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('yolo_bringup'), 'launch', 'yolov11.launch.py'])
        ]),
        launch_arguments={
            "namespace": "yolo_objects",
            "model": PathJoinSubstitution([EnvironmentVariable("HOME"), "yolo_weights", "yolo11n.pt"]),
            "device": "cpu",
            "threshold": "0.5",
            "imgsz_height": "376",
            "imgsz_width": "672",
            "input_image_topic": PathJoinSubstitution([LaunchConfiguration("camera_ns"), "rgb", "image_raw"]),
            "input_depth_topic": PathJoinSubstitution([LaunchConfiguration("camera_ns"), "depth", "image_raw"]),
            "input_depth_info_topic": PathJoinSubstitution([LaunchConfiguration("camera_ns"), "depth", "camera_info"]),
            "target_frame": "base_link",
            "depth_image_units_divisor": "1",
            "use_3d": "True",
            "use_debug": "True",
            "maximum_detection_threshold": "0.03"
        }.items()
    )

    moveit_config_pkg = "sigverse_turtlebot3_manipulation_moveit_config"
    srdf_content    = (Path(get_package_share_directory(moveit_config_pkg)).joinpath("config/turtlebot3_manipulation.srdf")).read_text()
    kinematics_dict = load_yaml(moveit_config_pkg, "config/kinematics.yaml")

#    logger = get_logger("dump")
#    logger.info("kinematics_dict:\n" + yaml.safe_dump(kinematics_dict, sort_keys=False))

    grasping_auto_node = Node(
        package="sigverse_turtlebot3",
        executable="grasping_auto",
        output="screen",
        prefix='gnome-terminal --title="TurtleBot3 teleop key" --geometry=60x20 --',
        parameters=[
            {"robot_description": Command(["xacro ", urdf_path])},
            {"robot_description_semantic": srdf_content},
            {"robot_description_kinematics": kinematics_dict},
        ]
    )

    sigverse_bridge_node = Node(
        package="sigverse_ros_bridge",
        executable="sigverse_ros_bridge",
        output="screen",
#        prefix='gnome-terminal --title="SIGVerse Rosbridge" --',
        arguments=[LaunchConfiguration("sigverse_ros_bridge_port")]
    )

    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'])
        ]),
        launch_arguments={
            "port": LaunchConfiguration("ros_bridge_port"),
            "default_call_service_timeout": "5.0",
            "call_services_in_new_thread": "true",
            "send_action_goals_in_new_thread": "true"
        }.items()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
#        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("sigverse_turtlebot3"), "launch", "grasping_auto_launch.rviz"
        ])]
    )

    return LaunchDescription([
        declare_sigverse_port,
        declare_rosbridge_port,
        declare_camera_ns,
        declare_controllers_file,

        controller_manager_node,
        spawn_arm_controller,
        move_group_node,
        yolo_launch,
        grasping_auto_node,
        sigverse_bridge_node,
        rosbridge_launch,
        rviz_node
    ])
