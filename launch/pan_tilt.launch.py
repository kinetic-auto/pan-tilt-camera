from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock = LaunchConfiguration("use_mock_hardware")
    port_name = LaunchConfiguration("port_name")
    baud_rate = LaunchConfiguration("baud_rate")
    rviz = LaunchConfiguration("rviz")
    namespace = LaunchConfiguration("namespace")
    controller_manager = ["/", namespace, "/controller_manager"]

    pkg = FindPackageShare("pan_tilt_camera")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([pkg, "urdf", "pan_tilt.urdf.xacro"]),
            " use_mock_hardware:=", use_mock,
            " port_name:=", port_name,
            " baud_rate:=", baud_rate,
        ]),
        value_type=str,
    )
    controllers = PathJoinSubstitution([pkg, "config", "controllers.yaml"])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,  # also namespaces the dynamixel_hardware_interface node in this process
        parameters=[{"robot_description": robot_description}, controllers],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", "--controller-manager", controller_manager],
    )

    # Start the trajectory controller only after joint_state_broadcaster is up
    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["pan_tilt_controller", "--controller-manager", controller_manager],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([pkg, "rviz", "pan_tilt.rviz"])],
        condition=IfCondition(rviz),
        output="log",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="true = simulated joints, no servo needed"),
        DeclareLaunchArgument("port_name", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud_rate", default_value="57600"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("namespace", default_value="pan_tilt_camera",
                              description="ROS namespace for all topics/services/actions (rviz/pan_tilt.rviz assumes the default)"),
        control_node,
        robot_state_publisher,
        jsb_spawner,
        RegisterEventHandler(OnProcessExit(target_action=jsb_spawner, on_exit=[jtc_spawner])),
        rviz_node,
    ])
