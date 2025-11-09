from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    在RViz中查看带夹具的UR16e机器人
    
    使用方法:
    ros2 launch ur16e_with_gripper_description view_robot.launch.py
    """
    
    # 声明参数
    declare_use_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='启动joint_state_publisher_gui来手动控制关节'
    )
    
    # 获取URDF
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('ur16e_with_gripper_description'),
            'urdf',
            'ur16e_with_suction_gripper.xacro'
        ]),
        ' name:=ur16e',
        ' ur_type:=ur16e',
        ' prefix:=""',
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    # Joint State Publisher GUI (手动控制关节)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    
    # Joint State Publisher (不带GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ur16e_with_gripper_description'),
        'config',
        'view_robot.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )
    
    return LaunchDescription([
        declare_use_gui,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz_node,
    ])
