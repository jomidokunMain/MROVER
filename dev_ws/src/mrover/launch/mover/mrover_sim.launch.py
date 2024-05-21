import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
# At the top with our imports
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.substitutions import FindPackageShare





def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pth_mavros_launcher = get_package_share_directory("mrover")
    # pth_param0 = pth_mavros_launcher + "/config/params.yaml"
    pth_param1 = pth_mavros_launcher + "/config/px4_config.yaml"
    # pth_param2 = pth_mavros_launcher + "/config/px4_pluginlists.yaml"  
    log_level = LaunchConfiguration("log_level")



    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='mrover' 
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp_mrover3.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package_name), "description/foward2Steer", "robot_core.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    diffbot_diff_drive_controller = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "my_controllers3.yaml",
        ]
    )
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )


    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[{'robot_controllers': robot_description},
    #                 controller_params_file]
    # )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, diffbot_diff_drive_controller]
        # output={
        #     "stdout": "screen",
        #     "stderr": "screen",
        # },
    )
    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["bicycle_steering_controller"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    

    # Run the camera Node
    camera_node_setup = Node(package='v4l2_camera',executable='v4l2_camera_node',
                             output='screen', parameters=[{'video_device': "/dev/video2",'image_size': [640,480],
                                                           'camera_frame_id': 'camera_link_optical'}])
   
   
    
    rviz_show = Node(package='rviz2',executable='rviz2',name='rviz2',arguments=['-d', 'dev_ws/src/mrover/config/movement_mrover.rviz'],output='screen')

    delayed_rviz_show= RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=camera_node_setup,
            on_start=[rviz_show],
        )
    )
    
    # Run the mavros node
    mavros_node = Node(package='mavros',executable='mavros_node', output="screen",
                       arguments=['--ros-args','-p', 'fcu_url:=/dev/ttyACM0:57600',
                                  '-p', 'gcs_url:=udp://@localhost:14550', '--params-file','dev_ws/src/mrover/config/px4_pluginlists.yaml', '--params-file', 'dev_ws/src/mrover/config/px4_config_ros2.yaml'],)
    




    # Launch them all!
    return LaunchDescription([
        rsp,
        node_robot_state_publisher,

        # delayed_controller_manager,
        # delayed_diff_drive_spawner,
        # delayed_joint_broad_spawner,
        camera_node_setup,
        mavros_node,

       

        delayed_rviz_show,
        
    ])



