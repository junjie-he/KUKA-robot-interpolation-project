from ament_index_python.packages       import get_package_share_path

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions                import Node
from launch.substitutions              import Command, LaunchConfiguration
from launch.actions                    import DeclareLaunchArgument, ExecuteProcess
from launch                            import LaunchDescription

import os


# The following function is mandatory
def generate_launch_description():

    # Defining the object, which must be returned
    ld = LaunchDescription()

    package_path = get_package_share_path('waam_interpolation')

    urdf_model_path  = os.path.join(package_path, 'model/kuka_robot.urdf')

    rviz_config_path = os.path.join(package_path, 'rviz/rvizconfig.rviz')

    #sim_time = LaunchConfiguration('use_sim_time', default = 'false')

    model_arg = DeclareLaunchArgument(name          = 'model',
                                      default_value = str(urdf_model_path),
                                      description   = "This is my URDF model definition")

    rviz_arg = DeclareLaunchArgument(name          = 'rvizconfig',
                                     default_value = str(rviz_config_path),
                                     description   = "This is my RViz config file")

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type = str)

    # Configure the main node
    main_node = Node(package    = "waam_interpolation",
                     executable = "digital_twin",
                     name='kuka_robot',
                     output     = "screen",
                     namespace  = None)
                     

 

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisheer",
        parameters=[{'robot_description': robot_description}]
    )
    
#    joint_state_publisher_node = Node(
#        package='joint_state_publisher',
#        executable='joint_state_publisher',
#        name = "joint_state_publisher",
#        output = "screen"
#    )

    visualization_node = Node(
        package='waam_interpolation',
        executable='visualization_marker',
        name = "tcp_marker",
        output = "screen")
        
    real_path_node = Node(
        package='waam_interpolation',
        executable='original_path',
        name = "originalpath",
        output = "screen")
        
    interpolated_path_node = Node(
        package='waam_interpolation',
        executable='interpolated_path',
        name = "interpolatedpath",
        output = "screen")
    

    # Configure the node for the joystick
    joy_node = Node(package    = "joy",
                    executable = "joy_node",
                    output     = "screen",
                    name       = "joy_node",
                    namespace  = None)

    # Configure RViz for visualization
    rviz_node = Node(package    = "rviz2",
                     executable = "rviz2",
                     output     = "screen",
                     name='rviz2',
                     namespace  = None,
                     arguments  = ["-d", LaunchConfiguration("rvizconfig")])


    ld.add_action(main_node)
    ld.add_action(rviz_arg)
    ld.add_action(model_arg)
    ld.add_action(joy_node)
    ld.add_action(rviz_node)
    ld.add_action(visualization_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(real_path_node)
    ld.add_action(interpolated_path_node)
  
    return ld
