import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    docs_path = os.path.join(get_package_share_directory('aws_ros2_bridge'), 'param', 'docs')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='core',
        description='Namespace for the node'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use Sim Time'
    )

    # TODO : will need to have better way to include cert files.
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    root_ca_path = LaunchConfiguration('root_ca_path', default=os.path.join(docs_path, 'root-CA.crt'))
    private_key_path = LaunchConfiguration('private_key_path', default=os.path.join(docs_path, 'c-apne2-rs01-thg-00.private.key'))
    certificate_path = LaunchConfiguration('certificate_path', default=os.path.join(docs_path, 'c-apne2-rs01-thg-00.cert.pem'))
    aws_iot_endpoint = LaunchConfiguration('aws_iot_endpoint', default='a3fo5xuaf6oyj6-ats.iot.ap-northeast-2.amazonaws.com')
    aws_iot_port = LaunchConfiguration('aws_iot_port', default='8883')

    core_node = Node(
        package='mr_core',
        executable='core',
        name='core',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'root_ca_path': root_ca_path,
            'private_key_path': private_key_path,
            'cert_path': certificate_path,
            'iot_endpoint': aws_iot_endpoint,
            'port': aws_iot_port,
        }],
    )

    launch_description = LaunchDescription([
        use_sim_time_arg,
        namespace_arg,
        core_node,
    ])

    return launch_description
