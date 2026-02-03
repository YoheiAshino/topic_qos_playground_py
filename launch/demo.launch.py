from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    depth = LaunchConfiguration('depth', default='1')
    reliable = LaunchConfiguration('reliable', default='false')
    pub_period_ms = LaunchConfiguration('pub_period_ms', default='10')
    sub_sleep_ms = LaunchConfiguration('sub_sleep_ms', default='20')

    return LaunchDescription([
        Node(
            package='topic_qos_playground_py',
            executable='qos_talker_py',
            name='qos_talker_py',
            output='screen',
            parameters=[{
                'depth': depth,
                'reliable': reliable,
                'pub_period_ms': pub_period_ms,
            }],
        ),
        Node(
            package='topic_qos_playground_py',
            executable='qos_listener_py',
            name='qos_listener_py',
            output='screen',
            parameters=[{
                'depth': depth,
                'reliable': reliable,
                'sub_sleep_ms': sub_sleep_ms,
            }],
        ),
    ])

