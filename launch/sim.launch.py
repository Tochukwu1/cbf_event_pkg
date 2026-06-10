from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cbf_box_path = os.path.join(get_package_share_directory('cbf_event_pkg'), 'models', 'cbf_box', 'model.sdf')
    return LaunchDescription([

        # =========================================================
        # 2. ROS 2 EXPERIMENT NODE (Unicycle Robot Control)
        # =========================================================
        Node(
            package='cbf_event_pkg',
            executable='cbf_event_node',
            name='unicycle_cbf_node',
            output='screen',
            parameters=[{
                'k_p': 0.8,               
                'epsilon': 0.05,          
                'lookahead_dist': 0.15,   
                'obs_radius': 0.35,       
                'goal_x': 5.0,            
                'max_v': 0.22,
                'obs_x': 2.5, 
                'obs_y': -0.2 
            }]
        ),
        
        # =========================================================
        # 3. OBSTACLE SPAWNER
        # =========================================================
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_obstacle',
            arguments=[
                '-entity', 'cbf_obstacle',
                '-file', cbf_box_path,
                '-x', '2.5',
                '-y', '-0.2',  
                '-z', '0.5'
            ],
            output='screen'
        ),
    ])