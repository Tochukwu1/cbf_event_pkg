from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cbf_box_path = os.path.join(get_package_share_directory('cbf_event_pkg'), 'models', 'cbf_box', 'model.sdf')
    
    # --- DYNAMICALLY FIND THE SOURCE FOLDER ---
    pkg_share_path = get_package_share_directory('cbf_event_pkg')
    workspace_root = os.path.abspath(os.path.join(pkg_share_path, '..', '..', '..', '..'))
    pkg_src_path = os.path.join(workspace_root, 'src', 'cbf_event_pkg')
    
    # --- Route to the 'data' folder ---
    log_path = os.path.join(pkg_src_path, 'data', 'sim_data.csv')

    # Automatically create the 'data' folder if it is missing
    data_dir = os.path.dirname(log_path)
    if not os.path.exists(data_dir):
        os.makedirs(data_dir)

    return LaunchDescription([

        # =========================================================
        # 1. ROS 2 EXPERIMENT NODE (Holonomic Robot Control)
        # =========================================================
        Node(
            package='cbf_event_pkg', 
            executable='holonomic_cbf_node', 
            name='holonomic_cbf_node',
            output='screen',
            parameters=[{
                'log_file_path': log_path,  # <-- Injects the dynamic path here
                'robot_radius': 0.18,
                'obs_x': 2.5,  
                'obs_y': -0.2, 
                'obs_radius': 0.35,       
                'goal_x': 5.0,            
                'goal_y': 0.0,
                'goal_tolerance': 0.05,
                'epsilon': 0.05,
                'k_p': 0.8,
                'max_v': 0.22,
                'timer_period_ms': 10
            }]
        ),

        # =========================================================
        # 2. OBSTACLE SPAWNER (Ignition/New Gazebo)
        # =========================================================
        Node(
            package='ros_gz_sim', 
            executable='create',
            name='spawn_obstacle',
            arguments=[
                '-name', 'cbf_obstacle',
                '-file', cbf_box_path,
                '-x', '2.5',
                '-y', '-0.2',  
                '-z', '0.5'
            ],
            output='screen'
        ),

        # # =========================================================
        # # 3. ROS 2 EXPERIMENT NODE (Unicycle Robot Control)
        # # =========================================================
        # Node(
        #     package='cbf_event_pkg',
        #     executable='cbf_event_node',
        #     name='unicycle_cbf_node',
        #     output='screen',
        #     parameters=[{
        #         'k_p': 0.8,               
        #         'epsilon': 0.05,          
        #         'lookahead_dist': 0.15,   
        #         'obs_radius': 0.35,       
        #         'goal_x': 5.0,            
        #         'max_v': 0.22,
        #         'obs_x': 2.5, 
        #         'obs_y': -0.2 
        #     }]
        # ),
        
        # =========================================================
        # 4. OBSTACLE SPAWNER For Gazebo CLassic
        # =========================================================
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     name='spawn_obstacle',
        #     arguments=[
        #         '-entity', 'cbf_obstacle',
        #         '-file', cbf_box_path,
        #         '-x', '2.5',
        #         '-y', '-0.2',  
        #         '-z', '0.5'
        #     ],
        #     output='screen'
        # ),
    ])