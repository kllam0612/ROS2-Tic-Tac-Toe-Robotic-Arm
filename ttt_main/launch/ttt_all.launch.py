import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get the path to the config file
    config_dir = os.path.join(
        get_package_share_directory('ttt_control'),
        'config',
        'picknplace_config.yaml',
        get_package_share_directory('ttt_cv'),
        'config',
        'camera_config.yaml',
        get_package_share_directory('ttt_main'),
        'config',
        'game_logic_config.yaml',
    )

    # 2. Define the Node with parameters AND the terminal prefix
    sound_node = Node(
        package='ttt_audio',
        executable='sound_node',
        name='sound_node',
        output='screen',
        parameters=[config_dir],
    )

    picknplace_node = Node(
        package='ttt_control',
        executable='picknplace_node',
        name='picknplace_node',
        output='screen',
        parameters=[config_dir],
    )

    camera_arm_node = Node(
        package='ttt_cv',
        executable='camera_arm_node',
        name='camera_arm_node',
        output='screen',
        parameters=[config_dir],
    )

    vision_node = Node(
        package='ttt_cv',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[config_dir],
    )

    main_orchestrator = Node(
        package='ttt_main',
        executable='main_orchestrator',
        name='main_orchestrator',
        output='screen',
        parameters=[config_dir],
        # --- NEW LINE BELOW ---
        # This opens a new terminal window specifically for this node
        prefix=['gnome-terminal --'] 
    )

    game_logic_node = Node(
        package='ttt_main',
        executable='game_logic_node',
        name='game_logic_node',
        output='screen',
        parameters=[config_dir],
    )

    ui_node = Node(
        package='ttt_ui',
        executable='ui_node',
        name='ui_node',
        output='screen',
        parameters=[config_dir],
        # --- NEW LINE BELOW ---
        # This opens a new terminal window specifically for this node
        prefix=['gnome-terminal --'] 
    )
   
    

    return LaunchDescription([
        sound_node,
        picknplace_node,
        camera_arm_node,
        vision_node,
        main_orchestrator,
        game_logic_node,
        ui_node,
       


    ])
