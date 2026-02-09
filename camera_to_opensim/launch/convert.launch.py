import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Arguments
    use_realsense_depth_arg = DeclareLaunchArgument(
        'use_realsense_depth',
        default_value='true',
        description='Use RealSense Depth for 3D reconstruction (True) or fallback to Sports2D (False)'
    )
    
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value=os.path.join(os.path.expanduser('~'), 'opensim_data'),
        description='Directory to save TRC and OpenSim Data'
    )
    
    pose2sim_path_arg = DeclareLaunchArgument(
        'pose2sim_path',
        default_value='',
        description='Path to Pose2Sim repository (if not installed)'
    )
    
    sports2d_path_arg = DeclareLaunchArgument(
        'sports2d_path',
        default_value='',
        description='Path to Sports2D repository (if not installed)'
    )

    launch_realsense_arg = DeclareLaunchArgument(
        'launch_realsense',
        default_value='false',
        description='Whether to launch realsense2_camera node'
    )

    # Launch Configurations
    use_realsense_depth = LaunchConfiguration('use_realsense_depth')
    output_dir = LaunchConfiguration('output_dir')
    pose2sim_path = LaunchConfiguration('pose2sim_path')
    sports2d_path = LaunchConfiguration('sports2d_path')
    launch_realsense = LaunchConfiguration('launch_realsense')

    # Nodes
    
    # RealSense Camera (Optional)
    # Assuming realsense2_camera package is installed
    try:
        realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
            ),
            condition=IfCondition(launch_realsense),
            launch_arguments={'align_depth.enable': 'true'}.items()
        )
    except Exception:
        # Fallback if package not found (for dev/test without hw)
        realsense_launch = Node(
            package='dummy', executable='dummy', condition=IfCondition('false')
        )

    # Pose Estimation Node (2D) - Detection
    pose_estimation_node = Node(
        package='camera_to_opensim',
        executable='pose_estimation_node',
        name='pose_estimation_node',
        parameters=[{
            'device': 'cpu', # Make configurable if needed
            'backend': 'onnxruntime'
        }],
        condition=IfCondition(use_realsense_depth)
    )
    
    # Data Processing Node (3D + TRC + IK)
    data_processing_node = Node(
        package='camera_to_opensim',
        executable='data_processing_node',
        name='data_processing_node',
        parameters=[{
            'output_dir': output_dir,
            'pose2sim_path': pose2sim_path
        }],
        condition=IfCondition(use_realsense_depth)
    )
    
    # Sports2D Node (Fallback)
    sports2d_node = Node(
        package='camera_to_opensim',
        executable='sports2d_node',
        name='sports2d_node',
        parameters=[{
            'output_dir': output_dir,
            'sports2d_path': sports2d_path
        }],
        condition=UnlessCondition(use_realsense_depth)
    )

    return LaunchDescription([
        use_realsense_depth_arg,
        output_dir_arg,
        pose2sim_path_arg,
        sports2d_path_arg,
        launch_realsense_arg,
        realsense_launch,
        pose_estimation_node,
        data_processing_node,
        sports2d_node
    ])
