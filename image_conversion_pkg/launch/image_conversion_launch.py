from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        # USB Camera Node
        LogInfo(msg="Starting USB Cam Node"),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'pixel_format': 'yuyv',
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'camera_calibration_file': ''
            }]
        ),
        
        # Image Conversion Node
        LogInfo(msg="Starting Image Conversion Node"),
        Node(
            package='image_conversion_pkg',
            executable='image_conversion_node',
            name='image_conversion_node',
            parameters=[{
                'input_topic': '/image_raw',
                'output_topic': '/converted_image'
            }]
        )
    ])
