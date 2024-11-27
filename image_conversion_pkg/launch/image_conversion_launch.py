from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # USB Camera Node
        Node(
            package='usb_cam',
            executable='usb_cam_node',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'mjpeg'
            }]
        ),
        
        # Image Conversion Node
        Node(
            package='image_conversion_pkg',
            executable='image_conversion_node',
            name='image_conversion_node',
            parameters=[{
                'input_topic': '/usb_cam/image_raw',
                'output_topic': '/converted_image'
            }]
        )
    ])
