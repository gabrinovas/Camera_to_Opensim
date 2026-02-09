import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
from std_srvs.srv import Trigger
from .sports2d_wrapper import run_sports2d

class Sports2DNode(Node):
    def __init__(self):
        super().__init__('sports2d_node')
        
        # Parameters
        self.declare_parameter('output_dir', os.path.expanduser('~/opensim_data'))
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.declare_parameter('sports2d_path', '')
        self.sports2d_path = self.get_parameter('sports2d_path').get_parameter_value().string_value

        os.makedirs(self.output_dir, exist_ok=True)
        
        self.bridge = CvBridge()
        self.recording = False
        self.video_writer = None
        self.video_path = None
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
            
        # Services
        self.start_srv = self.create_service(Trigger, 'start_recording', self.start_recording)
        self.stop_srv = self.create_service(Trigger, 'stop_recording', self.stop_recording)
        
        self.get_logger().info('Sports2D Node Initialized. Ready to record.')

    def start_recording(self, request, response):
        if self.recording:
            response.success = False
            response.message = "Already recording."
            return response
            
        self.recording = True
        filename = f'video_{datetime.now().strftime("%Y%m%d_%H%M%S")}.mp4'
        self.video_path = os.path.join(self.output_dir, filename)
        
        # Video Writer initialized on first frame to get dimensions
        self.video_writer = None
        
        response.success = True
        response.message = f"Recording started. Saving to {self.video_path}"
        self.get_logger().info(f"Recording started: {self.video_path}")
        return response

    def stop_recording(self, request, response):
        if not self.recording:
            response.success = False
            response.message = "Not recording."
            return response
            
        self.recording = False
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
            
        response.success = True
        response.message = f"Recording saved to {self.video_path}"
        self.get_logger().info(f"Recording stopped.")
        
        # Trigger Sports2D Processing
        self.process_video(self.video_path)
        
        return response

    def image_callback(self, msg):
        if not self.recording:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge failed: {e}")
            return
            
        if self.video_writer is None:
            height, width, _ = cv_image.shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v') # or XVID
            self.video_writer = cv2.VideoWriter(self.video_path, fourcc, 30.0, (width, height))
            
        self.video_writer.write(cv_image)

    def process_video(self, video_path):
        if not os.path.exists(video_path):
            self.get_logger().error(f"Video file not found: {video_path}")
            return

        self.get_logger().info(f"Starting Sports2D processing on {video_path}...")
        try:
            # If a custom path is provided, try to add it
            if self.sports2d_path:
                 import sys
                 if self.sports2d_path not in sys.path:
                     sys.path.append(self.sports2d_path)

            run_sports2d(video_path, self.output_dir)
            self.get_logger().info("Sports2D processing completed.")
        except Exception as e:
            self.get_logger().error(f"Sports2D processing failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Sports2DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
