
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import time

rclpy.init()
node = Node('mock_publisher')
pub_c = node.create_publisher(Image, '/camera/color/image_raw', 10)
pub_d = node.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 10)
bridge = CvBridge()

for i in range(10):
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    depth = np.ones((480, 640), dtype=np.uint16) * 1000 # 1 meter
    
    msg_c = bridge.cv2_to_imgmsg(img, 'bgr8')
    msg_d = bridge.cv2_to_imgmsg(depth, '16UC1')
    msg_c.header.frame_id = 'camera_link'
    msg_d.header.frame_id = 'camera_link'
    
    now = node.get_clock().now().to_msg()
    msg_c.header.stamp = now
    msg_d.header.stamp = now
    
    pub_c.publish(msg_c)
    pub_d.publish(msg_d)
    print(f"Published frame {i}")
    time.sleep(0.5)

node.destroy_node()
rclpy.shutdown()
