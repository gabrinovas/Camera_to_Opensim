import subprocess
import time
import os
import signal

def run_verification():
    print("Starting Verification...")
    
    # 1. Launch the nodes
    cmd = ["ros2", "launch", "camera_to_opensim", "convert.launch.py", "use_realsense_depth:=true", "launch_realsense:=false"]
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    
    try:
        time.sleep(5) # Wait for nodes to start
        print("Nodes launched.")
        
        # 2. Publish Camera Info (Mock)
        subprocess.run([
            "ros2", "topic", "pub", "--once", "/camera/aligned_depth_to_color/camera_info", "sensor_msgs/msg/CameraInfo",
            "{header: {frame_id: 'camera_link'}, width: 640, height: 480, k: [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]}"
        ], check=True)
        print("Camera Info Published.")
        
        # 3. Start Recording
        subprocess.run(["ros2", "service", "call", "/start_recording", "std_srvs/srv/Trigger"], check=True)
        print("Recording Started.")
        
        # 4. Publish Mock Images (Depth and Color) - The node needs them to trigger callback
        # We publish a few frames
        for i in range(5):
            # Depth: All 1000mm (1m)
            # 16UC1 is hard to mock via command line easily with data, but we can try empty or simple
            # Actually, pose_estimation_node publishes keypoints independently of input image in mock mode?
            # No, image_callback triggers it. So we must publish color image.
            
            # Color Image
            subprocess.Popen([
                "ros2", "topic", "pub", "--once", "/camera/color/image_raw", "sensor_msgs/msg/Image",
                "{header: {frame_id: 'camera_link'}, height: 480, width: 640, encoding: 'bgr8', step: 1920, data: [0]*921600}"
            ])
            
            # Depth Image (1m everywhere = 1000 in uint16. 1000 = 0x03E8. Low byte E8=232, High 03=3)
            # data length: 640*480*2 = 614400
            # constructing that huge array in CLI is bad.
            # Let's write a small python publisher instead.
            pass
            time.sleep(0.5)

        # Better: Use a python script to pub images
        print("Publishing images via sub-script...")
        pub_script = """
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
"""
        with open("mock_pub.py", "w") as f:
            f.write(pub_script)
            
        subprocess.run(["python3", "mock_pub.py"], check=True)
        
        # 5. Stop Recording
        subprocess.run(["ros2", "service", "call", "/stop_recording", "std_srvs/srv/Trigger"], check=True)
        print("Recording Stopped.")
        
    finally:
        os.kill(process.pid, signal.SIGINT)
        process.wait()

    # Check output
    # Assuming default output_dir is ~/opensim_data
    output_dir = os.path.expanduser("~/opensim_data/pose-3d")
    if os.path.exists(output_dir):
        files = os.listdir(output_dir)
        trc_files = [f for f in files if f.endswith(".trc")]
        if trc_files:
            print(f"Success! Generated TRC files: {trc_files}")
        else:
            print("Failure: No TRC files found.")
    else:
        print(f"Failure: Output directory {output_dir} does not exist.")

if __name__ == "__main__":
    run_verification()
