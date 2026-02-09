import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Import custom interfaces
from camera_to_opensim_interfaces.msg import PersonArray, Person, Keypoint2D

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        
        # Parameters to configure rtmlib
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('backend', 'onnxruntime')
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.backend = self.get_parameter('backend').get_parameter_value().string_value

        self.get_logger().info(f'Initializing Pose Estimation Node with device={self.device}, backend={self.backend}')

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Initialize RTMLib
        try:
            from rtmlib import RTMPose, Body
            # Using defaults for now, can be parameterized
            self.pose_model = Body(
                to_openpose=False, # We keep native format or adjust as needed
                mode='balanced', 
                backend=self.backend, 
                device=self.device
            )
            self.get_logger().info('RTMLib initialized successfully')
        except ImportError:
            self.get_logger().error('rtmlib not found. Please install rtmlib (pip install rtmlib). Running in MOCK mode.')
            self.pose_model = None

        # Subscribers
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        
        # Publishers
        self.publisher_ = self.create_publisher(PersonArray, '/body_tracking/keypoints_2d', 10)
        
        # COCO Keypoints Mapping (approximate, for reference)
        # 0: Nose, 1: LEye, 2: REye, 3: LEar, 4: REar, 5: LShoulder, 6: RShoulder
        # 7: LElbow, 8: RElbow, 9: LWrist, 10: RWrist, 11: LHip, 12: RHip
        # 13: LKnee, 14: RKnee, 15: LAnkle, 16: RAnkle
        self.keypoint_names = [
            "Nose", "Left Eye", "Right Eye", "Left Ear", "Right Ear", 
            "Left Shoulder", "Right Shoulder", "Left Elbow", "Right Elbow", 
            "Left Wrist", "Right Wrist", "Left Hip", "Right Hip", 
            "Left Knee", "Right Knee", "Left Ankle", "Right Ankle"
        ]

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge conversion failed: {e}')
            return

        persons_msg = PersonArray()
        persons_msg.header = msg.header

        if self.pose_model:
            # Inference
            keypoints, scores = self.pose_model(cv_image)
            
            # keypoints shape: (N, 17, 2), scores shape: (N, 17)
            # Iterate over detected persons
            for i in range(len(keypoints)):
                person_msg = Person()
                person_msg.id = i # Simple ID assignment, tracking would require more logic
                
                kps = keypoints[i]
                scs = scores[i]
                
                for j in range(len(kps)):
                    kp_msg = Keypoint2D()
                    kp_msg.name = self.keypoint_names[j] if j < len(self.keypoint_names) else f"KP_{j}"
                    kp_msg.x = float(kps[j][0])
                    kp_msg.y = float(kps[j][1])
                    kp_msg.confidence = float(scs[j])
                    person_msg.keypoints.append(kp_msg)
                
                persons_msg.persons.append(person_msg)
                
        else:
            # Mock Data
            # self.get_logger().warn('Publishing mock keypoints')
            person_msg = Person()
            person_msg.id = 0
            
            # Create a simple mock skeleton (e.g. cross shape)
            mock_skeleton = {
                "Nose": (320, 100), "Left Eye": (330, 90), "Right Eye": (310, 90),
                "Left Shoulder": (350, 150), "Right Shoulder": (290, 150),
                "Left Hip": (340, 300), "Right Hip": (300, 300),
                "Left Knee": (340, 400), "Right Knee": (300, 400),
                "Left Ankle": (340, 500), "Right Ankle": (300, 500)
            }
            
            for name, (x, y) in mock_skeleton.items():
                kp_msg = Keypoint2D()
                kp_msg.name = name
                kp_msg.x = float(x)
                kp_msg.y = float(y)
                kp_msg.confidence = 0.9
                person_msg.keypoints.append(kp_msg)
            
            persons_msg.persons.append(person_msg)

        self.publisher_.publish(persons_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
