import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from IntegrationClasses_v2 import *
from ultralytics import YOLO
import cv2
import time

# Attempt to import ServoKit for servo control
try:
    from adafruit_servokit import ServoKit
except ImportError:
    ServoKit = None


class rceti_AI(Node):
    def __init__(self):
        super().__init__('rceti_AI')
        self.get_logger().info('AI node initialized.')

        # Parameters that control model and video source
        self.declare_parameter('model_path', '../runs/detect/train41/weights/best.pt')  # Default model path
        self.declare_parameter('video_path', '0')  # Default to webcam, '0'

        # Get parameter values
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        video_param = self.get_parameter('video_path').get_parameter_value().string_value
        self.cap_source = int(video_param) if video_param.isdigit() else video_param

        # State variables to track detection status
        self.running = False  # Is detection running?
        self.last_frame_time = None  # To track time between frames
        self.servo1 = None 
        self.servo2 = None
        self.kit = None  # ServoKit controller
        self.state = None  # The state machine that controls the detection loop
        self.cap = None  # Video capture
        self.model = None  # YOLO model
        self.timer = None  # Timer to trigger frame processing

        # Interfaces
        self.status_pub = self.create_publisher(String, '/servo_detector/status', 10)  # Topic to publish status updates
        self.image_pub = self.create_publisher(Image, '/servo_detector/image_annotated', 10)  # Topic to publish annotated frames
        self.bridge = CvBridge()  # Convert OpenCV images to ROS images

        # Services to start and stop the detection
        self.start_srv = self.create_service(Trigger, '/servo_detector/start', self.handle_start)
        self.stop_srv = self.create_service(Trigger, '/servo_detector/stop', self.handle_stop)

        # Initialize servo controls
        self.init_servos()

        # Publish initial status
        self.publish_status("idle")

    def init_servos(self):
        # Initialize servo control (if ServoKit is available)
        if ServoKit:
            try:
                self.kit = ServoKit(channels=16)  # Initialize 16-channel servo kit
                self.servo1 = self.kit.servo[0]  # Assign first servo
                self.servo2 = self.kit.servo[1]  # Assign second servo
                self.servo1.angle = 90  # Set initial angle to middle
                self.servo2.angle = 90  # Set initial angle to middle
                self.get_logger().info("Servos initialized to 90 degrees.")
            except Exception as e:
                # If ServoKit initialization fails, log a warning
                self.get_logger().warn(f"ServoKit initialization failed: {e}")
        else:
            # If ServoKit is not available, log a warning
            self.get_logger().warn("ServoKit not found.")

    def init_camera_and_model(self):
        # Initialize YOLO model and video capture
        self.model = YOLO(self.model_path)  # Load YOLO model
        self.cap = cv2.VideoCapture(self.cap_source)  # Open video capture

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video source: {self.cap_source}")
            return False

        # If using a video file (not webcam), skip to frame 1000
        if not isinstance(self.cap_source, int):
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 1000)

        # Set video resolution (640x480)
        self.cap.set(3, 640)
        self.cap.set(4, 480)

        # Initialize the state machine to control detection and servo actions
        self.state = EvaluatingAnatomyState(
            None, 3, 0, 0.15, self.cap, self.model, self.servo1, self.servo2
        )

        return True

    def handle_start(self, request, response):
        # Service handler for starting the detection loop
        if self.running:
            response.success = False
            response.message = "Already running."
        else:
            self.get_logger().info("Starting detection loop...")
            # Initialize the camera and model
            if not self.init_camera_and_model():
                response.success = False
                response.message = "Failed to initialize camera or model."
                return response

            # Reset timing for frame processing
            self.last_frame_time = None
            # Start processing frames at 30ms intervals (33 FPS)
            self.timer = self.create_timer(0.03, self.process_frame)
            self.running = True
            self.publish_status("running")  # Update status to 'running'
            response.success = True
            response.message = "Detection loop started."
        return response

    def handle_stop(self, request, response):
        # Service handler for stopping the detection loop
        if not self.running:
            response.success = False
            response.message = "Already stopped."
        else:
            self.get_logger().info("Stopping detection loop...")
            if self.timer:
                self.timer.cancel()  # Stop the frame processing timer
            if self.cap:
                self.cap.release()  # Release the video capture
            cv2.destroyAllWindows()  # Close any OpenCV windows
            self.running = False
            self.publish_status("idle")  # Update status to 'idle'
            response.success = True
            response.message = "Detection loop stopped."
        return response

    def process_frame(self):
        # This function is called every 30ms to process a new frame
        current_time = time.time()
        dt = current_time - self.last_frame_time if self.last_frame_time else 0
        self.last_frame_time = current_time

        # Update the state machine with elapsed time since last frame
        self.state = self.state.Execute(dt)

        # If the state has a processed frame, publish it as a ROS image
        if hasattr(self.state, 'frame') and self.state.frame is not None:
            try:
                image_msg = self.bridge.cv2_to_imgmsg(self.state.frame, encoding='bgr8')
                self.image_pub.publish(image_msg)
            except Exception as e:
                self.get_logger().warn(f"Image conversion failed: {e}")

        # Check for manual override (press ESC to stop detection)
        k = cv2.waitKey(1) & 0xFF
        if k == 27:  # ESC key
            self.get_logger().info("Manual override triggered.")
            self.state = self.state.ManualOverride()  # Trigger manual override
            self.handle_stop(None, Trigger.Response())  # Stop the detection loop

    def publish_status(self, status: str):
        # Publish the current status of the system
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = rceti_AI()  # Create the node
    try:
        rclpy.spin(node)  # Keep the node running and listening for services
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gracefully...")
    finally:
        # Cleanup when node shuts down
        if node.cap:
            node.cap.release()  # Release video capture resources
        cv2.destroyAllWindows()  # Close OpenCV windows
        node.destroy_node()  # Clean up node
        rclpy.shutdown()  # Shutdown ROS 2


if __name__ == '__main__':
    main()