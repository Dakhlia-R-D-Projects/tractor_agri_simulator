import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class ImageInferenceNode(Node):
    def __init__(self):
        super().__init__('image_inference_node')
        
        # Initialize YOLO model
        self.model = YOLO('best.pt')  # Replace 'best.pt' with your model path if different
        
        # Create a subscriber to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/left/rgb/image',
            self.image_callback,
            1
        )
        
        # Create a publisher for the processed image
        self.publisher = self.create_publisher(
            Image,
            'image_processed',
            1
        )
        
        # Initialize CvBridge
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Perform inference using YOLO
        results = self.model.predict(cv_image,conf=0.7,augment=True)

        # Draw bounding boxes on the image
        for result in results:  # Iterate over each result
            for box in result.boxes:  # Access bounding boxes
                x1, y1, x2, y2 = box.xyxy[0].tolist()  # Bounding box coordinates
                conf = box.conf[0]  # Confidence score
                cls = box.cls[0]  # Class index
                label = f"{self.model.names[int(cls)]} {conf:.2f}"
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish the processed image
        processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.publisher.publish(processed_msg)

        # Display the image in a window
        cv2.imshow('Processed Image', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    
    node = ImageInferenceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
