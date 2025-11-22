import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        # Load YOLOv3 model
        self.net = cv2.dnn.readNet(
    "/home/sneh/Sneh_project_adjustments/src/yolov3.weights",
    "/home/sneh/Sneh_project_adjustments/src/yolov3.cfg"
)
        self.layer_names = self.net.getLayerNames()
        unconnected_layers = self.net.getUnconnectedOutLayers()
        self.output_layers = [self.layer_names[i - 1] for i in unconnected_layers]

        with open("/home/sneh/Sneh_project_adjustments/src/coco.names", "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

    def listener_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Perform object detection
        blob = cv2.dnn.blobFromImage(current_frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        # Process detections
        class_ids = []
        confidences = []
        boxes = []
        height, width, channels = current_frame.shape
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.85:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Non-max suppression to remove overlapping bounding boxes
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                confidence = confidences[i]
                color = (0, 255, 0)
                cv2.rectangle(current_frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(current_frame, f"{label} {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow("Camera Feed", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
