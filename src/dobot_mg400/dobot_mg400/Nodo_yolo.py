#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from time import time

class DetectorColores(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('detector_colores')
        self.subscription = self.create_subscription(Image, 'video_frames', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(String, 'color', 10)
        self.br = CvBridge()
        self.last_published_time = 0
   
    def listener_callback(self, data):
        """
        Callback function.
        """
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        hsvFrame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Define adjusted color ranges for red, blue, green, and yellow
        color_ranges = {
            "Rojo": ([160, 100, 100], [179, 255, 255]),
            "Azul": ([94, 80, 80], [126, 255, 255]),
            "Verde": ([40, 52, 72], [70, 255, 255]),
            "Amarillo": ([20, 100, 100], [30, 255, 255])
        }

        max_area = 0
        predominant_color = None

        for color_name, (lower, upper) in color_ranges.items():
            lower_np = np.array(lower, np.uint8)
            upper_np = np.array(upper, np.uint8)
            mask = cv2.inRange(hsvFrame, lower_np, upper_np)
            mask = cv2.dilate(mask, np.ones((5, 5), "uint8"))
            res = cv2.bitwise_and(current_frame, current_frame, mask=mask)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            total_area = sum(cv2.contourArea(contour) for contour in contours)

            if total_area > max_area:
                max_area = total_area
                predominant_color = color_name

        current_time = time()
        if predominant_color in color_ranges and (current_time - self.last_published_time) >= 3.5:
            self.get_logger().info(f'Detected predominant color: {predominant_color}')
            self.publisher.publish(String(data=predominant_color))
            self.last_published_time = current_time
        else:
            self.get_logger().info('No predominant color detected or detected color is not in defined ranges')

        cv2.imshow("Detector de Colores", current_frame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    detector_colores = DetectorColores()
    rclpy.spin(detector_colores)
    detector_colores.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



