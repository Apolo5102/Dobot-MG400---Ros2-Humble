import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 
class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('img_publisher')
       
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
       
    # We will publish a message every 0.1 seconds
    timer_period = 0.1  # seconds
       
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
         
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(2)
         
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    ret, frame = self.cap.read()
           
    if ret == True:
        # Obtener dimensiones de la imagen
        height, width = frame.shape[:2]
       
        # Calcular las coordenadas del área de recorte
        center_x = width // 2
        center_y = height // 2
        half_width = 50  # Mitad del ancho del área de recorte
        half_height = 50  # Mitad de la altura del área de recorte
        x1 = max(0, center_x - half_width)
        y1 = max(0, center_y - half_height)
        x2 = min(width, center_x + half_width)
        y2 = min(height, center_y + half_height)
       
        # Recortar la imagen para seleccionar el área de interés
        cropped_frame = frame[y1:y2, x1:x2]
       
        # Redimensionar la imagen al tamaño deseado (100x100)
        resized_frame = cv2.resize(cropped_frame, (100, 100))
     
        # Convertir la imagen recortada y redimensionada a un mensaje de imagen de ROS 2
        ros_image = self.br.cv2_to_imgmsg(resized_frame)
       
        # Publicar el mensaje de imagen en el tópico 'video_frames'
        self.publisher_.publish(ros_image)
       
    # Display the message on the console
    self.get_logger().info('Publishing video frame')

   
def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  img_publisher = ImagePublisher()
   
  # Spin the node so the callback function is called.
  rclpy.spin(img_publisher)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  img_publisher.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()
