import threading
from dobot_mg400.dobot_api.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import time
import numpy as np
from  dobot_mg400.dobot_api.cinematica_inversa import *
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import String
import numpy as np
  
class Dobot(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('dobot')
       
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(String, 'color', self.listener_callback, 10)
    self.subscription # prevent unused variable warning
    self.ip = "192.168.1.6"
    self.movePort = 30003
    self.feedPort = 30004
    self.dashboardPort = 29999
    self.move = DobotApiMove(self.ip, self.movePort)
    self.feed = DobotApi(self.ip, self.feedPort)

  def listener_callback(self, data):

    self.get_logger().info('Recibido: "%s"' %data.data)
    _, feed_joint = self.GetFeed(self.feed)
    if data.data == 'Rojo':
        q1 = calcular_cinematica_inversa([355.52,-60.28,-180,-33.9], np.deg2rad(feed_joint[:4]))
        self.RunPoint(self.move, np.rad2deg(q1))
        self.move.Sync()
        self.dashboard.DO(2,1)
        time.sleep(1.5)
        self.dashboard.DO(2,0)
        _, feed_joint2 = self.GetFeed(self.feed)
        q2 = calcular_cinematica_inversa([213.13,132.07,-98.45,-33.9], np.deg2rad(feed_joint2[:4]))
        self.RunPoint(self.move, np.rad2deg(q2))
        self.move.Sync()
        self.dashboard.DO(1,1)
        time.sleep(0.5)
        self.dashboard.DO(1,0)
        self.Home(self.move)

    elif data.data == 'Verde':
        q1 = calcular_cinematica_inversa([355.52,-60.28,-180,-33.9], np.deg2rad(feed_joint[:4]))
        self.RunPoint(self.move, np.rad2deg(q1))
        self.move.Sync()
        self.dashboard.DO(2,1)
        time.sleep(1.5)
        self.dashboard.DO(2,0)
        _, feed_joint2 = self.GetFeed(self.feed)
        q2 = calcular_cinematica_inversa([210.64,181.1,-98.45,-33.9], np.deg2rad(feed_joint2[:4]))
        self.RunPoint(self.move, np.rad2deg(q2))
        self.move.Sync()
        self.dashboard.DO(1,1)
        time.sleep(0.5)
        self.dashboard.DO(1,0)
        self.Home(self.move)
    elif data.data == 'Azul':
        q1 = calcular_cinematica_inversa([355.52,-60.28,-180,-33.9], np.deg2rad(feed_joint[:4]))
        self.RunPoint(self.move, np.rad2deg(q1))
        self.move.Sync()
        self.dashboard.DO(2,1)
        time.sleep(1.5)
        self.dashboard.DO(2,0)
        _, feed_joint2 = self.GetFeed(self.feed)
        q2 = calcular_cinematica_inversa([215.48,-195.81,-98.45,-33.9], np.deg2rad(feed_joint2[:4]))
        self.RunPoint(self.move, np.rad2deg(q2))
        self.move.Sync()
        self.dashboard.DO(1,1)
        time.sleep(0.5)
        self.dashboard.DO(1,0)
        self.Home(self.move)
    elif data.data == 'Amarillo':
        q1 = calcular_cinematica_inversa([355.52,-60.28,-180,-33.9], np.deg2rad(feed_joint[:4]))
        self.RunPoint(self.move, np.rad2deg(q1))
        self.move.Sync()
        self.dashboard.DO(2,1)
        time.sleep(1.5)
        self.dashboard.DO(2,0)
        _, feed_joint2 = self.GetFeed(self.feed)
        q2 = calcular_cinematica_inversa([210.64,230.12,-98.45,-33.9], np.deg2rad(feed_joint2[:4]))
        self.RunPoint(self.move, np.rad2deg(q2))
        self.move.Sync()
        self.dashboard.DO(1,1)
        time.sleep(0.5)
        self.dashboard.DO(1,0)
        self.Home(self.move)
    else:
        self.Home(self.move)


  def ConnectRobot(self):
    try:
        ip = "192.168.1.6"
        dashboardPort = 29999
        movePort = 30003
        feedPort = 30004
        print("Estableciendo conexión...")
        dashboard = DobotApiDashboard(ip, dashboardPort)
        move = DobotApiMove(ip, movePort)
        feed = DobotApi(ip, feedPort)
        dashboard.ClearError()
        print("¡Conexión exitosa!")
        return dashboard, move, feed
    except Exception as e:
        print("Error al conectar :(")
        raise e


  def RunPoint(self, move: DobotApiMove, point_list: list):
    move.JointMovJ(point_list[0], point_list[1], point_list[2], point_list[3])
  
  def Home(self, move: DobotApiMove):
    move.JointMovJ(0.1, 0.1, 0.1, 0.1)

  def GetFeed(self, feed: DobotApi):
    current_actual = None
    feed_joint = None
    globalLockValue = threading.Lock()
    hasRead = 0

    data = bytes()
    while hasRead < 1440:
        temp = feed.socket_dobot.recv(1440 - hasRead)
        if len(temp) > 0:
            hasRead += len(temp)
            data += temp

    feedInfo = np.frombuffer(data, dtype=MyType)
    if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
        with globalLockValue:
            # Refresh Properties
            current_actual = feedInfo["tool_vector_actual"][0]
            feed_joint = feedInfo['q_actual'][0]

    return current_actual, feed_joint

  def clear_error(self, dashboard: DobotApi):
    dashboard.ClearError()

        
        # Used to convert between ROS and OpenCV images
        #self.br = ros2_np_multiarray()
   
def main(args=None):
    rclpy.init(args=args)
    dobot = Dobot()
    try:
        dashboard, move, feed = dobot.ConnectRobot()
        dobot.dashboard = dashboard
        dashboard.EnableRobot()
        rclpy.spin(dobot)
    except KeyboardInterrupt:
        pass
    finally:
        dobot.destroy_node()
        dashboard.DisableRobot()
        dashboard.close()
        feed.close()
        move.close()
        rclpy.shutdown()

   
if __name__ == '__main__':
  main()