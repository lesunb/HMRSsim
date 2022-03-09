import rclpy
from rclpy.node import Node
import json

from std_msgs.msg import String

class ROS2_conn():
    def __init__(self):
        rclpy.init(args=None)

        self.seer_node = Node('seer_publisher')
        self.seer_node.publisher_ = self.seer_node.create_publisher(String, 'listener', 10)

    def seer_consumer(self, message, msg_idx):  
        msg = String()
        msg.data = json.dumps(message)
        self.seer_node.publisher_.publish(msg)

    def close(self):
        self.seer_node.destroy_node()
        rclpy.shutdown()
        
def main():
    ros2 = ROS2_conn()
    msg = {'data' : 'Hello ROS2!'}
    ros2.seer_consumer(msg, 0)
    ros2.close()
    
if __name__ == '__main__':
    main()
        
        
        
        
