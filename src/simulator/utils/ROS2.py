import rclpy
from rclpy.node import Node
import json
import time

from std_msgs.msg import String

class ROS2_conn():
    # start_time = 0
    def __init__(self):

        self.seer_node = Node('seer_publisher')
        self.seer_node.publisher_ = self.seer_node.create_publisher(String, 'live_report', 10)

    def seer_consumer(self, message, msg_idx):  
        msg = String()

        # Formating the message that send construct scenario information
        if msg_idx == 1:
            scenario = []
            data = {}

            for _, j in enumerate(message):
                scenario.append({j: message[j]})

            data["scenario"] = scenario
            msg.data = json.dumps(data)

        else:
            msg.data = json.dumps(message)
        
        # if 'timestamp' in message and message['timestamp'] != -1:
        #     timestamp = message['timestamp']
        #     if timestamp == 0:
        #         self.start_time = time.time()
        #     else:
        #         send_time = self.start_time + timestamp
        #         sleep_time = send_time - time.time()
        #         sleep_time = sleep_time if sleep_time > 0 else 0
        #         time.sleep(sleep_time)

        # Delay to control the publish rate because the subscriber is more slow 
        # and message may be lost
        time.sleep(0.02)

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
        
        
        
        
