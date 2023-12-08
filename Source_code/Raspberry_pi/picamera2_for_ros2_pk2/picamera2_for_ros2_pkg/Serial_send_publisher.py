import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Send_msg_motor(Node):
    def __init__(self):
        super().__init__("motor_publisher")
        self.publisher = self.create_publisher(String, 'motor_topic',10 )
    
    def get_keyboard_input(self):
        return input("enter command:")

    def publish_message(self,message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
    
def main():
    rclpy.init()
    my_publisher = Send_msg_motor()

    try:
        while rclpy.ok():
            user_input = my_publisher.get_keyboard_input()
            my_publisher.publish_message(user_input)
    except KeyboardInterrupt:
        print("KeyboardInterrupt: Shutting down.")
    finally:
        my_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()