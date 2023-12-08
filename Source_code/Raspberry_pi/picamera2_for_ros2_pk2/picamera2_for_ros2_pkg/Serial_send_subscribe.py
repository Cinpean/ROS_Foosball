import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, time

class MotorSubscriber(Node):

    def __init__(self):
        super().__init__("motor_subscriber")
        self.motor_subscriber = self.create_subscription(String, '/motor_topic', self.motor_callback, 10)
        self.motor_subscriber  # prevent unused variable warning
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1,xonxoff=True)
        time.sleep(1) #give the connection a second to settle
        self.ser.reset_input_buffer()

    def motor_callback(self, msg):
        self.get_logger().info(str(msg))
        self.ser.write((msg.data + '\n').encode('utf-8'))
        echo_line = self.ser.readline().decode('utf-8').strip()

def main(args=None):
    rclpy.init(args=args)
    motorSubscriber = MotorSubscriber()
    rclpy.spin(motorSubscriber)
    motorSubscriber.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()