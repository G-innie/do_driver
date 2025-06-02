import rclpy
from rclpy.node import Node
from do_interfaces.msg import DO
import rclpy.time
from std_msgs.msg import String
import struct

class DODecoder(Node):
    # stdo,HEX,HEX

    def __init__(self):
        super().__init__('do_driver')

        self.declare_parameter("input_topic", "do_raw")
        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value

        self.declare_parameter("output_topic", "do_parsed")
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self.declare_parameter("request_topic", "do_request")
        self.request_topic = self.get_parameter("request_topic").get_parameter_value().string_value

        self.get_logger().info("Starting DO driver node to decode raw DO data")
        self.publisher = self.create_publisher(DO, self.output_topic, 10)
        self.request_publisher = self.create_publisher(String, self.request_topic, 10)
        self.subscriber = self.create_subscription(
            String,
            self.input_topic,
            self.listener_callback,
            10
        )

        self.do_timer = self.create_timer(1, self.timer_callback)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received message: {msg.data}")
        msg = msg.data.strip(',\r\n').split(',')

        if len(msg) != 4:
            self.get_logger().error(f"Invalid message format. Length should be 5, got {len(msg)}")
            return
        
        # decode temperature
        temperature = struct.unpack('>h', bytes.fromhex(msg[1]))[0]
        temperature /= 1000
        temperature -= 5

        # decode dissolved oxygen
        dissolved_oxygen = struct.unpack('>H', bytes.fromhex(msg[2]))[0]
        dissolved_oxygen /= 100

        do_msg = DO()
        do_msg.time = self.get_clock().now().to_msg()
        do_msg.temperature = temperature
        do_msg.dissolved_oxygen = dissolved_oxygen
        do_msg.check_sum = msg[3]
   
        self.get_logger().info(f"Publishing message: {do_msg}")
        self.publisher.publish(do_msg)

    def timer_callback(self):    
        msg = String()
        msg.data = 'stdo,19\r\n'
        self.request_publisher.publish(msg)
        self.get_logger().info(f"Published request: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = DODecoder()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()