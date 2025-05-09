import rclpy
from rclpy.node import Node
from do_interfaces.msg import DO
import rclpy.time
from std_msgs.msg import String
import struct

class DODecoder(Node):
    # stdo,HEX,HEX

    def __init__(self, input_topic, output_topic):
        super().__init__('do_driver')
        self.get_logger().info("Starting DO driver node to decode raw DO data")
        self.publisher = self.create_publisher(DO, output_topic, 10)
        self.subscriber = self.create_subscription(
            String,
            input_topic,
            self.listener_callback,
            10
        )

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

def main(args=None):
    rclpy.init(args=args)
    # TODO: Parse in/out topic from config file or launch file
    node = DODecoder(
        input_topic='/do/raw',
        output_topic='/do'
    )
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()