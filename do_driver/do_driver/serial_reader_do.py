import rclpy
from std_msgs.msg import String
import serial
import time

def main():
    port = '/dev/ttyUSB2'
    baudrate = 38400    

    rclpy.init()
    node = rclpy.create_node('serial_reader_do')
    
    # Create a publisher to publish the data
    publisher = node.create_publisher(String, '/do/raw', 2)
    
    # Open the serial port
    connected = False
    while not connected and rclpy.ok():
        try:
            ser = serial.Serial(port, baudrate, timeout=1)  
            connected = True
        except serial.SerialException as e:
            node.get_logger().error(f"Error opening serial port: {e}")
            time.sleep(1)
    
    while rclpy.ok() and connected:
        try:
            # Send the request to the sensor
            ser.write(b'stdo,19\r\n')

            # Read the response from the sensor
            line = ser.readline().decode('utf-8').strip()
            if len(line) > 0:
                # Create a String message and publish it
                msg = String()
                msg.data = line
                publisher.publish(msg)
                node.get_logger().info(f'Publishing: {msg.data}')
                
            time.sleep(0.1)  
        except Exception as e:
            node.get_logger().error(f"Error during communication: {e}")
            time.sleep(0.1)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()