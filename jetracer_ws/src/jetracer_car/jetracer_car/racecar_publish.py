import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from getkey import getkey
import time

class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop')
        self.publisher_throttle = self.create_publisher(Float32, 'throttle', 80)
        self.publisher_steering = self.create_publisher(Float32, 'steering', 80)
        self.get_logger().info("Running teleop.py")
        self.get_logger().info("Z: Forward")
        self.get_logger().info("S: Backward")
        self.get_logger().info("Q: Turn left")
        self.get_logger().info("D: Turn right")
        self.get_logger().info("-: Throttle stop")
        self.get_logger().info("E: Steering reset")

    def run_loop(self):
        throttle_step = 0.25
        steering_step = 0.1
        while rclpy.ok():
            try:
                key = getkey()
            except Exception as e:
                self.get_logger().error(f"Error reading key: {e}")
                continue

            if key == 'z':
                msg = Float32(data=min(1.0,  throttle_step))
                self.publisher_throttle.publish(msg)
            elif key == 's':
                msg = Float32(data=-max(1.0, throttle_step))
                self.publisher_throttle.publish(msg)
            elif key == 'q':
                msg = Float32(data=max(-1.0, -steering_step))
                self.publisher_throttle.publish(msg)
            elif key == 'd':
                msg = Float32(data=min(1.0, steering_step))
                self.publisher_steering.publish(msg)
            elif key == 'e':
                msg = Float32(data=0.0)
                self.publisher_steering.publish(msg)
            
            time.sleep(0.1)  

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass