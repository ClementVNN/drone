import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from jetracerpy.jetracer_client import JetRacerClient





class RacecarNode(Node):

    def __init__(self):
        super().__init__('racecar')
        JETRACER_IP = "192.168.4.1"

         # Initialize the JetRacer client
        jetracer = JetRacerClient(jetracer_ip=JETRACER_IP)

        self.car=jetracer
   
        
        self.current_throttle = 0.0
        self.current_steering = 0.0

        
        self.subscription_throttle = self.create_subscription(
            Float32, 'throttle', self.callback_throttle, 10)
        self.subscription_steering = self.create_subscription(
            Float32, 'steering', self.callback_steering, 10)
        
        try:
            self.car.start()
            self.car.stream_on()
            self.get_logger().info('CA MARCCCCCCCCCCCCCCCCCCCCCCCCCCHE.')
        except Exception as e:
            self.get_logger().error(f'BAH NON CA MARCHE PÄAAAAAAAAAAAAAAAAAAAAAAS: {e}')


    def callback_throttle(self, msg):
    
        self.current_throttle=msg.data 
        self.car.send_rc_control(self.current_throttle, self.current_steering)
        self.get_logger().info(f'Throttle: {msg.data}')

    def callback_steering(self, msg):

        self.current_steering=msg.data 
        self.car.send_rc_control(self.current_throttle, self.current_steering)
        self.get_logger().info(f'Steering: {msg.data}')
     

  

def main(args=None):
    rclpy.init(args=args)
    racecar_node = RacecarNode()
    rclpy.spin(racecar_node)
    racecar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # JETRACER_IP = "192.168.4.1"

    # # Initialize the JetRacer client
    # jetracer = JetRacerClient(jetracer_ip=JETRACER_IP)

    # # Example usage of the SDK ,
    # print("Starting JetRacer...")
    # jetracer.start()

    # jetracer.stream_on()


    # Initialize throttle and steering
    # jetracer.current_throttle = 0.15
    # jetracer.current_steering = 0.15
    # jetracer.send_rc_control(jetracer.current_throttle, jetracer.current_steering)

    # Example usage of the SDK
    print("Starting JetRacer...")
    print("Running racecar.py")
    main()