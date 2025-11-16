import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class ServoToController(Node):
    def __init__(self):
        super().__init__('servo_to_controller')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Float32, '/servo_setpoint_deg', self.cb, 10)
        self.get_logger().info('servo_to_controller listo.')

    def cb(self, msg: Float32):
        tw = Twist()
        tw.linear.x = float(msg.data) / 180.0 * 0.5
        self.pub_cmd.publish(tw)

def main():
    rclpy.init()
    node = ServoToController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
