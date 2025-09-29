import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ShapeNode(Node):
    def __init__(self):
        super().__init__('shape_node')
        self.publisher_ = self.create_publisher(String, 'chosen_shape', 10)
        self.timer = self.create_timer(1.0, self.ask_user)

    def ask_user(self):
        shape = input("Enter shape to draw (heart / infinity / pentagon / stop): ")
        msg = String()
        msg.data = shape.lower()
        self.publisher_.publish(msg)
        self.get_logger().info(f'User chose: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ShapeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
