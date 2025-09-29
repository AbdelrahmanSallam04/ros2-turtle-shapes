import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
import math
import time

class TurtleCommander(Node):
    def __init__(self):
        super().__init__('turtle_commander')
        self.subscription = self.create_subscription(
            String,
            'chosen_shape',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        
        self.current_shape = None
        self.drawing = False
        self.shape_points = []
        self.current_point_index = 0
        self.timer = self.create_timer(0.2, self.timer_callback)  # Timer for drawing

    def listener_callback(self, msg):
        shape = msg.data
        self.get_logger().info(f'Received command: {shape}')
        
        if shape == 'stop':
            self.stop_turtle()
        elif shape in ['heart', 'infinity', 'pentagon'] and not self.drawing:
            self.current_shape = shape
            self.drawing = True
            self.current_point_index = 0
            self.generate_shape_points()
            self.get_logger().info(f'Starting to draw {shape}')

    def stop_turtle(self):
        """Stop the turtle immediately"""
        self.drawing = False
        self.shape_points = []
        # Stop any movement
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.publisher_.publish(vel_msg)
        self.get_logger().info("Turtle stopped!")

    def generate_shape_points(self):
        """Generate all points for the selected shape"""
        center_x, center_y = 5.5, 5.5
        self.shape_points = []
        
        if self.current_shape == 'heart':
            scale = 0.1
            num_points = 50
            for i in range(num_points + 1):
                t = 2 * math.pi * i / num_points
                x = 16 * math.sin(t)**3
                y = 13 * math.cos(t) - 5 * math.cos(2*t) - 2 * math.cos(3*t) - math.cos(4*t)
                self.shape_points.append((center_x + x * scale, center_y + y * scale))
                
        elif self.current_shape == 'infinity':
            scale = 1.5
            num_points = 50
            for i in range(num_points + 1):
                t = 2 * math.pi * i / num_points
                x = math.cos(t) / (1 + math.sin(t)**2)
                y = (math.cos(t) * math.sin(t)) / (1 + math.sin(t)**2)
                self.shape_points.append((center_x + x * scale, center_y + y * scale))
                
        elif self.current_shape == 'pentagon':
            radius = 2.0
            num_sides = 5
            for i in range(num_sides + 1):
                angle = 2 * math.pi * i / num_sides - math.pi/2
                x = center_x + radius * math.cos(angle)
                y = center_y + radius * math.sin(angle)
                self.shape_points.append((x, y))

    def timer_callback(self):
        """Timer callback to draw points one by one"""
        if not self.drawing or not self.shape_points:
            return
            
        if self.current_point_index < len(self.shape_points):
            # Get current point
            x, y = self.shape_points[self.current_point_index]
            
            # Calculate heading towards next point
            if self.current_point_index < len(self.shape_points) - 1:
                next_x, next_y = self.shape_points[self.current_point_index + 1]
                heading = math.atan2(next_y - y, next_x - x)
            else:
                heading = 0.0
            
            # Teleport to current point
            self.teleport_turtle(x, y, heading)
            self.current_point_index += 1
            
        else:
            # Finished drawing
            self.drawing = False
            self.get_logger().info(f'Finished drawing {self.current_shape}')

    def teleport_turtle(self, x, y, theta=0.0):
        """Teleport turtle to absolute position"""
        if not self.teleport_client.service_is_ready():
            self.get_logger().warn('Teleport service not ready')
            return
            
        request = TeleportAbsolute.Request()
        request.x = float(x)
        request.y = float(y)
        request.theta = float(theta)
        
        # Use call_async but don't wait for result
        self.teleport_client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()