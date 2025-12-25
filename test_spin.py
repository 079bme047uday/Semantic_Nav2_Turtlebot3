#note this program is just for calibration of robot to rotate around 360 degrees in simulation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

#  FINAL CALIBRATION SETTINGS
TEST_SPEED = 2.0     
TEST_DURATION = 22.67  # Calculated to hit 360 degrees


class Calibrator(Node):
    def __init__(self):
        super().__init__('rotation_calibrator')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
    def run_calibration(self):
        print(f"Starting Final Calibration: Speed={TEST_SPEED}, Duration={TEST_DURATION}")
        
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.twist.angular.z = float(TEST_SPEED)
        
        start_time = time.time()
        end_time = start_time + TEST_DURATION
        
        while time.time() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
            time.sleep(0.02)
            
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        self.publisher.publish(stop_msg)
        print("Calibration complete. Check if the robot faces the starting direction.")

def main():
    rclpy.init()
    node = Calibrator()
    time.sleep(2.0)
    node.run_calibration()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()