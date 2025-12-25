import rclpy
import time
import cv2
import base64
from rclpy.node import Node
from rclpy.action import ActionClient
from cv_bridge import CvBridge

# ROS 2 Message Imports
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image

# AI and Agent Imports
from langchain.agents import tool
from langchain.schema import HumanMessage
from rosa import ROSA, RobotSystemPrompts
from llm_robot import azure_llm 

class RobotHardware(Node):
    def __init__(self):
        super().__init__('tb3_vision_agent')
        self.bridge = CvBridge()
        self.latest_frame = None
        
        # Publisher for TwistStamped (Required for Ignition/Gazebo Sim)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Subscription for camera feed
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self._image_cb, 
            10
        )
        
        # Action Client for Navigation
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_handle = None

    def _image_cb(self, msg):
        """Callback to store the latest camera frame."""
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def get_b64_image(self):
        """Processes the frame with CLAHE to improve simulation contrast and returns base64."""
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.01)
            
        if self.latest_frame is None:
            return None
            
        # Image Enhancement: Increase contrast to help AI distinguish blocky edges in Gazebo
        lab = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        cl = clahe.apply(l)
        limg = cv2.merge((cl,a,b))
        enhanced_img = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
        
        # Resize for optimal processing speed
        resized = cv2.resize(enhanced_img, (640, 480))
        _, buffer = cv2.imencode('.jpg', resized)
        return base64.b64encode(buffer).decode('utf-8')

    def cancel_nav_goal(self):
        """Cancels the current navigation goal to unlock the motors for rotation."""
        if self.goal_handle is not None:
            print("System: Cancelling active navigation goal to allow manual rotation.")
            self.goal_handle.cancel_goal_async()
            time.sleep(1.0) 

    def rotate_inplace(self, speed=2.0, duration=5.67):
        """
        Calibrated rotation for 90 degrees based on simulation physics.
        Speed: 2.0 rad/s
        Duration: 5.67s (derived from 22.67s for a full 360)
        """
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.twist.angular.z = float(speed)
        
        start_time = self.get_clock().now()
        end_time = start_time + rclpy.duration.Duration(seconds=duration)
        
        while self.get_clock().now() < end_time:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02) # 50Hz for stability
            
        # Hard stop
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        self.cmd_vel_pub.publish(stop_msg)

# Initialize ROS 2 and Global Robot Object
rclpy.init()
bot = RobotHardware()


# Navigation and Vision Tools


@tool
def navigate_to_room(room_name: str) -> str:
    """
    Navigates the robot to a specific room. 
    Available rooms: kitchen, living_room, bedroom, kitchen_store, study_room, dining_room.
    """
    coords = {
        "kitchen": (6.9, 1.38),
        "living_room": (-1.71, 1.9),
        "bedroom": (-4.57, -0.851),
        "kitchen_store": (2.91, 2.78),
        "study_room": (-4.44, 3.38),
        "dining_room": (7.68, -2.29)
    }
    
    name = room_name.lower().strip().replace(" ", "_")
    
    if name not in coords:
        valid_rooms = ", ".join(coords.keys())
        return f"Error: '{room_name}' not found. Please choose from: {valid_rooms}"
    
    x, y = coords[name]
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    
    bot.nav_client.wait_for_server()
    future = bot.nav_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(bot, future)
    bot.goal_handle = future.result()
    
    res_future = bot.goal_handle.get_result_async()
    rclpy.spin_until_future_complete(bot, res_future)
    
    return f"Success: Arrived at {room_name}. You must now call 'scan_room_360' to see the area."

@tool
def scan_room_360() -> str:
    """Rotates the robot and performs a context-aware analysis to identify simulated objects."""
    bot.cancel_nav_goal()
    
    images = []
    print("Action: Commencing 360-degree room scan using calibrated rotation.")
    
    for i in range(4):
        img = bot.get_b64_image()
        if img:
            images.append(img)
        
        print(f"Status: Captured view {i+1}/4. Rotating 90 degrees...")
        bot.rotate_inplace(speed=2.0, duration=5.67)
        time.sleep(1.0) # Wait for simulation physics to settle before next photo

    # Chain of Thought Vision Prompt
    system_vision_prompt = (
        "You are an AI reasoning engine for a TurtleBot3 in a Gazebo simulation. "
        "The objects you see are low-fidelity 3D models. To identify them accurately, follow this reasoning process: "
        "1. Identify the Room: Based on the walls and overall layout, what room is this? "
        "2. Analyze Geometry: Look for shapes. A tall brown cylinder might be a lamp; a flat brown rectangle might be a desk. "
        "3. Handle Confusion: If an object is ambiguous (e.g., looks like both a shelf and a desk), explain your reasoning based on its "
        "height and proximity to walls. "
        "4. Contextual Guessing: In a kitchen, a large white block is likely a fridge. In a bedroom, it is likely a wardrobe. "
        "Please describe the room and list all identified objects with your reasoning for each."
    )

    content = [
        {"type": "text", "text": system_vision_prompt}
    ]
    
    for img in images:
        content.append({
            "type": "image_url", 
            "image_url": {"url": f"data:image/jpeg;base64,{img}"}
        })
    
    print("System: AI is reasoning about the imagery...")
    return azure_llm.invoke([HumanMessage(content=content)]).content


# Agent Configuration and Main Loop


system_prompts = RobotSystemPrompts(
    embodiment_and_persona=(
        "You are an autonomous TurtleBot3 service robot. "
        "When asked to find an object, first navigate to the most likely room, "
        "then perform a 'scan_room_360' to analyze the surroundings. "
        "If you are confused between objects, explain why to the user."
    )
)

agent = ROSA(
    ros_version=2, 
    llm=azure_llm, 
    tools=[navigate_to_room, scan_room_360], 
    prompts=system_prompts
)

def main():
    print("System Online: Calibrated Rotation and Geometric Reasoning Active.")
    try:
        while rclpy.ok():
            user_input = input("\nUser Input > ")
            if user_input.lower() in ['exit', 'quit']:
                break
            
            print("Robot: Processing...")
            response = agent.invoke(user_input)
            print(f"\nRobot Result: {response}")
            
    except KeyboardInterrupt:
        pass
    finally:
        bot.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()