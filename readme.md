# Semantic Navigation for TurtleBot3 with LLM Integration

## Project Overview
This project creates an autonomous robotic agent that goes beyond navigation to understand its environment within the ROS 2 and Gazebo ecosystem. By combining geometric reasoning, contextual awareness, and LLM-based vision, the robot interprets objects based on their shape, location, and relationships rather than relying on fragile visual features. Natural language commands guide purposeful exploration, while panoramic perception and semantic reasoning allow the robot to recognize what objects are and why they belong in a space, demonstrating true environmental understanding, not just movement through coordinates.

## Features
- Natural language command interpretation for TurtleBot3
- Integration with Azure OpenAI (GPT-4o) via environment variables
- ROS 2-based navigation and camera streaming
- Example scripts for robot navigation and calibration

## Prerequisites
- **Operating System:** Linux (recommended: Ubuntu 24.04)
- **ROS 2:** Jazzy Distribution
- **TurtleBot3 Packages:**
  - `turtlebot3`, `turtlebot3_msgs`, `turtlebot3_navigation2`, etc.
- **Python 3.8+**
- **Azure OpenAI account** (for LLM integration)

## Environment Setup
1. **Clone this repository:**
   ```bash
   git clone https://github.com/079bme047uday/Semantic_Nav2_Turtlebot3
   cd Semantic_Nav2_Turtlebot3
   ```
2. **Create a Python virtual environment:**
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```
3. **Set up environment variables:**
   - Create a `.env` file in the project root with the following keys:
     ```env
     AZURE_DEPLOYMENT_NAME=your-azure-deployment-name
     AZURE_OPENAI_API_KEY=your-azure-openai-api-key
     AZURE_OPENAI_ENDPOINT=your-azure-endpoint-url
     ```
   - These are required for LLM access (see `llm_robot.py`).

4. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

5. **Install TurtleBot3 and ROS 2 packages:**
   - Follow the [official TurtleBot3 ROS 2 installation guide](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) for your ROS 2 distro.
   - Example (for Ubuntu/ROS 2 Jazzy):
     ```bash
     sudo apt update
     sudo apt install ros-jazzy-turtlebot3* ros-jazzy-nav2-bringup
     ```
   - Set TurtleBot3 model:
     ```bash
     echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
     source ~/.bashrc
     ```

## Usage
- **Run navigation or calibration scripts:**
  - For semantic navigation: `python llm_robot.py`
  - For robot calibration: `python test_spin.py`
  - For robot navigation test: `python test_robot.py`

## Running the actual program
- **Run navigation and turtlebot3_launch.py file:**
  - For turtlebot:
    ```bash
     ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
     ```
  - For visualization in rviz (the maps/my_map.yaml is the saved map with directory, it can be different in your case, depends on where you save your map): 
    ```bash
     ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/my_map.yaml
     ```
  - For python_script integrating LLM (ensure you are inside your created environment): 
    ```bash
     python test_robot.py
     ```


## Notes
- Ensure your ROS 2 environment is sourced before running scripts:
  ```bash
  source /opt/ros/jazzy/setup.bash
  ```
- The `.env` file must be present for LLM features to work.
- This project is designed for TurtleBot3 in both simulation and real hardware.

## References
- [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Azure OpenAI Service](https://learn.microsoft.com/en-us/azure/cognitive-services/openai/)
- [Nasa-JPL-ROSA-Framework](https://github.com/nasa-jpl/rosa)
---

For questions or contributions, please open an issue or pull request.
