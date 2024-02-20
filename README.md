## **CONTROLLING OF 4-DOF ROBOT USING MICROROS**

This repository focuses on controlling a 4 DOF (Degree Of Freedom) self-made robotic arm using the microROS framework of ROS2, visualization performed through RViz and motion planning performed using the MoveIt framework to carry out pick and place tasks in an industry 4.0 benchmark bottle-filling plant.

### **HARDWARE DEPENDENCIES:**
Kindly check the PDF for details on Robot Hardware.

### **SOFTWARE DEPENDENCIES:**

- **OPERATING SYSTEM**- Ubuntu 22.04
- **ROS DISTRO**- ROS2 Iron
- **ARDUINO IDE VERSION**- Arduino IDE 2.1.1

### **QUICK START GUIDE:**

1. Follow the link [micro_ROS_Setup](https://micro.ros.org/docs/tutorials/core/overview/) to install and configure the micro-ROS libraries.
2. Clone this repository on the src folder of your working directory.
3. Build your workspace using `colcon build`.
4. Source your workspace.
5. First, launch the RViz platform using the command `ros2 launch asrs_description  rviz.launch.py`.
6. Then, launch the `joint_state_processor` package using the command `ros2 run joint_state_processor processor`.
7. Open the IDE file.
8. Launch the microros agent `ros2 run micro_ros_agent micro_micro_ros_agent serial --dev /dev/ttyACM0`.
9. Compile and Upload the code.
10. Use the GUI from RViz to simulate the Robot. Same motion will be replicated in the robot.
