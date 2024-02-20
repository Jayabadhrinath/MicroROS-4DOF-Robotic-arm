import rclpy
import numpy as np
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JointStateProcessorNode:

    def __init__(self):
        self.node = rclpy.create_node('joint_state_processor_node')
        
        # Creating a subscriber to receive joint_state from RViz
        self.joint_state_subscriber = self.node.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10) 
        
        # Declaration of variables
        self.name=[]
        self.position=np.zeros(10,dtype=float)
        self.velocity=[]
        self.effort=[] 
        self.prev_position=np.zeros(10,dtype=float)
        self.move_to_position=np.zeros(10,dtype=float)           

        # Creating a publisher to send processed_joint_state information to Arduino
        self.processed_joint_state_publisher = self.node.create_publisher(
            Float64MultiArray, 'processed_joint_states', 10)

    def joint_state_callback(self, joint_state_msg):

        # Splitting the information from joint_state topic. We only need position information for this application
        self.name=joint_state_msg.name
        self.position=joint_state_msg.position
        self.velocity=joint_state_msg.velocity
        self.effort=joint_state_msg.effort

        # Call to function to get end goal
        self.move_to_position=self.process_joint_state(self.position)

        # Printing information
        self.node.get_logger().info(f"Received joint state info: {self.position}")
        self.node.get_logger().info(f"Goal info: {self.move_to_position}")
        self.node.get_logger().info(f"Previous joint info: {self.prev_position}")

        # Typecasting to publish as a form of std_msgs.msg/Float64MultiArray
        end_goal=Float64MultiArray()
        end_goal.data=self.move_to_position.flatten().tolist()        
               
        # Publish the processed joint state information
        self.processed_joint_state_publisher.publish(end_goal)

        # Sleep to control the data transfer rate. Information is received from RViz and published every 2 seconds.
        time.sleep(10)

    def process_joint_state(self, position):

        # Function to compute the endgoal. numpy used for ease of array operation        
        goal=np.subtract(self.position,self.prev_position,dtype=float)

        #After computing, current position is stored as previous position (open loop control)
        self.prev_position=self.position

        return goal

    def spin(self):
        rclpy.spin(self.node)

    def destroy(self):
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    joint_state_processor_node = JointStateProcessorNode()
    joint_state_processor_node.spin()
    joint_state_processor_node.destroy()

if __name__ == '__main__':
    main()
