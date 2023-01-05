import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# from rclpy.clock import clock

from threading import Thread

from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from sciclops_driver.sciclops_driver import SCICLOPS

class SciclopsDescriptionClient(Node):

    def __init__(self, NODE_NAME = 'SciclopsDescriptionNode'):
        super().__init__(NODE_NAME)

        self.sciclops = SCICLOPS()

        timer_period = 0.1  # seconds
        
        self.state = "UNKNOWN"
        joint_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()

        self.statePub = self.create_publisher(String, NODE_NAME + '/state',10)
        # self.stateTimer = self.create_timer(timer_period, callback = self.stateCallback, callback_group = state_cb_group)

        self.joint_publisher = self.create_publisher(JointState,'joint_states', 10, callback_group = joint_cb_group)
        self.joint_state_handler = self.create_timer(timer_period, callback = self.joint_state_publisher_callback, callback_group = joint_cb_group)
    
    
    def stateCallback(self):
        '''
        Publishes the sciclops_description state to the 'state' topic. 
        '''
        msg = String()
        msg.data = 'State: %s' % self.state
        self.statePub.publish(msg)
        self.get_logger().info('Publishing State: "%s"' % msg.data)
        self.state = "READY"


    def joint_state_publisher_callback(self):
        
        # self.get_logger().info("BUGG")

        #TODO: Create a function in the driver level to complete the lines between #53 - #61
        #TODO: Make sure the angles are in radius format 
        #TODO: Call only one function from the driver that would return all joint angles in radius format
        # self.sciclops.get_position()
        # self.sciclops.get_gripper_lenght()
        # robot_joint_states = self.sciclops.CURRENT_POS
        # robot_gripper_state = self.sciclops.GRIPLENGTH
        # joint_states = robot_joint_states
        # joint_states.append(robot_gripper_state/2)
        # joint_states.append(-robot_gripper_state/2)

        joint_states = [-0.4,0.0,0.0,0.0,0.0,0.0]
        sciclops_joint_msg = JointState()
        sciclops_joint_msg.header = Header()
        sciclops_joint_msg.header.stamp = self.get_clock().now().to_msg()
        sciclops_joint_msg.name = ['Sciclops_joint1','Sciclops_joint2','Sciclops_joint3','Sciclops_joint4','Sciclops_gripper','Sciclops_gripper_mirror']
        sciclops_joint_msg.position = joint_states
        # print(joint_states)

        # sciclops_joint_msg.position = [0.01, -1.34, 1.86, -3.03, 0.05, 0.05, 0.91]
        sciclops_joint_msg.velocity = []
        sciclops_joint_msg.effort = []

        self.joint_publisher.publish(sciclops_joint_msg)
        self.get_logger().info('Publishing joint states: "%s"' % joint_states)


def main(args=None):
    rclpy.init(args=args)
    try:
        sciclops_joint_state_publisher = SciclopsDescriptionClient()
        executor = MultiThreadedExecutor()
        executor.add_node(sciclops_joint_state_publisher)

        try:
            sciclops_joint_state_publisher.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            sciclops_joint_state_publisher.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            sciclops_joint_state_publisher.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()