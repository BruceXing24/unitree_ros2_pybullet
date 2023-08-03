import numpy as np
import time
from rclpy.node import Node
import rclpy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3

class simulator(Node):
    def __init__(self,name):
        super().__init__(name)
        self.joints = np.array([0]*12)
        self.subscription = self.create_subscription(Vector3,'/robot_joints',self.joint_callback,10)

    def joint_callback(self,msg:Vector3):
        self.joints[0] = msg.x
        self.joints[1] = msg.y
        print("callback")
        print(self.joints[0])


def main(args = None):
    rclpy.init()
    sim = simulator('sim')
    rclpy.spin(sim)
    sim.destroy_node()   
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
