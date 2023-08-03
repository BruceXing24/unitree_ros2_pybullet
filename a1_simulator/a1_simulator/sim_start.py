import pybullet as p
import pybullet_data as pd
from pybullet_utils import bullet_client
import numpy as np
import time
from rclpy.node import Node
import rclpy
from .pb_motors import Motors
from geometry_msgs.msg import Vector3
from a1_msg.msg import JointAngles

np.set_printoptions(suppress=True)

class simulator(Node):
    def __init__(self,name):
        super().__init__(name)
        #pybullet setting 
        self.pybullet_client = bullet_client.BulletClient(connection_mode = p.GUI)
        self.robot_id = self.pybullet_client.loadURDF("src/a1_simulator/urdf/a1.urdf",
                                            [0, 0.0, 0.5],
                                            baseOrientation = self.pybullet_client.getQuaternionFromEuler([0, 0, 0]),
                                        #    flags=p.URDF_USE_INERTIA_FROM_FILE,
                                        #    useFixedBase = 1
                                            )
        dt = 1.0 / 240.0  # Simulation time step
        self.pybullet_client.setAdditionalSearchPath(pd.getDataPath())
        self.planeID = self.pybullet_client.loadURDF("plane.urdf")
        self.pybullet_client.setGravity(0, 0, -9.8)

        self.init_counter = 0
        ## pybullet debug line
        self.pybullet_client.setDebugObjectColor(self.robot_id, -1, objectDebugColorRGB=[1, 0, 0])



        # ros2 settinhg
        self.create_timer(dt,self.timer_callback)
        self.motors = Motors(self.pybullet_client,self.robot_id)
        self.motors.positions_control( [0,39.7, -75, 0,39.7, -75, 0,39.7, -75, 0,39.7, -75,])
        self.joints = np.array([0,39.7, -75, 0,39.7, -75, 0,39.7, -75, 0,39.7, -75])
        self.subscription = self.create_subscription(JointAngles,'/robot_joints',self.joint_callback,10)

        self.pybullet_client.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
        self.pybullet_client.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    def joint_callback(self,msg:JointAngles):
        self.joints = np.array(msg.joint_angles)
        self.joints = self.joints*180/np.pi

    

    def timer_callback(self):
        if self.init_counter<200:
            self.init_counter+=1
            self.motors.positions_control( [0,39.7, -75, 0,39.7, -75, 0,39.7, -75, 0,39.7, -75,])

        else:
            self.motors.positions_control(self.joints)


            contact_points = self.pybullet_client.getContactPoints(self.robot_id,self.planeID)
            # print(len(contact_points))
            for contact in contact_points:
                scaled_force  = [0,0,0]
                contact_force = contact[9]
                contact_direction = contact[7]
                # print(contact_direction)
                contact_position = contact[6]
                # print(contact_direction)
                # Scale the contact force for visualization
                
                for i in range(3):
                    scaled_force[i] = 0.005 * contact_force*contact_direction[i]

                # Calculate the endpoint of the force vector
                force_endpoint = [
                    contact_position[i] + scaled_force[i] for i in range(3)
                ]
                # Add a debug line to represent the contact force
                self.pybullet_client.addUserDebugLine(contact_position,
                                force_endpoint,
                                lineColorRGB=[1, 0, 0], lineWidth=3, lifeTime=0.1)            

        self.pybullet_client.stepSimulation()

            # todo:debug line 



def main(args = None):
    rclpy.init()
    sim = simulator('sim')
    rclpy.spin(sim)
    sim.destroy_node()   
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
