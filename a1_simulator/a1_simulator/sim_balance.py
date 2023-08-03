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
from a1_msg.msg import RobotStates
from a1_msg.msg import JointState
from a1_msg.msg import ContactDetection

class simulator(Node):
    def __init__(self,name):
        super().__init__(name)
        #pybullet setting 
        self.pybullet_client = bullet_client.BulletClient(connection_mode = p.GUI)
        self.robot_id = self.pybullet_client.loadURDF("src/a1_simulator/urdf/a1.urdf",
                                            [0, 0.0, 0.5],
                                            baseOrientation = self.pybullet_client.getQuaternionFromEuler([0, 0, 0]),
                                           flags=p.URDF_USE_INERTIA_FROM_FILE,
                                        #    useFixedBase = 1
                                            )
        dt = 1.0 / 240.0  # Simulation time step
        self.pybullet_client.setAdditionalSearchPath(pd.getDataPath())
        self.planeID = self.pybullet_client.loadURDF("plane.urdf")
        self.pybullet_client.setGravity(0, 0, -9.8)
        self.init_counter = 0

        ## pybullet debug line
        self.pybullet_client.setDebugObjectColor(self.robot_id, -1, objectDebugColorRGB=[1, 0, 0])
        self.pybullet_client.changeDynamics(self.planeID, linkIndex=-1, lateralFriction=1.0)


        # ros2 settinhg
        self.create_timer(dt,self.timer_callback)
        self.motors = Motors(self.pybullet_client,self.robot_id)
        self.motors.positions_control( [0,39.7, -75, 0,39.7, -75, 0,39.7, -75, 0,39.7, -75,])
        self.joints = np.array([0,39.7, -75, 0,39.7, -75, 0,39.7, -75, 0,39.7, -75])
        self.subscription = self.create_subscription(JointState,'/cmd_joints',self.cmd_callback,10)

        self.robot_state = RobotStates()
        self.contact_flag = ContactDetection()
        self.joints_state = JointState()
        self.estimator = self.create_publisher(RobotStates,"/estimator/robot_state",5)
        self.jointState_pub = self.create_publisher(JointState,"/joint_state",10)
        self.contact_pub = self.create_publisher(ContactDetection, "/estimator/contacts",5)
        
        self.cmd_torque = np.array([0]*12)


        self.joint_torque = np.array([0.]*12)
        self.current_angle = np.array([0.]*12)
        self.current_vel = np.array([0.]*12)
        self.angle_cmd = np.array([0.]*12)
        
        self.target_angles = np.array([0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 
                            0.0, 0.67, -1.3, 0.0, 0.67, -1.3])
        self.init_angles = np.array([0.16034899 , 2.6450247  ,-2.69653237, -0.16104047, 2.64523499, -2.69577746,
                            0.16034487 , 2.6450228  ,-2.69653255, -0.16098723, 2.64521532, -2.69562313])
        self.duration = 1000.
        self.percent = 0.


    def cmd_callback(self,msg:JointState):
        for i in range(12):
            self.cmd_torque[i] = msg.joints_torque[i]


    def timer_callback(self):

        all_joint_states = self.pybullet_client.getJointStates(self.robot_id,[2,4,5,7,9,10,12,14,15,17,19,20])
        for i in range(12):
            self.joints_state.joints_angle[i] = all_joint_states[i][0]
            # print(f"joints{i}==={all_joint_states[i][3]}")
        self.jointState_pub.publish(self.joints_state)
        self.pub_contact()
        self.estimator_pub()

        if self.init_counter<1000:
            self.motors.positions_control( [0,39.7, -75, 0,39.7, -75, 0,39.7, -75, 0,39.7, -75,])
        else:
            if self.init_counter==1000:
                print("torque control activated")
                self.motors.disable_PandV()


            self.motors.torque_control(self.cmd_torque)            
            if self.init_counter % 100 ==0:
                print("torque control working")
            # time.sleep(0.01)

        self.pybullet_client.stepSimulation()
        self.init_counter+=1
        # self.debug_line(0.01)

     






    def debug_line(self,sleep_time):
        contact_points = self.pybullet_client.getContactPoints(self.robot_id,self.planeID)
        # print(len(contact_points))
        for contact in contact_points:
            scaled_force_1  = [0,0,0]
            scaled_force_2 = [0,0,0]
            scaled_force_3 = [0,0,0]
            force_endpoint_4 = np.array([0]*3)
            contact_force_1 = contact[9]
            contact_force_2 = contact[10]
            contact_force_3 = contact[12]
            contact_direction_1 = contact[7]
            contact_direction_2 = contact[11]
            contact_direction_3 = contact[13]
            contact_position = contact[6]
            # Scale the contact force for visualization
            
            for i in range(3):
                scaled_force_1[i] = 0.005 * contact_force_1*contact_direction_1[i]
                scaled_force_2[i] = 0.005 * contact_force_2*contact_direction_2[i]
                scaled_force_3[i] = 0.005 * contact_force_3*contact_direction_3[i]

            # Calculate the endpoint of the force vector
            force_endpoint_1 = [
                contact_position[i] + scaled_force_1[i] for i in range(3)
            ]
            force_endpoint_2 = [
                contact_position[i] + scaled_force_2[i] for i in range(3)
            ]
            force_endpoint_3 = [
                contact_position[i] + scaled_force_3[i] for i in range(3)
            ]
            force_endpoint_4 = [force_endpoint_3[0],force_endpoint_2[1],force_endpoint_1[2]]
            self.pybullet_client.addUserDebugLine(contact_position,
                            force_endpoint_4,
                            lineColorRGB=[0, 1, 0], lineWidth=3, lifeTime=sleep_time)                





    def get_whole_mass(self):
        sum_mass = 0
        for i in range(22):
            info = self.pybullet_client.getDynamicsInfo(self.robot_id,i)
            print(f"{i}_mass =={info[0]}")
            sum_mass+= info[0]
            print(f"sum_mass =={sum_mass}")

    def pub_contact(self):
        contact_points = self.pybullet_client.getContactPoints(self.robot_id,self.planeID)
        self.contact_flag.contacts=[False,False,False,False]
        for contact in contact_points:
            if contact[3]==6:
                self.contact_flag.contacts[0]=True
            if contact[3]==11:
                self.contact_flag.contacts[1]=True
            if contact[3]==16:
                self.contact_flag.contacts[2]=True
            if contact[3]==21:
                self.contact_flag.contacts[3]=True
        self.contact_pub.publish(self.contact_flag)

    def estimator_pub(self):
        pos_s = self.pybullet_client.getBasePositionAndOrientation(self.robot_id)[0]
        vel_s = self.pybullet_client.getBaseVelocity(self.robot_id)[0]
        wv_s = self.pybullet_client.getBaseVelocity(self.robot_id)[1]
        oriQ_s = self.pybullet_client.getBasePositionAndOrientation(self.robot_id)[1]
        oirE_s = self.pybullet_client.getEulerFromQuaternion(oriQ_s)
        oirE_s = np.array(oirE_s,dtype=np.float32)
        oirE_s[0]= -  oirE_s[0]
        self.robot_state.robot_pos_s = pos_s
        self.robot_state.robot_vel_s = vel_s
        self.robot_state.robot_ori_s = oirE_s
        self.robot_state.robot_w_s  = wv_s
        self.estimator.publish(self.robot_state)

    def joint_state_pub(self):
        pass


# TODO: 在开始力控前更新一下初始位置，就不用每次手动调正了


def main(args = None):
    rclpy.init()
    sim = simulator('sim')
    rclpy.spin(sim)
    sim.destroy_node()   
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()





# contact link id
# 6 11 16 21
# FR FL BR BL


