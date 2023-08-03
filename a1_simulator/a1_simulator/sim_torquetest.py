import pybullet as p
import pybullet_data as pd
from pybullet_utils import bullet_client
import numpy as np
import time
from rclpy.node import Node
import rclpy
from pb_motors import Motors
from geometry_msgs.msg import Vector3
from a1_msg.msg import JointAngles
from a1_msg.msg import RobotStates
from a1_msg.msg import JointState
from a1_msg.msg import ContactDetection

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
                                           useFixedBase = 1
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

        self.subscription = self.create_subscription(JointState,'/cmd_joints',self.cmd_callback,10)

        self.robot_state = RobotStates()
        self.contact_flag = ContactDetection()
        self.joints_state = JointState()
        self.estimator = self.create_publisher(RobotStates,"/estimator/robot_state",5)
        self.jointState_pub = self.create_publisher(JointState,"/joint_state",10)
        self.contact_pub = self.create_publisher(ContactDetection, "/estimator/contacts",5)
        
        self.joint_torque = np.array([0.]*12)
        self.current_angle = np.array([0.]*12)
        self.current_vel = np.array([0.]*12)
        self.angle_cmd = np.array([0.]*12)

        self.target_angles = np.array([0,39.7, -75, 0,39.7, -75, 0,39.7, -75, 0,39.7, -75,])*np.pi/180

        self.init_angles = np.array([0.16034899 , 2.6450247  ,-2.69653237, -0.16104047, 2.64523499, -2.69577746,
                            0.16034487 , 2.6450228  ,-2.69653255, -0.16098723, 2.64521532, -2.69562313])


        self.duration = 500.
        self.percent = 0.

    def joint_callback(self,msg:JointAngles):
        self.joints = np.array(msg.joint_angles)
        self.joints = self.joints*180/np.pi


    def cmd_callback(self,msg:JointState):
        for i in range(12):
            self.joint_torque[i] = msg.joints_torque[i]

    def timer_callback(self):
        if self.init_counter<100:
            self.motors.positions_control( [0,39.7, -75, 0,39.7, -75, 0,39.7, -75, 0,39.7, -75,])
            # self.motors.positions_control( [0,0, 0, 
            #                                 0,0,0, 
            #                                 0,0, 0,
            #                                 0,0, 0,])
        elif self.init_counter<101:
            self.motors.disable_PandV()

        elif self.init_counter<2500:
            all_joint_states = self.pybullet_client.getJointStates(self.robot_id,[2,4,5,7,9,10,12,14,15,17,19,20])
            for i in range(12):
                self.current_angle[i] = all_joint_states[i][0]
                self.current_vel[i] = all_joint_states[i][1]
            print(f"current_angle=={self.current_angle}")


            self.percent += 1.0/self.duration
            percent_front = 2 *self.percent
            self.percent = 1 if self.percent>1 else self.percent
            percent_front = 1 if percent_front>1 else percent_front

            self.angle_cmd[0:6] = (1-percent_front)*self.init_angles[0:6] + percent_front * self.target_angles[0:6]
            self.angle_cmd[6:12] = (1-self.percent)*self.init_angles[6:12] + self.percent * self.target_angles[6:12]
            torques = self.motors.pose_control_2torque(self.angle_cmd,self.current_angle,self.current_vel)
            self.motors.torque_control(torques)
            print(f"torques=={torques}")


        else:
            pos_s = self.pybullet_client.getBasePositionAndOrientation(self.robot_id)[0]
            vel_s = self.pybullet_client.getBaseVelocity(self.robot_id)[0]
            wv_s = self.pybullet_client.getBaseVelocity(self.robot_id)[1]
            oriQ_s = self.pybullet_client.getBasePositionAndOrientation(self.robot_id)[1]
            oirE_s = self.pybullet_client.getEulerFromQuaternion(oriQ_s)

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







            self.jointState_pub.publish(self.joints_state)
            self.robot_state.robot_pos_s = pos_s
            self.robot_state.robot_vel_s = vel_s
            self.robot_state.robot_ori_s = oirE_s
            self.robot_state.robot_w_s  = wv_s
            self.estimator.publish(self.robot_state)





# joints0===-0.3524
# joints1===-3.2322
# joints2===3.83337
# joints3===0.07027
# joints4===-1.5584
# joints5===4.41508
# joints6===-4.5740
# joints7===2.13944
# joints8===6.04558
# joints9===4.67835
# joints10===1.7466
# joints11===6.3196      

# _tau=== 3.063 ,-0.140, 4.368
#         -3.126, -0.144, 4.318
#          2.832 ,-0.055 ,4.111
#         -2.892 ,-0.060 ,4.061


        self.pybullet_client.stepSimulation()
        self.init_counter+=1



    def contact_detection(self):
        pass


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


