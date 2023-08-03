import pybullet as p
import numpy as np


NUM_MOTORS = 12


class Motors:
    def __init__(self,pybullet_client,robot_id):
        self.pybullet_client=pybullet_client
        self.robot_id = robot_id
        #                    RF      LF     RB    LB
        self.Position_Gain = 2
        self.Velocity_Gain = .5
        self.Max_velocity = 10
        self.force  = 100

        self._kp = 60
        self._kd = 1
        self.torque_limitiation = 33.0

    def positions_control(self, joint_angles):
        joint_angles = np.array(joint_angles)
        joint_angles = joint_angles *np.pi/180 
        for i in range(4):
            self.pybullet_client.setJointMotorControl2(bodyIndex=self.robot_id,
                                    jointIndex=5*i+2,
                                    controlMode=self.pybullet_client.POSITION_CONTROL,
                                    targetPosition=joint_angles[3*i],
                                    positionGain=self.Position_Gain,
                                    velocityGain=self.Velocity_Gain,
                                    maxVelocity=self.Max_velocity
                                    )
            
            self.pybullet_client.setJointMotorControl2(bodyIndex=self.robot_id,
                                    jointIndex=5*i+4,
                                    controlMode=self.pybullet_client.POSITION_CONTROL,
                                    targetPosition=joint_angles[3*i+1],
                                    positionGain=self.Position_Gain,
                                    velocityGain=self.Velocity_Gain,
                                    maxVelocity=self.Max_velocity
                                    )      
                 
            self.pybullet_client.setJointMotorControl2(bodyIndex=self.robot_id,
                                    jointIndex=5*i+5,
                                    controlMode=self.pybullet_client.POSITION_CONTROL,
                                    targetPosition=joint_angles[3*i+2],
                                    maxVelocity=self.Max_velocity
                                    )         
        # self.pybullet_client.setJointMotorControl2(self.robot_id ,5, self.pybullet_client.POSITION_CONTROL, targetPosition=joint_angles[2])
        # self.pybullet_client.setJointMotorControl2(self.robot_id ,10, self.pybullet_client.POSITION_CONTROL, targetPosition=joint_angles[5])
        # self.pybullet_client.setJointMotorControl2(self.robot_id ,15, self.pybullet_client.POSITION_CONTROL, targetPosition=joint_angles[8])
        # self.pybullet_client.setJointMotorControl2(self.robot_id ,20, self.pybullet_client.POSITION_CONTROL, targetPosition=joint_angles[11])    
    
    def torque_control(self, joint_torques):
        for i in range(4):
            self.pybullet_client.setJointMotorControl2(bodyIndex=self.robot_id,
                                    jointIndex=5*i+2,
                                    controlMode=self.pybullet_client.TORQUE_CONTROL,
                                    force=joint_torques[3*i],
                                    )
            
            self.pybullet_client.setJointMotorControl2(bodyIndex=self.robot_id,
                                    jointIndex=5*i+4,
                                    controlMode=self.pybullet_client.TORQUE_CONTROL,
                                    force=joint_torques[3*i+1],
                  
                                    )      
                 
            self.pybullet_client.setJointMotorControl2(bodyIndex=self.robot_id,
                                    jointIndex=5*i+5,
                                    controlMode=self.pybullet_client.TORQUE_CONTROL,
                                    force=joint_torques[3*i+2],
                                    )        
            
    def disable_PandV(self):
        for i in range(4):
            self.pybullet_client.setJointMotorControl2(bodyIndex=self.robot_id,
                                    jointIndex=5*i+2,
                                    controlMode=self.pybullet_client.POSITION_CONTROL,
                                    positionGain=self.Position_Gain,
                                    velocityGain=self.Velocity_Gain,
                                    maxVelocity=self.Max_velocity,
                                    force =0
                                    )
            
            self.pybullet_client.setJointMotorControl2(bodyIndex=self.robot_id,
                                    jointIndex=5*i+4,
                                    controlMode=self.pybullet_client.POSITION_CONTROL,
                                    positionGain=self.Position_Gain,
                                    velocityGain=self.Velocity_Gain,
                                    maxVelocity=self.Max_velocity,
                                    force =0
                                    )      
                 
            self.pybullet_client.setJointMotorControl2(bodyIndex=self.robot_id,
                                    jointIndex=5*i+5,
                                    controlMode=self.pybullet_client.POSITION_CONTROL,
                                    maxVelocity=self.Max_velocity,
                                    force =0
                                    )         
            
    def pose_control_2torque(self,motor_angle_cmd, motor_angle,motor_velocity):
        kp = self._kp
        kd = self._kd

        desired_motor_angles = motor_angle_cmd
        desired_motor_velocities = np.full(NUM_MOTORS, 0)
        additional_torques = np.full(NUM_MOTORS, 0.5)
        motor_torques = -1 * (kp * (motor_angle - desired_motor_angles)) - kd * (
        motor_velocity - desired_motor_velocities) + additional_torques

        motor_torques = np.clip(a=motor_torques,a_min=-self.torque_limitiation,a_max=self.torque_limitiation)

        return motor_torques
    

    def pose2torque_1(self,motor_angle_cmd, motor_angle,motor_velocity,additional_torques):
        kp = self._kp
        kd = self._kd

        desired_motor_angles = motor_angle_cmd
        desired_motor_velocities = np.full(NUM_MOTORS, 0)
        # additional_torques = np.full(NUM_MOTORS, 0.5)
        motor_torques = -1 * (kp * (motor_angle - desired_motor_angles)) - kd * (
        motor_velocity - desired_motor_velocities) + additional_torques

        motor_torques = np.clip(a=motor_torques,a_min=-self.torque_limitiation,a_max=self.torque_limitiation)

        return motor_torques
    

    def pose2torque_2(self,desired_q, curr_q,curr_dq,desired_dq,additional_torques):
        kp = self._kp
        kd = self._kd

        # additional_torques = np.full(NUM_MOTORS, 0.5)
        motor_torques = -1 * (kp * (curr_q - desired_q)) - kd * (
        curr_dq - desired_dq) + additional_torques

        motor_torques = np.clip(a=motor_torques,a_min=-self.torque_limitiation,a_max=self.torque_limitiation)

        return motor_torques