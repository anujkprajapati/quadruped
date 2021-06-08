import pybullet as p
import numpy as np
import os

class Robot:
    def __init__(self,client):
        self.client = client
        f_name = os.path.join(os.path.dirname(__file__), 'custom_robot.urdf')
        self.robot = p.loadURDF(fileName=f_name, basePosition=[0,0,0.2], physicsClientId=client)
        self.jointArray = [i for i in range(14)]
        
    def get_ids(self):
        return self.client, self.robot
    
    def apply_action(self, action):
        #action = np.full(14,-0.5)
        p.setJointMotorControlArray(self.robot, self.jointArray, controlMode=p.POSITION_CONTROL,
            targetPositions=action, physicsClientId=self.client)
        
        

        
    def get_obs(self):
        status = p.getJointStates(self.robot, self.jointArray,physicsClientId=self.client)
        linear,_ = p.getBaseVelocity(self.robot, self.client)
        pos,_ = p.getBasePositionAndOrientation(self.robot, self.client)
        obs = [t[0] for t in status]
        
        return pos[2], linear[0], obs
