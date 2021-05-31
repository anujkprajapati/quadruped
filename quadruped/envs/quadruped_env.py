import gym
import numpy as np
import pybullet as p
from quadruped.resources.robot import Robot
from quadruped.resources.plane import Plane


class QuadrupedEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            low=np.full(14, -0.5),
            high=np.full(14, 0.5))
        self.observation_space = gym.spaces.box.Box(
            low=np.full(14,-0.5),
            high=np.full(14, 0.5)
            )
        self.np_random, _ = gym.utils.seeding.np_random()
        #high=np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]))
        # self.client = p.connect(p.GUI)
        self.client =p.connect(p.DIRECT)


        #self.robot = None
        self.done = False
        self.reset()

    def step(self, action):

        # feed action, note observation
        self.robot.apply_action(action)
        p.stepSimulation()
        #print(action)
        posz, velx, status = self.robot.get_obs()

        # compute reward
        rvel = 10*velx
        rtime = 10 if posz >= 0.1 else -100 # length of time
        reward = rvel+rtime
        # check if done
        if posz <= 0.1:
            self.done = True
        return status, reward, self.done, dict()
        

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0,0,-10)
        Plane(self.client)
        self.robot = Robot(self.client)
        
        self.done = False
        return np.full(14,0)

    def render(self):
        pass

    def close(self):
        p.disconnect(self.client)
        
        
        
    def seed(self, seed=None): 
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]