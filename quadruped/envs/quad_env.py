import gym
import numpy as np
import pybullet as p
from quad.resources.robot import Robot
from quad.resources.plane import Plane
from collections import deque
# from quad.resources.trajectory import Path



class QuadEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            # low=np.full(12,-0.5),
            # action space for each leg: stride_length, phi, height
            # fl =1, fr =2, bl = 3, br = 4
            low = np.array([-0.03,-1, 0]*4),
            high= np.array([0.03, 1, 0.05]*4))
        self.observation_space = gym.spaces.box.Box(
            low = np.array([-0.2,-0.5,-0.5,-0.2,-0.5,-0.5,-0.2,-0.5,-0.5,-0.2,-0.5,-0.5]),
            high=np.full(12, 0.5)
            )
        self.np_random, _ = gym.utils.seeding.np_random()
        self.client = p.connect(p.GUI)
        #self.client =p.connect(p.DIRECT)
        

        self.done = False
        self.reset()
        # self.robot = Robot(self.client)
        self.last_dist = 0
        self.leap_list = deque([0]*100, maxlen=100)

        
        print("this is the version 3:")



    def step(self, action, n_step):
        

        #swing for left(1,3) or right(2,4)
        count = 200
        # left = (n_step/count)%2
        theta = n_step%count  #theta cycles from 0 to 200

        # feed action, note observation
        self.robot.apply_action(action, theta)
        
        status, linear = self.robot.get_obs()
        velx = linear[0]


        pos,ori,h1,h2,h3,h4 = self.robot.get_location()
        posx = pos[0]
        posy = pos[1]
        posz = pos[2]
        
    
        # centre of gravity: area b/w h1, h4, cg_footprint
        if theta>=100:
            area = posx*(h1[1]-h4[1]) + h1[0]*(h4[1]-posy) + h4[0]*(posy-h1[1])
        else:
            area = posx*(h2[1]-h3[1]) + h2[0]*(h3[1]-posy) + h3[0]*(posy-h2[1])

        area_penalty = np.exp(-1000*np.round(area**2,4))


        reward_ori = np.exp(-1000*ori[0]**2 - 300*ori[1]**2 - 50*ori[2]**2)

        #step_distance as used in stoch
        lastx = self.last_dist
        leap = posx - lastx
        self.last_dist = posx
        self.leap_list.append(leap)

        penalty = 0
        if sum(self.leap_list) < 0.001 and n_step>200:
            penalty = -4
        
        # compute reward
        
        rvel = velx
        rstep = 1-1*np.exp(-n_step/3000)   # length of time
        # rheight = 0 if posz <= 0.15 or posz>=0.20  else 1
        rheight = np.exp(-50*(np.round((posz-0.018),4))**2)

        reward = np.round(area_penalty,5) + penalty + np.round(rvel,5) + np.round(rstep,4) + np.round(reward_ori,5) + rheight + 500*leap
        # print(f'rewards {}') #only for debugging

        # check if done
        if posz <= 0.1 or n_step>=10000 or abs(ori[0])>=0.4 or abs(ori[1])>=0.4 or abs(ori[2])>=0.4: #about 40 deg
            reward = 0
            self.done = True
            print(f'no steps {n_step}') #only for debugging
        
            
        return status, reward, self.done, dict()
        

    def reset(self):
        p.resetSimulation(self.client)
        Plane(self.client)
        self.robot = Robot(self.client)
        p.setGravity(0,0,-10)
        self.done = False
        self.last_dist = 0

        return np.full(12,0)
        

    def render(self):
        pass

    def close(self):
        p.disconnect(self.client)
        
        
        
    def seed(self, seed=None): 
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
