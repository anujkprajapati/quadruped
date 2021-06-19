import math
import numpy as np


class Path:
    def __init__(self):
        pass
    

    # def get_location(odd,n_step,count,posx):
        
    #     stride = 0.05
    #     height = 0.03
    #     # basex = n_step/count * stride
    #     basex = posx + stride
    #     theta = n_step%count * np.pi
    #     if odd ==0:
    #         # zdirection
    #         h2z = 0.05
    #         h3z = 0.05
    #         h1z = 0.05 + height*np.sin(theta)
    #         h4z = 0.05 + height*np.sin(theta)

    #         #xdirection
    #         h1x = basex + 0.102 - stride/2 * np.cos(theta)
    #         h4x = basex - 0.102 - stride/2 * np.cos(theta)
    #         h2x = basex + 0.102
    #         h3x = basex - 0.102

    #     else: 
    #         # zdirection
    #         h1z = 0.05
    #         h4z = 0.05
    #         h2z = 0.05 + height*np.sin(theta)
    #         h3z = 0.05 + height*np.sin(theta)

    #         #xdirection
    #         h2x = basex + 0.102 - stride/2 * np.cos(theta)
    #         h3x = basex - 0.102 - stride/2 * np.cos(theta)
    #         h1x = basex + 0.102
    #         h4x = basex - 0.102
    #     # print(f'the count is{count}')
    #     return h1x, h2x, h3x, h4x, h1z, h2z, h3z, h4z


    def inv2d(self,y,x,leg):
        a=0.1
        b=0.1
        
        maxdist = a**2 + b**2
        dist2 = np.clip(x**2 + y**2, 0, maxdist)
        theta3 = np.round(np.arccos((dist2 - maxdist)/(2*a*b)),4)
        if leg==3 or leg==4:
            if theta3>0:
                theta3 = -theta3
        theta2 = np.round((np.arctan2(y,x) - np.arctan2(b*np.sin(theta3),(a+b*np.cos(theta3)))),4)
        return theta2, theta3
    
    def invkin(self,x,y,z,leg,pitch):

        m=x
        x = x*np.cos(pitch)+z*np.sin(pitch)
        z = -m*np.sin(pitch)+z*np.cos(pitch)
        p = 0.065
        
        if leg==1:
            X = (x-0.1)
            theta1 = np.round(np.arctan2((y-p),z),4)
            Z = z/np.cos(theta1)
            # print(f'leg {leg} {theta1}')
            theta2, theta3 = self.inv2d(X,Z,leg)
            return theta1, theta2 + 0.5, 1 - theta3 
        elif leg==2:
            X = (x-0.1)
            theta1 = np.round(np.arctan2((-y-p),z),4)
            # print(f'leg {leg} {theta1}')
            Z = z/np.cos(theta1)
            theta2, theta3 = self.inv2d(X,Z,leg)            
            return theta1, theta2 + 0.5, 1 - theta3 
        elif leg ==3:
            X = x+0.1
            theta1 = np.round(np.arctan2((y-p),z),4)
            # print(f'leg {leg} {theta1}')
            Z = z/np.cos(theta1)
            theta2, theta3 = self.inv2d(X,Z,leg)
            # print(theta1, theta2, theta3)
            return theta1, -theta2 + 0.5, -theta3 - 1
        elif leg ==4:
            X = (x+0.1)
            theta1 = np.round(np.arctan2((-y-p),z),4)
            # print(f'leg {leg} {theta1}')
            Z = z/np.cos(theta1)
            theta2, theta3 = self.inv2d(X,Z,leg)
            return theta1, -theta2 + 0.5, -theta3 -1

        

    def get_pt(self,h,h0,k,k0,height, theta,leg,posz):
        
        if theta<100:
            
            if leg==1 or leg==4:
                th = theta*np.pi /100
            else: 
                th = 0

        else:
            if leg==3 or leg==2:
                th = (theta-100)*np.pi/100
            else:
                th=0

        x = h - h0*np.cos(th)
        y = k - k0*np.cos(th)
        z = posz - height*np.sin(th)
        return x,y,z











        