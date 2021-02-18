
from BuggySimulator import *
import numpy as np
import scipy
from scipy.ndimage import gaussian_filter1d
from util import *
import scipy.signal
import math 

class controller():
    
    def __init__(self,traj,vehicle, e=0, e111=0, e_d=0, e222=0):
        self.vehicle=vehicle
        self.traj=traj
        self.e = e
        self.e111 = e111
        self.e_d = e_d
        self.e222 = e222

        # Add additional member variables according to your need here.

    def control_update(self):

        traj=self.traj
        vehicle=self.vehicle 
        
        lr = vehicle.lr
        lf = vehicle.lf
        Ca = vehicle.Ca
        Iz = vehicle.Iz
        f = vehicle.f
        m = vehicle.m
        g = vehicle.g

        delT = 0.05  # The simulator runs at a constant fps.

        #reading current vehicle states
        X = vehicle.state.X
        Y = vehicle.state.Y
        xdot = vehicle.state.xd
        ydot = vehicle.state.yd
        phi = vehicle.state.phi
        phidot = vehicle.state.phid
        delta = vehicle.state.delta
        Vx = xdot
        
       

        dist, index1 = closest_node(X, Y, traj)

        n = 50

        index = index1 + n

        if(index>=8203):
        	index = 8202

        X_new = traj[index][0]
        Y_new = traj[index][1]

        K_p = 100
      	K_i = 0.00001
      	K_d = 1.5

      	dist1 = math.sqrt((Y_new - Y)**2 + (X_new - X)**2)

      	v_d = dist1/(n*delT)

      	
      	
      	v = math.sqrt(xdot**2 + ydot**2)

      	
      	# print(self.e)

      	

      	







        # ---------------|Lateral Controller|-------------------------
        
        e11 = (v_d - v)
      	
      	F = K_p*(e11) + K_i*(self.e + e11) + K_d*(e11 - self.e111)
      	self.e += e11
        self.e111 = e11 
        

        #--------|Longitudinal Controller|------------------------------
        
        e22 = wrap2pi(np.arctan2(Y_new-Y, X_new-X)-phi-delta)

        deltad = K_p*(e22) + K_i*(self.e_d + e22) + K_d*(e22 - self.e222)
      	self.e_d += e22
        self.e222 = e22 

        
        
        # Communicating the control commands with the BuggySimulator
        controlinp = vehicle.command(F,deltad)
        # F: Force
        # deltad: desired rate of steering command

        return controlinp,Vx



