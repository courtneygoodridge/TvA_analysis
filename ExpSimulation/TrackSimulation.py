import sys

rootpath = "C:/Users/psccgoo/OneDrive - University of Leeds/TvA_analysis/ExpSimulation" 
# change "C:/Users/psccgoo/OneDrive - University of Leeds" to the location of where you save TvA_analysis on your machine.
sys.path.append(rootpath)

import numpy as np
from numpy import savetxt
import matplotlib.pyplot as plt
import pdb
import pandas as pd
import math as mt
import os
import simTrackMaker

class vehicle:
    
    def __init__(self, initialyaw, speed, dt, Course):

        
        self.pos = [0, 0]
        self.yaw = initialyaw #heading offset angle, radians
        self.speed = speed 
        self.dt = dt       
        self.midline = Course
    
        
        self.yawrate = 0
        
        self.pos_history = []
        self.yaw_history = []
        self.yawrate_history = []                
        self.steering_bias_history = []   
        self.closestpt_history = []         
        self.vis_angle_corrected_history = []
        self.vis_angle_history = []

        self.Course = Course      
        
        self.closest_pt, self.current_steering_bias, self.current_vis_angle_corrected, self.current_vis_angle  = self.calculate_errors() 


        #calculate road angle with correction.      

        #calculate road angle without correction.      

        #self.save_history()     
        

    def calculate_errors(self, ego_dist = 10, camera_rotation = 0): # 2.5

        #calculates lane error (steering bias), road angle, and road angle with camera rotation.
        
        #for straights, steering bias is just x.
        steering_bias = self.pos[0]

        #for road angle we find the point in the road at ego_dist ahead (10 m is default)

        midlinedist = np.sqrt(
            ((self.pos[0]-self.midline[:,0])**2)
            +((self.pos[1]+ego_dist-self.midline[:,1])**2)
            ) #get a 4000 array of distances from the midline
        idx = np.argmin(abs(midlinedist)) #find smallest difference. This is the closest index on the midline.	

        chosen_pt = self.midline[idx,:] #xy of chosen point

        #when is camera_aligned with z axis, the angle is tan(opp/adj) = tan(x/z)
        x_dist = abs(self.pos[0] - chosen_pt[0])
        z_dist = abs(self.pos[1] - chosen_pt[1])
        vis_angle_corrected = mt.tan(x_dist / z_dist) #distance from closest point				

        #when camera rotation is veridical to yaw you add the yaw angle.

        vis_angle = vis_angle_corrected + camera_rotation


        return chosen_pt, steering_bias, vis_angle_corrected, vis_angle


    def move_vehicle(self, newyawrate = 0):           
        """update the position of the vehicle over timestep dt"""                        
                                 
        self.yawrate = newyawrate

        # self.yawrate = np.deg2rad(0.5) # np.random.normal(0, 0.001)

        maxheadingval = np.deg2rad(35.0) #in rads per second
        
        self.yawrate = np.clip(self.yawrate, -maxheadingval, maxheadingval)
        # print(self.yawrate)
        # self.yawrate = 0.0

        self.yaw = self.yaw + self.yawrate * self.dt  #+ np.random.normal(0, 0.005)
        
        #zrnew = znew*cos(omegaH) + xnew*sin(omegaH);
        #xrnew = xnew*cos(omegaH) - znew*sin(omegaH)

        x_change = self.speed * self.dt * np.sin(self.yaw)
        y_change = self.speed * self.dt * np.cos(self.yaw)
        
        self.pos = self.pos + np.array([x_change, y_change]) 

        self.closest_pt, self.current_steering_bias, self.current_vis_angle_corrected, self.current_vis_angle = self.calculate_errors(camera_rotation=self.yaw)
        
        self.save_history()
    
    def save_history(self):

        self.pos_history.append(self.pos)        
        self.yaw_history.append(self.yaw)
        self.yawrate_history.append(self.yawrate)
        self.steering_bias_history.append(self.current_steering_bias)
        self.closestpt_history.append(self.closest_pt)
        self.vis_angle_corrected_history.append(self.current_vis_angle_corrected)
        self.vis_angle_history.append(self.current_vis_angle)   

def runSimulation(Course, headingoffset= 0, onsettime = 0):

    """run simulation and return RMS"""

    #Sim params
    fps = 60.0
    speed = 8.0
 
   # print ("speed; ", speed)

    dt = 1.0 / fps # 1.0
    run_time = 4 #seconds
    time = 0

    Car = vehicle(headingoffset, speed, dt, Course)

    i = 0

    while (time < run_time):

        time += dt              
        
        Car.move_vehicle()           

        i += 1

    return Car
    
if __name__ == '__main__':
    

    #create straight that is arbitrarily long. 
    mystraight  = simTrackMaker.lineStraight(startpos = [0,0], length= 200, size = 2000)#, texturefile='strong_edge_soft.bmp')    
    #mystraight  = simTrackMaker.lineBend(startpos = [0,0], rads = 2000, size = 500)
        
    Course = mystraight.midline
    #create array of heading offsets. keep in radians
    headingoffsets = np.deg2rad(np.linspace(0.5,2,4)) # linspace(1, 10, 10) [0]

    totalrows = len(headingoffsets) 
    
    simResults = []

    fig, a =  plt.subplots(1,3, figsize = [25,10])

    row_i = 0    
    for ho_i,ho in enumerate(headingoffsets):        
        
        print("ran")
        Car = runSimulation(Course, headingoffset = ho)

        #Append results.
        simResults.append(Car)

    #now plot lane positions over time for each condition.
    #matplotlib.style.use('classic') 
    """ plt.figure(1)

    for i, sim in enumerate(simResults):
        
        print(sim)
        steeringbias = np.array(sim.steering_bias_history)

        a[2].plot(np.arange(0, 2, 0.01652892561983471), steeringbias, 'black') # range(len(steeringbias)),
        a[2].set_xlabel("Time (s)", fontsize = 30)
        a[2].set_ylabel("Positional Error (m)", fontsize = 30)
        a[2].tick_params(axis = 'x', labelsize = 25) 
        a[2].tick_params(axis = 'y', labelsize = 25)
        a[2].text(0.05, 0.55, "C", fontweight ="bold", fontsize = 50)
        
        #plt.title("Heading offsets of 0.5°-2°")
        #plt.savefig('PositionalError_preview_2.5_0.5°-2°.png', dpi = 300)
        #plt.show() """
    

    #egocentric angle to point at 10 m ahead without corrected camera.
    #matplotlib.style.use('classic')
    """ plt.figure(2)

    for i, sim in enumerate(simResults):
        
        print(sim)
        vis_angle = np.rad2deg(np.array(sim.vis_angle_history))
        a[0].plot(np.arange(0, 2, 0.01652892561983471), vis_angle, 'black') # range(len(vis_angle))
        a[0].set_xlabel("Time (s)", fontsize = 30)
        a[0].set_ylabel(r"Veridical (unrotated) $\alpha\, (\degree)$", fontsize = 30)
        a[0].tick_params(axis = 'x', labelsize = 25) 
        a[0].tick_params(axis = 'y', labelsize = 25)
        a[0].text(0.05, 6.4, "A", fontweight ="bold", fontsize = 50) """
    #plt.title("Heading offsets of 0.5°-2°")
    
    #plt.savefig('VisualAngle_preview_2.5_0.5°-2°.png', dpi = 300)
    #plt.show()

    #egocentric angle to point at 10 m ahead with corrected camera
    #matplotlib.style.use('classic')
    """ plt.figure(3)

    for i, sim in enumerate(simResults):
        
        print(sim)
        vis_angle_corrected = np.rad2deg(np.array(sim.vis_angle_corrected_history))
        a[1].plot(np.arange(0, 2, 0.01652892561983471), vis_angle_corrected, 'black') # range(len(vis_angle_corrected)) np.arange(0, 2, 0.01652892561983471)
        a[1].set_xlabel("Time (s)", fontsize = 30)
        a[1].set_ylabel(r"Rotated Camera $\alpha\, (\degree)$", fontsize = 30)
        a[1].tick_params(axis = 'x', labelsize = 25) 
        a[1].tick_params(axis = 'y', labelsize = 25)
        a[1].text(0.05, 4.1, "B", fontweight ="bold", fontsize = 50)
        fig.tight_layout()

    #plt.title("Heading offsets of 0.5°-2°")
    
    fig.savefig('VisualAngle_PositionalError.tif', dpi = 300, units = "cm", width = 14, height = 7) """
    #plt.show()


# Combined plots

# saves bias after 9 frames for each heading offset condition. This is the additional lateral position covered during the motor latency period
""" motor_bias = []

for i, sim in enumerate(simResults):
        steeringbias = np.array(sim.steering_bias_history)

        motor_bias.append(sim.steering_bias_history[9])

        plt.plot(range(len(steeringbias)), steeringbias, 'black')
        plt.xlim(0, 20)
        plt.ylim(0, 0.15)
        plt.xlabel("Frames")
        plt.ylabel("Positional Error (m)")
        plt.vlines(x = 9, ymin = 0, ymax = 0.046)
        plt.hlines(y = 0.046, xmin = 0, xmax = 9)
        plt.hlines(y = 0.012, xmin = 0, xmax = 9)
 """
#Three panel plot each one zooming into th section between 0 and 20 frames to show increase in lateral position caused by motor latency 

#motor_bias[0] # 0.5 heading
#motor_bias[3] # 1 heading
#motor_bias[6] #1.5 heading
#motor_bias[9] # 2 heading


### Saving out simulation data for R plotting

# steering bias
latpos = []

for i, sim in enumerate(simResults):
    steeringbias = np.array(sim.steering_bias_history)
    latpos.append(steeringbias)

latpos = np.transpose(latpos)

savetxt('TrackSimSteeringBias.csv', latpos, delimiter = ',')


# visual angle (uncorrected)
visangle = []

for i, sim in enumerate(simResults):
    vis_angle = np.rad2deg(np.array(sim.vis_angle_history))
    visangle.append(vis_angle)

visangle = np.transpose(visangle)

savetxt('TrackSimEgoVisAngle.csv', visangle, delimiter = ',') #TrackSimEgoVisAngle

# visual angle (corrected)
visanglecorrected = []

for i, sim in enumerate(simResults):
    vis_angle_corrected = np.rad2deg(np.array(sim.vis_angle_corrected_history))

    visanglecorrected.append(vis_angle_corrected)

visanglecorrected = np.transpose(visanglecorrected)

savetxt('TrackSimVisAngleCorrected.csv', visanglecorrected, delimiter = ',')

