import sys

import numpy as np
import matplotlib.pyplot as plt
import pdb
import pandas as pd
import math as mt


from __future__ import division

import simTrackMaker

class vehicle:
    
    def __init__(self, initialyaw, speed, dt, Course, ego = 10, alphadot = 30):

        
        self.pos = np.array([0, 0])
        self.yaw = initialyaw #heading offset angle, radians
        self.speed = speed 
        self.dt = dt       
        self.midline = Course
        self.ego_dist = ego        
        self.k_alphadot = alphadot
        
        self.yawrate = 0
        
        self.pos_history = []
        self.yaw_history = []
        self.yawrate_history = []                
        self.steering_bias_history = []   
        self.closestpt_history = []         
        self.vis_angle_corrected_history = []
        self.vis_angle_corrected_diff_history = []
        self.vis_angle_history = []
        self.vis_angle_diff_history = []
        self.alphaalphadot = []


        
        self.past_angle = 0
        self.Course = Course      
        
        self.closest_pt, self.current_steering_bias, self.current_vis_angle_corrected, self.current_vis_angle, self.corrected_diff, self.vis_diff  = self.calculate_errors() 

    

        #calculate road angle with correction.      

        #calculate road angle without correction.      

        self.save_history()     
        

    def calculate_errors(self, camera_rotation = 0):

        #calculates lane error (steering bias), road angle, and road angle with camera rotation.
        
        #for straights, steering bias is just x.
        steering_bias = self.pos[0]

        #for road angle we find the point in the road at ego_dist ahead (10 m is default)
        #print("pos", self.pos)
        #print("mid", self.midline[:,0])

        midlinedist = np.sqrt(
            ((self.pos[0]-self.midline[:,0])**2)
            +((self.pos[1]+self.ego_dist-self.midline[:,1])**2)
            ) #get a 4000 array of distances from the midline
        idx = np.argmin(abs(midlinedist)) #find smallest difference. This is the closest index on the midline.	

        chosen_pt = self.midline[idx,:] #xy of chosen point

        #when is camera_aligned with z axis, the angle is tan(opp/adj) = tan(x/z)
        x_dist = abs(self.pos[0] - chosen_pt[0])
        z_dist = abs(self.pos[1] - chosen_pt[1])
        vis_angle_corrected = mt.tan(x_dist / z_dist) #distance from closest point				
        corrected_diff = vis_angle_corrected - self.past_angle
        #print(corrected_diff)
        #hehe
        #when camera rotation is veridical to yaw you add the yaw angle.

        vis_angle = vis_angle_corrected + camera_rotation
        vis_diff = vis_angle -(self.past_angle+camera_rotation)


        self.past_angle = vis_angle_corrected

        return chosen_pt, steering_bias, vis_angle_corrected, vis_angle, corrected_diff, vis_diff


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

        self.pos = np.add(self.pos, np.array([x_change, y_change]))                

        self.closest_pt, self.current_steering_bias, self.current_vis_angle_corrected, self.current_vis_angle, self.corrected_diff, self.vis_diff = self.calculate_errors(camera_rotation=self.yaw)
        
        self.save_history()
    
    def save_history(self):

        self.pos_history.append(self.pos)        
        self.yaw_history.append(self.yaw)
        self.yawrate_history.append(self.yawrate)
        self.steering_bias_history.append(self.current_steering_bias)
        self.closestpt_history.append(self.closest_pt)
        self.vis_angle_corrected_history.append(self.current_vis_angle_corrected)
        self.vis_angle_history.append(self.current_vis_angle) 
        self.vis_angle_corrected_diff_history.append(self.corrected_diff)  
        self.vis_angle_diff_history.append(self.vis_diff)  

        self.alphaalphadot.append(self.current_vis_angle + (self.k_alphadot*self.corrected_diff))

def runSimulation(Course, headingoffset= 0, onsettime = 0, ego = 10):

    """run simulation and return RMS"""

    #Sim params
    fps = 60
    speed = 8.0
 
   # print ("speed; ", speed)

    dt = 1.0 / fps
    run_time = 2.5 #seconds
    time = 0

    Car = vehicle(headingoffset, speed, dt, Course, ego = ego)

    i = 0

    while (time < run_time):

        time += dt              
        
        Car.move_vehicle()           

        i += 1

    return Car

def simulate_scenarios(headingoffsets = np.deg2rad(np.linspace(0,10,10)), ego = 10):
    
    #create straight that is arbitrarily long. 
    #change size for efficiency. Low size increases speed but also increases discretization errors.
    mystraight  = simTrackMaker.lineStraight(startpos = [0,0], length= 200, size = 500000)
    Course = mystraight.midline
    
    simResults = []
    
    for ho in headingoffsets:        
        
        print("ran, ho: ", ho)        
        Car = runSimulation(Course, headingoffset = ho, ego = ego)

        #Append results.
        simResults.append(Car)

    return (simResults)


def threshold(arr, thres = .5):

    """takes a threshold and array and returns an index. Thresh needs to be in units of arr"""

    frames = len(arr[arr<thres]) #+ 9 #  #150ms motor latency  #number of frames it took to get to threshold
    return (frames)


def accum(arr, thres = 10):

    """takes a threshold and returns an index
        Here the threshold is metres units and the accumulation rate is one
    """

    cumsum = np.cumsum(arr)
    frames = len(cumsum[cumsum<thres]) #+ 9 # #150ms motor latency 
        
    return(frames)


def predict_response(simResults, startingpos = [0, .5, 1], ego = 10, alphadot = 30): # for experiment 2, starting positions were startingpos = [0, .5, 1]

    """predict response times based on vis angle, vis angle + diff(vis angle), and lateral position, on threshold or accumulator"""

    startingpos = np.array(startingpos) * 60.0 #in frames so we can use it as the index.

    #array for RTs    
    thres_resps = np.empty([len(simResults), len(startingpos), 3])
    accum_resps = np.empty([len(simResults), len(startingpos), 3])

    #array for lpos
    thres_err = np.empty([len(simResults), len(startingpos), 3])
    accum_err = np.empty([len(simResults), len(startingpos), 3])

    #Scaled by preview for the angular ones.
    thresholds = {0: 2/ego, 1: 9/ego, 2: .03}   
    decision_boundary = {0: 3/ego, 1: 270/ego, 2: 0.6} 


    for sim_i, sim in enumerate(simResults):
        
        steeringbias = np.array(sim.steering_bias_history) 
        alpha = np.rad2deg(np.array(sim.vis_angle_corrected_history))
        alphaalphadot = np.rad2deg(np.array(sim.alphaalphadot))

        perc_vars = [alpha, alphaalphadot, steeringbias]

        for sp_i, sp in enumerate(startingpos):
         
            for p_i, pv in enumerate(perc_vars): #loop through perceptual signals
                
                arr = pv[int(sp):]

                idx = threshold(arr, thres = thresholds[p_i])
                thres_resps[sim_i, sp_i, p_i] = idx / 60 #timing


                thres_err[sim_i, sp_i, p_i] = steeringbias[np.clip(idx, 0, len(steeringbias)-1)] #error at response
                            
                idx2 = accum(arr, thres = decision_boundary[p_i])
                accum_resps[sim_i, sp_i, p_i] = idx2 / 60
                accum_err[sim_i, sp_i, p_i] = steeringbias[np.clip(idx2, 0, len(steeringbias)-1)]

    return([thres_resps, thres_err], [accum_resps, accum_err])


headingoffsets = np.deg2rad(np.linspace(0.5,2,4)) # np.deg2rad(np.linspace(0.5,2,4))
simResults = simulate_scenarios(headingoffsets, 8)
pred_thres, pred_accum = predict_response(simResults, startingpos=[0], ego = 8)

#RTs predictions without motor latency
threshold_rt = pred_thres[0][:, 0]
accum_rt = pred_accum[0][:, 0]

threshold_rt = pd.DataFrame(threshold_rt)
threshold_rt.columns = ['LPE', 'alphadot', 'alpha']
np.savetxt("threshold_rt.csv", threshold_rt, delimiter = ",")

accum_rt = pd.DataFrame(accum_rt)
accum_rt.columns = ['LPE', 'alphadot', 'alpha']
np.savetxt("accum_rt.csv", accum_rt, delimiter = ",")

# Lateral position error predictions without motor latency
threshold_lpe = pred_thres[1][:, 0]
accum_lpe = pred_accum[1][:, 0]

threshold_lpe = pd.DataFrame(threshold_lpe)
threshold_lpe.columns = ['LPE', 'alphadot', 'alpha']
np.savetxt("threshold_lpe.csv", threshold_lpe, delimiter = ",")

accum_lpe = pd.DataFrame(accum_lpe)
accum_lpe.columns = ['LPE', 'alphadot', 'alpha']
np.savetxt("accum_lpe.csv", accum_lpe, delimiter = ",")


# For predictions with addition of motor latency run threshold() and accum() function with +9 function. 

def threshold(arr, thres = .5):
    
    """takes a threshold and array and returns an index. Thresh needs to be in units of arr"""

    frames = len(arr[arr<thres]) + 9 #  #150ms motor latency  #number of frames it took to get to threshold
    return (frames)


def accum(arr, thres = 10):

    """takes a threshold and returns an index
        Here the threshold is metres units and the accumulation rate is one
    """

    cumsum = np.cumsum(arr)
    frames = len(cumsum[cumsum<thres]) + 9 # #150ms motor latency 
        
    return(frames)


headingoffsets = np.deg2rad(np.linspace(0.5,2,4)) # np.deg2rad(np.linspace(0.5,2,4))
simResults = simulate_scenarios(headingoffsets, 8)
pred_thres_latency, pred_accum_latency = predict_response(simResults, startingpos=[0], ego = 8)

#RTs predictions WITH motor latency
threshold_rt_latency = pred_thres_latency[0][:, 0]
accum_rt_latency = pred_accum_latency[0][:, 0]

threshold_rt_latency = pd.DataFrame(threshold_rt_latency)
threshold_rt_latency.columns = ['LPE', 'alphadot', 'alpha']
np.savetxt("threshold_rt_latency.csv", threshold_rt_latency, delimiter = ",")

accum_rt_latency = pd.DataFrame(accum_rt_latency)
accum_rt_latency.columns = ['LPE', 'alphadot', 'alpha']
np.savetxt("accum_rt_latency.csv", accum_rt_latency, delimiter = ",")

# Lateral position error predictions WITH motor latency
threshold_lpe_latency = pred_thres_latency[1][:, 0]
accum_lpe_latency = pred_accum_latency[1][:, 0]

threshold_lpe_latency = pd.DataFrame(threshold_lpe_latency)
threshold_lpe_latency.columns = ['LPE', 'alphadot', 'alpha']
np.savetxt("threshold_lpe_latency.csv", threshold_lpe_latency, delimiter = ",")

accum_lpe_latency = pd.DataFrame(accum_lpe_latency)
accum_lpe_latency.columns = ['LPE', 'alphadot', 'alpha']
np.savetxt("accum_lpe_latency.csv", accum_lpe_latency, delimiter = ",")