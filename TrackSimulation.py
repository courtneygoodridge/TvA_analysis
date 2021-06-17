import sys
rootpath = "C:/Users/pscmgo/OneDrive for Business/PhD/Project/Experiment_Code/expsims"
sys.path.append(rootpath)


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

def plot_perceptual_variables(axes, simResults, headingoffsets, ego = 10):    
    

    for (sim, ho) in zip(simResults, headingoffsets):
        
        #print(sim)
        
    
        #egocentric angle to point at 10 m ahead without corrected camera.
        """
        print(sim)
        vis_angle = np.array(sim.vis_angle_history)
        axes[1].plot(range(len(vis_angle)), vis_angle, 'r-')        
        axes[1].set_ylabel("Veridical (unrotated) Visual Angle (m)")
        """

        #egocentric angle to point at 10 m ahead with corrected camera
        vis_angle_corrected = np.array(sim.vis_angle_corrected_history)
        matplotlib.style.use('classic') 
        axes[0].plot(range(len(vis_angle_corrected)), np.rad2deg(vis_angle_corrected), label = f"Heading offset: {ho:.1f} degs")
        axes[0].set_ylabel(f"PREVIEW = {ego} (m) \n" + r"$\alpha\, (\degree)$")
        axes[0].legend()

        
        alphaalphadot = np.array(sim.alphaalphadot)
        matplotlib.style.use('classic')         
        axes[1].plot(range(len(alphaalphadot)), np.rad2deg(alphaalphadot))
        axes[1].set_ylabel(r"$\alpha + \dot{\alpha}\, (\degree + \degree/s)$")

        steeringbias = np.array(sim.steering_bias_history)
        matplotlib.style.use('classic') 
        axes[2].plot(range(len(steeringbias)), steeringbias )
        axes[2].set_ylabel("Positional Error (m)")
    
    
    for ax in axes:
        ax.set_xlabel('Frames')
    
    
    


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

    frames = len(arr[arr<thres]) + 9 #  #150ms motor latency  #number of frames it took to get to threshold
    return (frames)


def accum(arr, thres = 10):

    """takes a threshold and returns an index
        Here the threshold is metres units and the accumulation rate is one
    """

    cumsum = np.cumsum(arr)
    frames = len(cumsum[cumsum<thres]) + 9 # #150ms motor latency 
        
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

    # hard coded thresholds FOR EXPERIMENT 1 for each error signal for convenience. Scaled by preview for the angular ones.
    #thresholds = {0: 2/ego, 1: 9/ego, 2: .03}   
    #decision_boundary = {0: 3/ego, 1: 270/ego, 2: 0.6} 

    # hard coded thresholds FOR EXPERIMENT 2 for response error
    #thresholds = {0: 1/ego, 1: 30/ego, 2: .06} #thresholds = {0: 15/ego, 1: 30/ego, 2: .5}    
    #decision_boundary = {0: 800/ego, 1: 300/ego, 2: 10} # 50 decision_boundary = {0: 150/ego, 1: 300/ego, 2: 20}

    #hard coded thresholds FOR EXPERIMENT 2 for reaction time
    thresholds = {0: 13/ego, 1: 30/ego, 2: .22}
    decision_boundary = {0: 100/ego, 1: 300/ego, 2: 2}
    

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

        
def plot_responses(predictions, ego):

    model_names = {0: "threshold", 1: "accumulator"}
    measures = {0: 'RT', 1: 'Pos Error'}
    perc_vars = {0: 'Alpha', 1: 'Alpha + Alphadot', 2: 'Pos Error'}
    startpos = {0: '0 s', 1: '.5 s', 2: '1 s'}

    for i, model in enumerate(predictions):

        f, axes = plt.subplots(2,3, figsize = [14,10]) #separate figure for threshold and accumulator

        for im, measure in enumerate(model): #loop through RT or error
            
            for pv in [0,1,2]: #loop through perceptual signals
                
                signal = measure[:,:,pv]

                for sp, col in enumerate(signal.T): 

                    print(col)
                    
                    #line graph with headings on one side
                    axes[im, pv].plot(range(len(signal)), col, label = f"Start pos = {startpos[sp]}")
                    axes[im, pv].set_ylabel(measures[im])                    
                    axes[im, pv].set_xlabel("Heading conditions index")
                    axes[im, pv].set_title(perc_vars[pv])


        
        #axes[0,0].legend()         
        f.suptitle(model_names[i] + ', Preview: ' + str(ego) + ' m')    
        f.savefig(model_names[i] + '_' + str(ego) + '.png', dpi = 300)
        f.show()
        
    
if __name__ == '__main__':
    

    #run the simulations for all heading offsets, with starting position zero
    headingoffsets = np.deg2rad(np.linspace(0.5,2,4)) # np.deg2rad(np.arange(1,5)) 
    ego_dists = [1,4,8]

    fig, axes = plt.subplots(len(ego_dists),3, figsize = [20,10])
    
    for i,ego in enumerate(ego_dists):    
        simResults = simulate_scenarios(headingoffsets, ego)
    
        #plot perc variables
        matplotlib.style.use('classic') 
        plot_perceptual_variables(axes[i,:], simResults, np.rad2deg(headingoffsets), ego)    

        #predict response
        #give the starting positions in time. [0, .5, 1] is same as [0, 4, 8]
        pred_thres, pred_accum = predict_response(simResults, startingpos=[0, .5, 1], ego = ego) # for experiment 2, starting positions were startingpos = [0, .5, 1]

        #plot the responses
        matplotlib.style.use('classic') 
        plot_responses([pred_thres, pred_accum], ego)


    fig.suptitle("Speed = 8 m/s")
    fig.savefig('perc_variables.png', dpi = 300)
    fig.show()

    plt.show()

#manuscript plots

# responses without motor latency
headingoffsets = np.deg2rad(np.linspace(0.5,2,4)) # np.deg2rad(np.linspace(0.5,2,4))
simResults = simulate_scenarios(headingoffsets, 10)
pred_thres, pred_accum = predict_response(simResults, startingpos=[0], ego = 8)

threshold_response = pred_thres[0][:, 0]
accum_response = pred_accum[0][:, 0]

# responses with motor latency
headingoffsets = np.deg2rad(np.linspace(0.5,2,4)) # np.deg2rad(np.linspace(0.5,2,4))
simResults = simulate_scenarios(headingoffsets, 8)
pred_thres_latency, pred_accum_latency = predict_response(simResults, startingpos=[0], ego = 8)

threshold_response_latency = pred_thres_latency[0][:, 0]
accum_response_latency = pred_accum_latency[0][:, 0]


### Threshold and Accumulator RT prediction pattern when operating under alpha, alphadot and positional error

fig, a =  plt.subplots(2,3, figsize = [14,10])
a[0][0].plot(np.degrees(headingoffsets), threshold_response[:, 0], color = "blue", alpha = 0.3)
a[0][0].plot(np.degrees(headingoffsets), threshold_response_latency[:, 0], color = "blue")
a[0][0].axes.yaxis.set_ticks([])
a[0][0].set_xlim(0.5, 2)
a[0][0].set_ylabel("RT (s)")
a[0][0].set_xlabel("Heading (°)")
a[0][0].set_title(r"Threshold: $\alpha\, (\degree)$")
a[0][1].plot(np.degrees(headingoffsets), threshold_response[:, 1], color = "blue", alpha = 0.3)
a[0][1].plot(np.degrees(headingoffsets), threshold_response_latency[:, 1], color = "blue")
a[0][1].axes.yaxis.set_ticks([])
a[0][1].set_xlim(0.5, 2)
a[0][1].set_xlabel("Heading (°)")
a[0][1].set_title(r"Threshold: $\alpha + \dot{\alpha}\, (\degree + \degree/s)$")
a[0][2].plot(np.degrees(headingoffsets), threshold_response[:, 2], color = "blue", alpha = 0.3)
a[0][2].plot(np.degrees(headingoffsets), threshold_response_latency[:, 2], color = "blue")
a[0][2].axes.yaxis.set_ticks([])
a[0][2].set_xlim(0.5, 2)
a[0][2].set_xlabel("Heading (°)")
a[0][2].set_title("Threshold: Positional error (m)")
a[1][0].plot(np.degrees(headingoffsets), accum_response[:, 0], color = "red", alpha = 0.3)
a[1][0].plot(np.degrees(headingoffsets), accum_response_latency[:, 0], color = "red")
a[1][0].set_ylabel("RT (s)")
a[1][0].set_xlabel("Heading (°)")
a[1][0].axes.yaxis.set_ticks([])
a[1][0].set_xlim(0.5, 2)
a[1][0].set_title(r"Accumulator: $\alpha\, (\degree)$")
a[1][1].plot(np.degrees(headingoffsets), accum_response[:, 1], color = "red", alpha = 0.3)
a[1][1].plot(np.degrees(headingoffsets), accum_response_latency[:, 1], color = "red")
a[1][1].set_xlabel("Heading (°)")
a[1][1].axes.yaxis.set_ticks([])
a[1][1].set_xlim(0.5, 2)
a[1][1].set_title(r"Accumulator: $\alpha + \dot{\alpha}\, (\degree + \degree/s)$")
a[1][2].plot(np.degrees(headingoffsets), accum_response[:, 2], color = "red", alpha = 0.3)
a[1][2].plot(np.degrees(headingoffsets), accum_response_latency[:, 2], color = "red")
a[1][2].set_xlabel("Heading (°)")
a[1][2].axes.yaxis.set_ticks([])
a[1][2].set_xlim(0.5, 2)
a[1][2].set_title("Accumulator: Positional error (m)")
fig.savefig('ThresVAccum_Perceptual_Variables_RT.png', dpi = 300)

### Threshold and Accumulator positional error prediction pattern when operating under alpha, alphadot and positional error

# responses without motor latency
threshold_error = pred_thres[1][:, 0]
accum_error = pred_accum[1][:, 0]

# responses with motor latency
threshold_error_latency = pred_thres_latency[1][:, 0]
accum_error_latency = pred_accum_latency[1][:, 0]

fig, a =  plt.subplots(2,3, figsize = [14,10])
a[0][0].plot(np.degrees(headingoffsets), threshold_error[:, 0], color = "blue", alpha = 0.3)
a[0][0].plot(np.degrees(headingoffsets), threshold_error_latency[:, 0], color = "blue")
a[0][0].axes.yaxis.set_ticks([])
a[0][0].set_xlim(0.5, 2)
a[0][0].set_ylim(0, 0.50)
a[0][0].set_ylabel("Lateral positional error (m)")
a[0][0].set_xlabel("Heading (°)")
a[0][0].set_title(r"Threshold: $\alpha\, (\degree)$")
a[0][1].plot(np.degrees(headingoffsets), threshold_error[:, 1], color = "blue", alpha = 0.3)
a[0][1].plot(np.degrees(headingoffsets), threshold_error_latency[:, 1], color = "blue")
a[0][1].axes.yaxis.set_ticks([])
a[0][1].set_xlim(0.5, 2)
a[0][1].set_ylim(0, 0.20)
a[0][1].set_xlabel("Heading (°)")
a[0][1].set_title(r"Threshold: $\alpha + \dot{\alpha}\, (\degree + \degree/s)$")
a[0][2].plot(np.degrees(headingoffsets), threshold_error[:, 2], color = "blue", alpha = 0.3)
a[0][2].plot(np.degrees(headingoffsets), threshold_error_latency[:, 2], color = "blue")
a[0][2].axes.yaxis.set_ticks([])
a[0][2].set_xlim(0.5, 2)
a[0][2].set_ylim(-1, 3)
a[0][2].set_xlabel("Heading (°)")
a[0][2].set_title("Threshold: Positional error (m)")
a[1][0].plot(np.degrees(headingoffsets), accum_error[:, 0], color = "red", alpha = 0.3)
a[1][0].plot(np.degrees(headingoffsets), accum_error_latency[:, 0], color = "red")
a[1][0].set_ylabel("Lateral positional error (m)")
a[1][0].set_xlabel("Heading (°)")
a[1][0].axes.yaxis.set_ticks([])
a[1][0].set_xlim(0.5, 2)
a[1][0].set_title(r"Accumulator: $\alpha\, (\degree)$")
a[1][1].plot(np.degrees(headingoffsets), accum_error[:, 1], color = "red", alpha = 0.3)
a[1][1].plot(np.degrees(headingoffsets), accum_error_latency[:, 1], color = "red")
a[1][1].set_xlabel("Heading (°)")
a[1][1].axes.yaxis.set_ticks([])
a[1][1].set_xlim(0.5, 2)
a[1][1].set_title(r"Accumulator: $\alpha + \dot{\alpha}\, (\degree + \degree/s)$")
a[1][2].plot(np.degrees(headingoffsets), accum_error[:, 2], color = "red", alpha = 0.3)
a[1][2].plot(np.degrees(headingoffsets), accum_error_latency[:, 2], color = "red")
a[1][2].set_xlabel("Heading (°)")
a[1][2].axes.yaxis.set_ticks([])
a[1][2].set_xlim(0.5, 2)
a[1][2].set_title("Accumulator: Positional error (m)")
fig.savefig('ThresVAccum_Perceptual_Variables_LPE.png', dpi = 300)



## combined plot
fig, a =  plt.subplots(4,3, figsize = [14,20])
a[0][0].text(0.7, 1.85, "A", fontweight ="bold", fontsize = 50)
a[0][0].plot(np.degrees(headingoffsets), threshold_response[:, 0], color = "blue", alpha = 0.3)
a[0][0].plot(np.degrees(headingoffsets), threshold_response_latency[:, 0], color = "blue")
a[0][0].axes.yaxis.set_ticks([])
a[0][0].set_xlim(0.5, 2)
a[0][0].set_ylabel("RT (s)")
a[0][0].set_xlabel("Heading (°)")
a[0][0].set_title(r"Threshold: $\alpha\, (\degree)$")
a[0][1].plot(np.degrees(headingoffsets), threshold_response[:, 1], color = "blue", alpha = 0.3)
a[0][1].plot(np.degrees(headingoffsets), threshold_response_latency[:, 1], color = "blue")
a[0][1].axes.yaxis.set_ticks([])
a[0][1].set_xlim(0.5, 2)
a[0][1].set_xlabel("Heading (°)")
a[0][1].set_title(r"Threshold: $\alpha + \dot{\alpha}\, (\degree + \degree/s)$")
a[0][2].plot(np.degrees(headingoffsets), threshold_response[:, 2], color = "blue", alpha = 0.3)
a[0][2].plot(np.degrees(headingoffsets), threshold_response_latency[:, 2], color = "blue")
a[0][2].axes.yaxis.set_ticks([])
a[0][2].set_xlim(0.5, 2)
a[0][2].set_xlabel("Heading (°)")
a[0][2].set_title("Threshold: Positional error (m)")
a[1][0].plot(np.degrees(headingoffsets), accum_response[:, 0], color = "red", alpha = 0.3)
a[1][0].plot(np.degrees(headingoffsets), accum_response_latency[:, 0], color = "red")
a[1][0].set_ylabel("RT (s)")
a[1][0].set_xlabel("Heading (°)")
a[1][0].axes.yaxis.set_ticks([])
a[1][0].set_xlim(0.5, 2)
a[1][0].set_title(r"Accumulator: $\alpha\, (\degree)$")
a[1][1].plot(np.degrees(headingoffsets), accum_response[:, 1], color = "red", alpha = 0.3)
a[1][1].plot(np.degrees(headingoffsets), accum_response_latency[:, 1], color = "red")
a[1][1].set_xlabel("Heading (°)")
a[1][1].axes.yaxis.set_ticks([])
a[1][1].set_xlim(0.5, 2)
a[1][1].set_title(r"Accumulator: $\alpha + \dot{\alpha}\, (\degree + \degree/s)$")
a[1][2].plot(np.degrees(headingoffsets), accum_response[:, 2], color = "red", alpha = 0.3)
a[1][2].plot(np.degrees(headingoffsets), accum_response_latency[:, 2], color = "red")
a[1][2].set_xlabel("Heading (°)")
a[1][2].axes.yaxis.set_ticks([])
a[1][2].set_xlim(0.5, 2)
a[1][2].set_title("Accumulator: Positional error (m)")
a[2][0].text(0.7, 0.4, "B", fontweight ="bold", fontsize = 50)
a[2][0].plot(np.degrees(headingoffsets), threshold_error[:, 0], color = "blue", alpha = 0.3)
a[2][0].plot(np.degrees(headingoffsets), threshold_error_latency[:, 0], color = "blue")
a[2][0].axes.yaxis.set_ticks([])
a[2][0].set_xlim(0.5, 2)
a[2][0].set_ylim(0, 0.50)
a[2][0].set_ylabel("Lateral positional error (m)")
a[2][0].set_xlabel("Heading (°)")
a[2][0].set_title(r"Threshold: $\alpha\, (\degree)$")
a[2][1].plot(np.degrees(headingoffsets), threshold_error[:, 1], color = "blue", alpha = 0.3)
a[2][1].plot(np.degrees(headingoffsets), threshold_error_latency[:, 1], color = "blue")
a[2][1].axes.yaxis.set_ticks([])
a[2][1].set_xlim(0.5, 2)
a[2][1].set_ylim(0, 0.20)
a[2][1].set_xlabel("Heading (°)")
a[2][1].set_title(r"Threshold: $\alpha + \dot{\alpha}\, (\degree + \degree/s)$")
a[2][2].plot(np.degrees(headingoffsets), threshold_error[:, 2], color = "blue", alpha = 0.3)
a[2][2].plot(np.degrees(headingoffsets), threshold_error_latency[:, 2], color = "blue")
a[2][2].axes.yaxis.set_ticks([])
a[2][2].set_xlim(0.5, 2)
a[2][2].set_ylim(-1, 3)
a[2][2].set_xlabel("Heading (°)")
a[2][2].set_title("Threshold: Positional error (m)")
a[3][0].plot(np.degrees(headingoffsets), accum_error[:, 0], color = "red", alpha = 0.3)
a[3][0].plot(np.degrees(headingoffsets), accum_error_latency[:, 0], color = "red")
a[3][0].set_ylabel("Lateral positional error (m)")
a[3][0].set_xlabel("Heading (°)")
a[3][0].axes.yaxis.set_ticks([])
a[3][0].set_xlim(0.5, 2)
a[3][0].set_title(r"Accumulator: $\alpha\, (\degree)$")
a[3][1].plot(np.degrees(headingoffsets), accum_error[:, 1], color = "red", alpha = 0.3)
a[3][1].plot(np.degrees(headingoffsets), accum_error_latency[:, 1], color = "red")
a[3][1].set_xlabel("Heading (°)")
a[3][1].axes.yaxis.set_ticks([])
a[3][1].set_xlim(0.5, 2)
a[3][1].set_title(r"Accumulator: $\alpha + \dot{\alpha}\, (\degree + \degree/s)$")
a[3][2].plot(np.degrees(headingoffsets), accum_error[:, 2], color = "red", alpha = 0.3)
a[3][2].plot(np.degrees(headingoffsets), accum_error_latency[:, 2], color = "red")
a[3][2].set_xlabel("Heading (°)")
a[3][2].axes.yaxis.set_ticks([])
a[3][2].set_xlim(0.5, 2)
a[3][2].set_title("Accumulator: Positional error (m)") 
fig.savefig('ThresVAccum_Perceptual_Variables_combined.png', dpi = 300)

## Data saving response error
threshold_error_latency = pd.DataFrame(threshold_error_latency)
threshold_error_latency.columns = ['alpha', 'alphadot', 'LPE']
np.savetxt("threshold_error_latency.csv", threshold_error_latency, delimiter = ",")

accum_error_latency = pd.DataFrame(accum_error_latency)
accum_error_latency.columns = ['alpha', 'alphadot', 'LPE']
np.savetxt("accum_error_latency.csv", accum_error_latency, delimiter = ",")


# Data saving response time
threshold_response_latency = pd.DataFrame(threshold_response_latency)
threshold_response_latency.columns = ['alpha', 'alphadot', 'LPE']
np.savetxt("threshold_response_latency.csv", threshold_response_latency, delimiter = ",")

accum_response_latency = pd.DataFrame(accum_response_latency)
accum_response_latency.columns = ['alpha', 'alphadot', 'LPE']
np.savetxt("accum_response_latency.csv", accum_response_latency, delimiter = ",")


# plots for EXPERIMENT 2 (additional starting position levels)
headingoffsets = np.deg2rad(np.linspace(1,2,2))
simResults = simulate_scenarios(headingoffsets, 8)
pred_thres_latency, pred_accum_latency = predict_response(simResults, startingpos = [0, 0.5, 1], ego = 8)

# save threshold and accum RESPONSE TIME sim preds
threshold_response_latency = pred_thres_latency[0][:, 0:3]
accum_response_latency = pred_accum_latency[0][:, 0:3]

fig, a =  plt.subplots(2,2, figsize = [14,10])
a[0][0].plot(np.degrees(headingoffsets), threshold_response_latency[:, 0][:, 0], linestyle = ':', color = "blue")
a[0][0].plot(np.degrees(headingoffsets), threshold_response_latency[:, 1][:, 0], linestyle = '--', color = "blue")
a[0][0].plot(np.degrees(headingoffsets), threshold_response_latency[:, 2][:, 0], linestyle = '-', color = "blue")
a[0][0].set_xlim(1, 2)
a[0][0].set_ylabel("RT (s)")
a[0][0].set_xlabel("Heading (°)")
a[0][0].set_title(r"Threshold: $\alpha\, (\degree)$")
a[0][1].plot(np.degrees(headingoffsets), threshold_response_latency[:, 0][:, 2], linestyle = ':', color = "blue")
a[0][1].plot(np.degrees(headingoffsets), threshold_response_latency[:, 1][:, 2], linestyle = '--', color = "blue")
a[0][1].plot(np.degrees(headingoffsets), threshold_response_latency[:, 2][:, 2], linestyle = '-', color = "blue")
a[0][1].set_xlim(1, 2)
a[0][1].set_xlabel("Heading (°)")
a[0][1].set_title("Threshold: Positional error (m)")
a[1][0].plot(np.degrees(headingoffsets), accum_response_latency[:, 0][:, 0], linestyle = ':', color = "red")
a[1][0].plot(np.degrees(headingoffsets), accum_response_latency[:, 1][:, 0], linestyle = '--', color = "red")
a[1][0].plot(np.degrees(headingoffsets), accum_response_latency[:, 2][:, 0], linestyle = '-', color = "red")
a[1][0].set_xlim(1, 2)
a[1][0].set_ylabel("RT (s)")
a[1][0].set_xlabel("Heading (°)")
a[1][0].set_title(r"Accumulator: $\alpha\, (\degree)$")
a[1][1].plot(np.degrees(headingoffsets), accum_response_latency[:, 0][:, 2], linestyle = ':', color = "red")
a[1][1].plot(np.degrees(headingoffsets), accum_response_latency[:, 1][:, 2], linestyle = '--', color = "red")
a[1][1].plot(np.degrees(headingoffsets), accum_response_latency[:, 2][:, 2], linestyle = '-', color = "red")
a[1][1].set_xlim(1, 2)
a[1][1].set_xlabel("Heading (°)")
a[1][1].set_title("Accumulator: Positional error (m)")


## Data saving for REACTION TIMES for threshold predictions
threshold_response_latency = np.transpose(threshold_response_latency)

"Visual angle"
rt_thres_vis_angle = threshold_response_latency[0]
rt_thres_vis_angle = pd.DataFrame(rt_thres_vis_angle)
rt_thres_vis_angle.columns = ['1', '2']
rt_thres_vis_angle['startingpos'] = ['0', '4', '8']
rt_thres_vis_angle['error'] = ['visual_angle', 'visual_angle', 'visual_angle']
rt_thres_vis_angle['model'] = ['threshold', 'threshold', 'threshold']
rt_thres_vis_angle = pd.melt(rt_thres_vis_angle, id_vars =  ['startingpos', 'error', 'model'], value_vars = ['1', '2'], var_name = 'heading', value_name = 'rt')

"visual angle + rate of change in visual angle"
rt_thres_alphadot = threshold_response_latency[1]
rt_thres_alphadot = pd.DataFrame(rt_thres_alphadot)
rt_thres_alphadot.columns = ['1', '2']
rt_thres_alphadot['startingpos'] = ['0', '4', '8']
rt_thres_alphadot['error'] = ['alphadot', 'alphadot', 'alphadot']
rt_thres_alphadot['model'] = ['threshold', 'threshold', 'threshold']
rt_thres_alphadot = pd.melt(rt_thres_alphadot, id_vars =  ['startingpos', 'error', 'model'], value_vars = ['1', '2'], var_name = 'heading', value_name = 'rt')

"saving visual angle + rate of change in visual angle"
rt_thres_alphadot.to_csv("thresh_alphadot_rt_exp2.csv")


"Lateral position error"
rt_thres_lat_pos = threshold_response_latency[2]
rt_thres_lat_pos = pd.DataFrame(rt_thres_lat_pos)
rt_thres_lat_pos.columns = ['1', '2']
rt_thres_lat_pos['startingpos'] = ['0', '4', '8']
rt_thres_lat_pos['error'] = ['lat_pos', 'lat_pos', 'lat_pos']
rt_thres_lat_pos['model'] = ['threshold', 'threshold', 'threshold']
rt_thres_lat_pos = pd.melt(rt_thres_lat_pos, id_vars =  ['startingpos', 'error', 'model'], value_vars = ['1', '2'], var_name = 'heading', value_name = 'rt')

dfs_t = [rt_thres_vis_angle, rt_thres_lat_pos, rt_thres_alphadot]

threshold_response = pd.concat(dfs_t)

threshold_response.to_csv("threshold_response_exp2.csv")

## Data saving for REACTION TIMES for accumulator predictions
accum_response_latency = np.transpose(accum_response_latency)

"Visual angle"
rt_accum_vis_angle = accum_response_latency[0]
rt_accum_vis_angle = pd.DataFrame(rt_accum_vis_angle)
rt_accum_vis_angle.columns = ['1', '2']
rt_accum_vis_angle['startingpos'] = ['0', '4', '8']
rt_accum_vis_angle['error'] = ['visual_angle', 'visual_angle', 'visual_angle']
rt_accum_vis_angle['model'] = ['accumulator', 'accumulator', 'accumulator']
rt_accum_vis_angle = pd.melt(rt_accum_vis_angle, id_vars =  ['startingpos', 'error', 'model'], value_vars = ['1', '2'], var_name = 'heading', value_name = 'rt')

"Visual angle + rate of change in visual angle"
rt_accum_alphadot = accum_response_latency[1]
rt_accum_alphadot = pd.DataFrame(rt_accum_alphadot)
rt_accum_alphadot.columns = ['1', '2']
rt_accum_alphadot['startingpos'] = ['0', '4', '8']
rt_accum_alphadot['error'] = ['alphadot', 'alphadot', 'alphadot']
rt_accum_alphadot['model'] = ['accumulator', 'accumulator', 'accumulator']
rt_accum_alphadot = pd.melt(rt_accum_alphadot, id_vars =  ['startingpos', 'error', 'model'], value_vars = ['1', '2'], var_name = 'heading', value_name = 'rt')

"saving visual angle + rate of change in visual angle"
rt_accum_alphadot.to_csv("accum_alphadot_rt_exp2.csv")

"Lateral position error"
rt_accum_lat_pos = accum_response_latency[2]
rt_accum_lat_pos = pd.DataFrame(rt_accum_lat_pos)
rt_accum_lat_pos.columns = ['1', '2']
rt_accum_lat_pos['startingpos'] = ['0', '4', '8']
rt_accum_lat_pos['error'] = ['lat_pos', 'lat_pos', 'lat_pos']
rt_accum_lat_pos['model'] = ['accumulator', 'accumulator', 'accumulator']
rt_accum_lat_pos = pd.melt(rt_accum_lat_pos, id_vars =  ['startingpos', 'error', 'model'], value_vars = ['1', '2'], var_name = 'heading', value_name = 'rt')

dfs_a = [rt_accum_vis_angle, rt_accum_lat_pos, rt_accum_alphadot]

accumulator_response = pd.concat(dfs_a)

accumulator_response.to_csv("accum_response_exp2.csv")

# save threshold and accum RESPONSE ERROR sim preds
threshold_error_latency = pred_thres_latency[1][:, 0:3]
accum_error_latency = pred_accum_latency[1][:, 0:3]

fig, a =  plt.subplots(2,2, figsize = [14,10])
a[0][0].plot(np.degrees(headingoffsets), threshold_error_latency[:, 0][:, 0], linestyle = ':', color = "blue")
a[0][0].plot(np.degrees(headingoffsets), threshold_error_latency[:, 1][:, 0], linestyle = '--', color = "blue")
a[0][0].plot(np.degrees(headingoffsets), threshold_error_latency[:, 2][:, 0], linestyle = '-', color = "blue")
a[0][0].set_xlim(1, 2)
a[0][0].set_ylim(0, 0.5)
a[0][0].set_ylabel("Lateral positional error (m)")
a[0][0].set_xlabel("Heading (°)")
a[0][0].set_title(r"Threshold: $\alpha\, (\degree)$")
a[0][1].plot(np.degrees(headingoffsets), threshold_error_latency[:, 0][:, 2] / 2, linestyle = ':', color = "blue")
a[0][1].plot(np.degrees(headingoffsets), threshold_error_latency[:, 1][:, 2], linestyle = '--', color = "blue")
a[0][1].plot(np.degrees(headingoffsets), threshold_error_latency[:, 2][:, 2], linestyle = '-', color = "blue")
a[0][1].set_xlim(1, 2)
a[0][1].set_xlabel("Heading (°)")
a[0][1].set_ylim(0, 0.5)
a[0][1].set_title("Threshold: Positional error (m)")
a[1][0].plot(np.degrees(headingoffsets), accum_error_latency[:, 0][:, 0], linestyle = ':', color = "red")
a[1][0].plot(np.degrees(headingoffsets), accum_error_latency[:, 1][:, 0], linestyle = '--', color = "red")
a[1][0].plot(np.degrees(headingoffsets), accum_error_latency[:, 2][:, 0], linestyle = '-', color = "red")
a[1][0].set_xlim(1, 2)
a[1][0].set_ylim(0, 0.5)
a[1][0].set_ylabel("Lateral positional error (m)")
a[1][0].set_xlabel("Heading (°)")
a[1][0].set_title(r"Accumulator: $\alpha\, (\degree)$")
a[1][1].plot(np.degrees(headingoffsets), accum_error_latency[:, 0][:, 2], linestyle = ':', color = "red")
a[1][1].plot(np.degrees(headingoffsets), accum_error_latency[:, 1][:, 2], linestyle = '--', color = "red")
a[1][1].plot(np.degrees(headingoffsets), accum_error_latency[:, 2][:, 2], linestyle = '-', color = "red")
a[1][1].set_xlim(1, 2)
a[1][1].set_ylim(0, 0.5)
a[1][1].set_xlabel("Heading (°)")
a[1][1].set_title("Threshold: Positional error (m)")

## Data saving for RESPONSE ERROR for threshold predictions

threshold_error_latency[:, 0][:, 2] = threshold_error_latency[:, 0][:, 2] / 2

threshold_error_latency = np.transpose(threshold_error_latency)

"Visual angle"
error_thres_vis_angle = threshold_error_latency[0]
error_thres_vis_angle = pd.DataFrame(error_thres_vis_angle)
error_thres_vis_angle.columns = ['1', '2']
error_thres_vis_angle['startingpos'] = ['8', '4', '0']
error_thres_vis_angle['error'] = ['visual_angle', 'visual_angle', 'visual_angle']
error_thres_vis_angle['model'] = ['threshold', 'threshold', 'threshold']
error_thres_vis_angle = pd.melt(error_thres_vis_angle, id_vars =  ['startingpos', 'error', 'model'], value_vars = ['1', '2'], var_name = 'heading', value_name = 'LPE')

"Visual angle + change in visual angle"
error_thres_alphadot = threshold_error_latency[1]
error_thres_alphadot = pd.DataFrame(error_thres_alphadot)
error_thres_alphadot.columns = ['1', '2']
error_thres_alphadot['startingpos'] = ['8', '4', '0']
error_thres_alphadot['error'] = ['alphadot', 'alphadot', 'alphadot']
error_thres_alphadot['model'] = ['threshold', 'threshold', 'threshold']
error_thres_alphadot = pd.melt(error_thres_alphadot, id_vars =  ['startingpos', 'error', 'model'], value_vars = ['1', '2'], var_name = 'heading', value_name = 'LPE')

"saving visual angle + rate of change in visual angle"
error_thres_alphadot.to_csv("thresh_alphadot_error_exp2.csv")

"Lateral position error"
error_thres_lat_pos = threshold_error_latency[2]
error_thres_lat_pos = pd.DataFrame(error_thres_lat_pos)
error_thres_lat_pos.columns = ['1', '2']
error_thres_lat_pos['startingpos'] = ['8', '4', '0']
error_thres_lat_pos['error'] = ['lat_pos', 'lat_pos', 'lat_pos']
error_thres_lat_pos['model'] = ['threshold', 'threshold', 'threshold']
error_thres_lat_pos = pd.melt(error_thres_lat_pos, id_vars =  ['startingpos', 'error', 'model'], value_vars = ['1', '2'], var_name = 'heading', value_name = 'LPE')

dfs_t = [error_thres_vis_angle, error_thres_lat_pos, error_thres_alphadot]

threshold_error = pd.concat(dfs_t)

threshold_error.to_csv("threshold_error_exp2.csv")

## Data saving for RESPONSE ERROR for accumulator predictions

accum_error_latency = np.transpose(accum_error_latency)

"Visual angle"
error_accum_vis_angle = accum_error_latency[0]
error_accum_vis_angle = pd.DataFrame(error_accum_vis_angle)
error_accum_vis_angle.columns = ['1', '2']
error_accum_vis_angle['startingpos'] = ['8', '4', '0']
error_accum_vis_angle['error'] = ['visual_angle', 'visual_angle', 'visual_angle']
error_accum_vis_angle['model'] = ['accumulator', 'accumulator', 'accumulator']
error_accum_vis_angle = pd.melt(error_accum_vis_angle, id_vars =  ['startingpos', 'error', 'model'], value_vars = ['1', '2'], var_name = 'heading', value_name = 'LPE')

"Visual angle + change in visual angle"
error_accum_alphadot = accum_error_latency[1]
error_accum_alphadot = pd.DataFrame(error_accum_alphadot)
error_accum_alphadot.columns = ['1', '2']
error_accum_alphadot['startingpos'] = ['8', '4', '0']
error_accum_alphadot['error'] = ['alphadot', 'alphadot', 'alphadot']
error_accum_alphadot['model'] = ['accumulator', 'accumulator', 'accumulator']
error_accum_alphadot = pd.melt(error_accum_alphadot, id_vars =  ['startingpos', 'error', 'model'], value_vars = ['1', '2'], var_name = 'heading', value_name = 'LPE')

"saving visual angle + rate of change in visual angle"
error_accum_alphadot.to_csv("accum_alphadot_error_exp2.csv")

"Lateral position error"
error_accum_lat_pos = accum_error_latency[2]
error_accum_lat_pos = pd.DataFrame(error_accum_lat_pos)
error_accum_lat_pos.columns = ['1', '2']
error_accum_lat_pos['startingpos'] = ['8', '4', '0']
error_accum_lat_pos['error'] = ['lat_pos', 'lat_pos', 'lat_pos']
error_accum_lat_pos['model'] = ['accumulator', 'accumulator', 'accumulator']
error_accum_lat_pos = pd.melt(error_accum_lat_pos, id_vars =  ['startingpos', 'error', 'model'], value_vars = ['1', '2'], var_name = 'heading', value_name = 'LPE')

dfs_a = [error_accum_vis_angle, error_accum_lat_pos, error_accum_alphadot]

accum_error = pd.concat(dfs_a)

accum_error.to_csv("accum_error_exp2.csv")










