import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation



class DataFile:
    def __init__(self,dataPath,fileName):

        self.fileName = fileName
        self.dataPath = dataPath
        filepath = self.dataPath+self.fileName

        self.trial_df = pd.read_csv(filepath)

        ## GET TRIAL INFO (REGEX IS LIKELY BETTER WAY TO DO THIS)
        idx = fileName.find('--Vz')
        self.agent = fileName[:idx]
        idx = fileName.find('trial_')
        self.trialNum = int(fileName[idx+6:idx+7])
        self.v_d = self.grab_vel_d()


        self.n_rollouts = int(self.trial_df.iloc[-1]['n_rollouts'])
        self.k_epMax = int(self.trial_df.iloc[-1]['k_ep'])
       

        ## WHAT DO THESE DO AGAIN?
        # trial_df = self.trial_df[ self.trial_df['Error'] != 'Error: NAN found in state vector'] 
        # self.trial_df = trial_df[ trial_df['vz'].notna()] # Drops all rows that vz is blank

        ## NOTE: Trial needs to be truncated to last complete episode
        

    def select_run(self,k_ep,k_run): ## Create run dataframe from k_ep and k_run
        run_df = self.trial_df[(self.trial_df['k_ep']==k_ep) & (self.trial_df['k_run']==k_run)]
        IC_df = run_df[-1:]                                     # Remove last row with Initial Conditions
        run_df = run_df[:-1]                                    # Drop IC row from dataframe
        run_df = run_df.replace({'False':False,'True':True})    # Change all string bools to normal bools

        return run_df,IC_df

    def grab_rewardData(self):
        ## CREATE ARRAYS FOR REWARD, K_EP 
        reward_df = self.trial_df.iloc[:][['k_ep','reward']].dropna() # Create df from k_ep/rewards and drop blank reward rows
        rewards_arr = reward_df.to_numpy()
        rewards = rewards_arr[:,1]
        k_ep_r = rewards_arr[:,0]

        ## CREATE ARRAYS FOR REWARD_AVG, K_EP
        rewardAvg_df = reward_df.groupby('k_ep').mean() # Group rows by k_ep value and take their mean 
        rewards_avg = rewardAvg_df.to_numpy()
        k_ep_ravg = reward_df.k_ep.unique() # Drops duplicate values so it matches dimension of rewards_avg (1 avg per ep and n ep)

        return k_ep_r,rewards,k_ep_ravg,rewards_avg

    def plot_rewardData(self):
        """Plot rewards for overall trial

        Args:
            figNum (int, optional): Figure Number. Defaults to 0.
        """        
        k_ep_r,rewards,k_ep_ravg,rewards_avg = self.grab_rewardData()

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.scatter(k_ep_r,rewards,marker='_',color='black',alpha=0.5,label='Reward')
        ax.scatter(k_ep_ravg,rewards_avg,marker='o',color='red',label='Reward_avg')
        
        

        ax.set_ylabel("Rewards")
        ax.set_xlabel("k_ep")
        ax.set_xlim(-2,self.k_epMax+5)
        ax.set_ylim(-2,150)
        ax.set_title(f"Reward vs Episode | Rollouts per Episode: {self.n_rollouts}")
        ax.legend()
        ax.grid()

        plt.show()

    def landing_plot(self): ## FUNCTIONAL BUT NOT PRETTY OR USEFUL
        impact_df = self.trial_df.iloc[:][['k_ep','reward','impact_flag']].dropna() # Use reward to select final impact row
        impact_df['impact_flag'] = pd.to_numeric(impact_df['impact_flag'])          # Convert number of legs (str) to type (int)
        impact_df = impact_df.replace(3,4) # 3 and 4 legs are equivalent so just replace them

        ## CONVERT DF COLUMNS TO NUMPY ARRAYS
        k_ep_arr = impact_df['k_ep'].to_numpy()
        landing_arr = impact_df['impact_flag'].to_numpy()



        ## PLOT LANDING DATA
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
        ax.scatter(k_ep_arr,landing_arr,marker='_',color='black',alpha=0.5,label='Reward')

        ax.set_ylabel("Landing Legs")
        ax.set_xlabel("time [s]")
        ax.legend()
        ax.grid()

        plt.show()

    def landing_rate(self):
        impact_df = self.trial_df.iloc[:][['k_ep','reward','flip_flag','impact_flag']].dropna() # Use reward to select final impact row
        impact_df['impact_flag'] = pd.to_numeric(impact_df['impact_flag'])          # Convert number of legs (str) to type (int)

        temp = impact_df.iloc[-int(self.n_rollouts*3):][['flip_flag','impact_flag']] # Grab leg impact number and body impact flag for final 3 episodes

        
        # Trim rows to match conditions and find final number (flip_flag IC row shows body impact on True)
        landings = temp[(temp.impact_flag >= 3) & (temp.flip_flag == False)].shape[0]
        attempts = self.n_rollouts*3
        landingRate = landings/attempts
        

        return landingRate

    def plotSummary(self):
        fig = plt.figure(figsize=(12,6))

        ## REWARD DATA PLOT
        #region
        k_ep_r,rewards,k_ep_ravg,rewards_avg = self.grab_rewardData()
        

        ax1 = fig.add_subplot(221)
        ax1.scatter(k_ep_r,rewards,marker='_',color='black',alpha=0.5,label='Reward')
        ax1.scatter(k_ep_ravg,rewards_avg,marker='o',color='red',label='Reward_avg')
        
        ax1.set_ylabel("Rewards")
        ax1.set_xlabel("k_ep")
        ax1.set_xlim(-2,self.k_epMax+5)
        ax1.set_ylim(-2,150)
        ax1.set_title(f"Reward vs Episode | Rollouts per Episode: {self.n_rollouts}")
        ax1.legend()
        ax1.grid()
        #endregion

        ## MU DATA PLOT
        #region
        k_ep_arr,mu_arr,sigma_arr = self.grab_convg_data()
        ax2 = fig.add_subplot(222)

        num_col = mu_arr.shape[1] # Number of policy gains in mu [Currently 3]
        G_Labels = ['RREV_trigger','G1','G2','G3','G4','G5'] # List of policy gain names
        for jj in range(num_col): # Iterate through gains and plot each
            ax2.plot(k_ep_arr,mu_arr[:,jj],label=G_Labels[jj])

        ax2.set_ylabel('Policy Values')
        ax2.set_xlabel('K_ep')
        ax2.set_ylim(0,15) # Set lower ylim
        ax2.set_xlim(-1,self.k_epMax+1)
        ax2.set_title(self.fileName)
        ax2.legend(loc='upper center',ncol=num_col)
        ax2.grid()
        #endregion

        ## SIGMA DATA PLOT
        #region
        ax4 = fig.add_subplot(224)
        for jj in range(num_col): # Iterate through gains and plot each
            ax4.plot(k_ep_arr,sigma_arr[:,jj],label=G_Labels[jj])

        ax4.set_ylabel('Standard Deviation')
        ax4.set_xlabel('K_ep')
        ax4.set_ylim(0,4)
        ax4.set_xlim(-1,self.k_epMax+1)
        ax4.legend(ncol=3,loc='upper right')
        ax4.grid()
        #endregion

        ## GENERAL DATA
        landingRate = self.landing_rate()
        ax3 = fig.add_subplot(223)
        ax3.text(0.2,0.5,f'Landing Rate: {landingRate:.2f} \n',size=12)

        fig.tight_layout()
        plt.show()
        
    def landing_bool(self,k_ep,k_run):

        ## SELECT RUN DF
        run_df,IC_df = self.select_run(k_ep,k_run)
        n_legs = pd.to_numeric(IC_df.iloc[0]['impact_flag'])
        body_contact = IC_df.iloc[0]['flip_flag']

        ## CHECK FOR NO BODY CONTACT 
        if body_contact == False:
            
            # CHECK FOR 3-4 LEGS CONTACTING
            if n_legs >= 3:
                landingBool = True
            else:
                landingBool = False

        else:
            landingBool = False

        return landingBool

## POLICY FUNCTIONS
    def grab_policy(self,k_ep,k_run):
        """Returns policy from specific run

        Args:
            k_ep (int): Episode number
            k_run (int): Run number

        Returns:
            np.array: [RREV,G1,G2,...]
        """        
        run_df,IC_df = self.select_run(k_ep,k_run)

        ## SELECT POLICY
        policy = IC_df.iloc[0]['policy']
        policy = np.fromstring(policy[1:-1], dtype=float, sep=' ')  # Convert str to np array

        return policy

    def grab_convg_data(self):
        ## CLEAN AND GRAB DATA FOR MU & SIGMA
        policy_df = self.trial_df.iloc[:][['k_ep','mu','sigma']]
        policy_df = policy_df.dropna().drop_duplicates()
        k_ep_arr = policy_df.iloc[:]['k_ep'].to_numpy()

        

        ## CREATE NP ARRAYS FOR MU & SIGMA OVER TRIAL
        mu_arr = []
        for mu in policy_df.iloc[:]['mu']:
            mu = np.fromstring(mu[1:-1], dtype=float, sep=' ')
            mu_arr.append(mu)
        mu_arr = np.array(mu_arr)

        sigma_arr = []
        for sigma in policy_df.iloc[:]['sigma']:
            sigma = np.fromstring(sigma[1:-1], dtype=float, sep=' ')
            sigma_arr.append(sigma)
        sigma_arr = np.array(sigma_arr)

        return k_ep_arr,mu_arr,sigma_arr

    def plot_policy_convg(self,trialNum=np.nan): ## NEEDS UPDATED
        """Creates subplots to show convergence for policy gains

        Args:
            trialNum ([int], optional): Display trial number. Defaults to np.nan.
        """      

        ## GRAB CONVERGENCE DATA
        k_ep_arr,mu_arr,sigma_arr = self.grab_convg_data()

        

        num_col = mu_arr.shape[1] # Number of policy gains in mu [Currently 3]
        G_Labels = ['RREV_trigger','G1','G2','G3','G4','G5'] # List of policy gain names
        vx,vy,vz = self.grab_vel_d()


        ## CREATE SUBPLOT FOR MU 
        fig = plt.figure()
        ax = fig.add_subplot(211)
        for jj in range(num_col): # Iterate through gains and plot each
            ax.plot(k_ep_arr,mu_arr[:,jj],label=G_Labels[jj])


        ax.set_ylabel('Policy Values')
        ax.set_xlabel('K_ep')
        ax.set_ylim(0) # Set lower ylim
        ax.set_title(f'Policy Value vs Episode (Trial:{trialNum}) | Vx = {vx} Vz = {vz}')
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.25),ncol=num_col)
        ax.grid()
        fig.tight_layout()


        ## CREATE SUBPLOT FOR SIGMA
        ax = fig.add_subplot(212)
        for jj in range(num_col): # Iterate through gains and plot each
            ax.plot(k_ep_arr,sigma_arr[:,jj],label=G_Labels[jj])

        ax.set_ylabel('Standard Deviation')
        ax.set_xlabel('K_ep')
        ax.set_title(f'Policy S.D. vs Episode (Trial:{trialNum}) | Vx = {vx} Vz = {vz}')
        ax.legend(ncol=3,loc='upper right')
        ax.grid()

        fig.tight_layout()
        plt.show()

    def grab_finalPolicy(self):
        """Returns the final policy for a trial

        Returns:
           mu [np.array]: Final policy average
           sigma [np.array]: Final policy standard deviation

        """        
        
        ## FIND FINAL RUN DF
        k_ep = self.trial_df.iloc[-1]['k_ep']
        k_run = self.trial_df.iloc[-1]['k_run']
        run_df,IC_df = self.select_run(k_ep,k_run)

        ## SELECT MU & SIGMA
        mu = IC_df.iloc[0]['mu']
        mu = np.fromstring(mu[1:-1], dtype=float, sep=' ')  # Convert str to np array
        

        sigma = IC_df.iloc[0]['sigma']
        sigma = np.fromstring(sigma[1:-1], dtype=float, sep=' ')
        

        return mu,sigma



## STATE FUNCTIONS 
    def grab_stateData(self,k_ep,k_run,stateName):
        """Returns np.array of specified state

        Args:
            k_ep (int): Episode number
            k_run (int): Run number
            stateName (int): State name

        Returns:
            state [np.array]: Returned state values
        """        
        run_df,IC_df = self.select_run(k_ep,k_run)

        ## GRAB STATE DATA AND CONVERT TO NUMPY ARRAY
        state = run_df.iloc[:][stateName]
        state = state.to_numpy()
        state = np.expand_dims(state, axis=0).T

        return state

    def plot_state(self,k_ep,k_run,stateName,figNum=0):
        """Plot state data from run vs time

        Args:
            k_ep (int): Ep. Num
            k_run (int): Run. Num
            stateName (str): State name from .csv
            figNum (int, optional): Figure Number. Defaults to 0.
        """        

        ## GRAB STATE/TIME DATA
        state = self.grab_stateData(k_ep,k_run,stateName)
        t = self.grab_stateData(k_ep,k_run,'t')
        t_norm = t - np.min(t) # Normalize time

        
        t_flip,t_flip_norm = self.grab_flip_time(k_ep,k_run)
        state_flip = self.grab_flip_state(k_ep,k_run,stateName)

        t_impact,t_impact_norm,body_impact = self.grab_impact_time(k_ep,k_run)
        state_impact = self.grab_impact_state(k_ep,k_run,stateName)


        
        ## PLOT STATE/TIME DATA
        fig = plt.figure(figNum)
        ax = fig.add_subplot(111)
        
        ax.plot(t_norm,state,label=f"{stateName}",zorder=1)
        ax.scatter(t_flip_norm,state_flip,label="Flip",zorder=2)

        if body_impact==True:
            ax.scatter(t_impact_norm,state_impact,label="Body Impact",zorder=2)
        else:
            ax.scatter(t_impact_norm,state_impact,label="Leg Impact",zorder=2)


        ax.set_ylabel(f"{stateName}")
        ax.set_xlabel("time [s]")
        ax.legend()
        ax.grid()

        plt.show()


    def grab_eulerData(self,k_ep,k_run,degrees=True):
        
        ## CREATE QUAT DATAFRAME FROM RUN_DF
        run_df,IC_df = self.select_run(k_ep,k_run)
        quat_df = run_df.iloc[:][['t','qw','qx','qy','qz']]
        

        ## CONVERT QUATS TO EULER ANGLES AND BACK TO DF
        quat_arr = quat_df[['qx','qy','qz','qw']].to_numpy()
        R = Rotation.from_quat(quat_arr)
        eul_arr = R.as_euler('YZX', degrees=degrees)
        # Center axis [theta_2] is limited to +- 90deg while [theta_1 & theta_3] can rotate to 180deg 
        # https://graphics.fandom.com/wiki/Conversion_between_quaternions_and_Euler_angles
        # https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/Quaternions.pdf
        eul_df = pd.DataFrame(eul_arr,columns=['eul_y','eul_z','eul_x'])

        ## IF EUL_Y JUMPS FROM PAST -180 T0 +180 THEN BRING IT BACK DOWN AGAIN
        eul_df.loc[eul_df['eul_y'] > 170, 'eul_y'] = eul_df['eul_y']-360

        eul_x = eul_df['eul_x']
        eul_y = eul_df['eul_y']
        eul_z = eul_df['eul_z']

        return [eul_x,eul_y,eul_z]

    def plot_eulerData(self,k_ep,k_run,eul_type):
        
        if eul_type == 'eul_x':
            eul = self.grab_eulerData(k_ep,k_run)[0]
        elif eul_type == 'eul_y':
            eul = self.grab_eulerData(k_ep,k_run)[1]
        elif eul_type == 'eul_z':
            eul = self.grab_eulerData(k_ep,k_run)[2]
        else:
            print('Error, please select: ("eul_x","eul_y", or "eul_z")')
            eul = np.nan


        t = self.grab_stateData(k_ep,k_run,'t')
        t_norm = t - np.min(t) # Normalize time

        t_norm_impact = self.grab_impact_time(k_ep,k_run)[1]
        eul_impact = self.grab_impact_eul(k_ep,k_run,eul_type)

        t_norm_flip = self.grab_flip_time(k_ep, k_run)[1]
        eul_flip = self.grab_flip_eul(k_ep,k_run,eul_type)
        

        
        ## PLOT STATE/TIME DATA
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(t_norm,eul,label=f"{eul_type}")
        ax.scatter(t_norm_flip,eul_flip,label="Flip")
        ax.scatter(t_norm_impact,eul_impact,label="Impact")
        


        ax.set_ylabel(f"{eul_type} [deg]")
        ax.set_xlabel("time [s]")
        ax.legend()
        ax.grid()

        plt.show()


    def plot_traj(self,k_ep,k_run):
        fig = plt.figure()
        ax = fig.add_subplot(111)

        x = self.grab_stateData(k_ep,k_run,'x')
        z = self.grab_stateData(k_ep,k_run,'z')
        

        ax.plot(x, z)
        ax.legend()
        ax.set_xlabel("x-pos [m]")
        ax.set_ylabel("z-pos [m]")
        

        ax.set_xlim([-1,3])
        ax.set_ylim([0,3])
        ax.grid()
        
        

        plt.show()


    def plot_traj2(self,k_ep,k_run):
        fig = plt.figure()
        ax = fig.add_subplot(111)

        x = self.grab_stateData(k_ep,k_run,'x')
        z = self.grab_stateData(k_ep,k_run,'z')

        eul_y = self.grab_eulerData(k_ep,k_run,'eul_y')[1]
        

        ax.quiver(x,z,0.5*np.cos(eul_y),0.5*np.sin(eul_y))
        ax.plot(x,z)
        
        # ax.legend()
        ax.set_xlabel("x-pos [m]")
        ax.set_ylabel("z-pos [m]")
        

        ax.set_xlim([-1,3])
        ax.set_ylim([0,3])
        ax.grid()
        
        

        plt.show()
    
    
    def plot_traj_3D(self,k_ep,k_run):
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        x = self.grab_stateData(k_ep,k_run,'x').flatten()
        y = self.grab_stateData(k_ep,k_run,'y').flatten()
        z = self.grab_stateData(k_ep,k_run,'z').flatten()

        ax.plot(x, y, z, label='parametric curve')
        ax.legend()
        ax.set_xlabel("x-pos [m]")
        ax.set_ylabel("y-pos [m]")
        ax.set_zlabel("z-pos [m]")

        ax.set_xlim([-1,3])
        ax.set_ylim([-1,1])
        ax.set_zlim([0,3])
        

        plt.show()




## DESIRED IC FUNCTIONS
    def grab_vel_d(self):
    
        """Return IC values

        Returns:
            float: Initial Velocity Conditions (vx,vy,vz)
        """        
        vx_IC = self.trial_df.iloc[-1]['vx']
        vy_IC = self.trial_df.iloc[-1]['vy']
        vz_IC = self.trial_df.iloc[-1]['vz']
        
        return vx_IC,vy_IC,vz_IC

    def grab_My_d(self,k_ep,k_run):
        """Returns desired M_d as list
        Args:
            k_ep (int): Episode number
            k_run (int): Run number

        Returns:
            Float: My_d
        """        
        run_df,IC_df = self.select_run(k_ep,k_run)


        My_d = IC_df.iloc[0]['My']


        return My_d

    def grab_My_d_trial(self):           

        ## CREATE ARRAY OF ALL EP/RUN COMBINATIONS FROM LAST 3 ROLLOUTS
        ep_df = self.trial_df.iloc[:][['k_ep','k_run']].drop_duplicates()
        ep_arr = ep_df.iloc[-self.n_rollouts*3:].to_numpy() # Grab episode/run listing from past 3 rollouts

        ## ITERATE THROUGH ALL RUNS AND FIND My_d FOR SUCCESSFUL LANDINGS
        My_dList = []
        epList = []
        for k_ep,k_run in ep_arr:
            if self.landing_bool(k_ep, k_run): # IGNORE FAILED LANDINGS
                My_dList.append(self.grab_My_d(k_ep,k_run))
                epList.append(k_ep)
                
        ## CONVERT LIST TO NP ARRAY AND CALC MEAN
        arr = np.asarray(My_dList)
        avg_My_d = np.mean(arr,axis=0)

        return avg_My_d

    def plot_My_d_trial(self):
        ## CREATE ARRAY OF ALL EP/RUN COMBINATIONS FROM LAST 3 ROLLOUTS
        ep_df = self.trial_df.iloc[:][['k_ep','k_run']].drop_duplicates()
        ep_arr = ep_df.iloc[:].to_numpy() # Grab episode/run listing from past 3 rollouts

        ## ITERATE THROUGH ALL RUNS AND FIND My_d FOR SUCCESSFUL LANDINGS
        My_dList = []
        epList = []
        for k_ep,k_run in ep_arr:
            if self.landing_bool(k_ep, k_run): # IGNORE FAILED LANDINGS
                My_dList.append(self.grab_My_d(k_ep,k_run))
                epList.append(k_ep)
                
        ## CONVERT LIST TO NP ARRAY AND CALC MEAN
        arr = np.asarray(My_dList)
        avg_My_d = self.grab_My_d_trial()
        

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.scatter(epList,My_dList,marker="_",color="black",label="4-Leg Landings")
        ax.hlines(avg_My_d,0,self.k_epMax,label="Avg My_d Final Landings") # SHOW AVG IMPACT ANGLE ACROSS TRIAL
        ax.hlines(-7.77,0,self.k_epMax,color="red",label="Max Moment Limit")
        

        
        ax.set_xlim(-1,20)
        ax.set_ylim(-0,-10)
        
        ax.set_xticks(range(0,self.k_epMax+1,5))

        ax.set_title("4 Leg Landings - My_d")
        ax.set_xlabel("Episode Number")
        ax.set_ylabel("My_d [N*mm]")

        ax.grid()
        ax.legend()

        plt.show()









    def grab_flip_time(self,k_ep,k_run):
        """Returns time of flip

        Args:
            k_ep (int): Episode number
            k_run (int): Run number

        Returns:
            [float,float]: [t_flip,t_flip_norm]
        """        
        run_df,IC_df = self.select_run(k_ep,k_run)
        t_flip = run_df.query(f"flip_flag=={True}").iloc[0]['t']
        t_flip_norm = t_flip - run_df.iloc[0]['t']

        return t_flip,t_flip_norm

    def grab_flip_state(self,k_ep,k_run,stateName):
        """Returns desired state at time of flip

        Args:
            k_ep (int): Episode number
            k_run (int): Run number
            stateName (str): State name

        Returns:
            float: state_flip
        """        
        run_df,IC_df = self.select_run(k_ep,k_run)
        state_flip = run_df.query(f"flip_flag=={True}").iloc[0][stateName]

        return state_flip

    def grab_flip_eul(self,k_ep,k_run,eul_type):
        ## GRAB RUN DF AND RUN QUERY FOR QUAT AT FIRST IMPACT
        run_df,IC_df = self.select_run(k_ep,k_run)
        quat_flip = run_df.query(f"flip_flag=={True}").iloc[0][['qw','qx','qy','qz']]

        ## CONVERT QUAT TO EULER ANGLE
        quat_arr = quat_flip[['qx','qy','qz','qw']].to_numpy()
        R = Rotation.from_quat(quat_arr)
        eul_arr = R.as_euler('YZX', degrees=True)

        ## RETURN EULER IMPACT ANGLE
        if eul_type == 'eul_y':
            return eul_arr[0]
        elif eul_type == 'eul_z':
            return eul_arr[1]
        elif eul_type == 'eul_x':
            return eul_arr[2]
        else:
            return np.nan

    




    def grab_impact_time(self,k_ep,k_run):
        """Returns time of impact of body/legs

        Args:
            k_ep (int): Episode number
            k_run (int): Run number

        Returns:
            [float,float,bool]: [t_impact,t_impact_norm,body_impact]
        """        
        run_df,IC_df = self.select_run(k_ep,k_run)
        t_impact = run_df.query(f"impact_flag=={True}").iloc[0]['t']    # Grab first t value in df filtered to where flag == True
        t_impact_norm = t_impact - run_df.iloc[0]['t']                  # Normalize time to zero

        body_impact = IC_df.iloc[0]['flip_flag'] # Reads value if body impacted the ceiling

        return t_impact,t_impact_norm,body_impact

    def grab_impact_state(self,k_ep,k_run,stateName):
        """Returns state at time of impact

        Args:
            k_ep (int): Episode number
            k_run (int): Run number
            stateName (str): State name

        Returns:
            float: state_impact
        """        

        run_df,IC_df = self.select_run(k_ep,k_run)
        state_impact = run_df.query(f"impact_flag=={True}").iloc[0][stateName]    # Grab first t value in df filtered to where flag == True
        
        return state_impact

    def grab_impact_eul(self,k_ep,k_run,eul_type):
        ## GRAB RUN DF AND RUN QUERY FOR QUAT AT FIRST IMPACT
        run_df,IC_df = self.select_run(k_ep,k_run)
        quat_impact = run_df.query(f"impact_flag=={True}").iloc[0][['qw','qx','qy','qz']]

        ## CONVERT QUAT TO EULER ANGLE
        quat_arr = quat_impact[['qx','qy','qz','qw']].to_numpy()
        R = Rotation.from_quat(quat_arr)
        eul_arr = R.as_euler('YZX', degrees=True)

        

        ## RETURN EULER IMPACT ANGLE
        if eul_type == 'eul_y':
            eul =  eul_arr[0]
            if eul > 170:
                eul = eul - 360
                
        elif eul_type == 'eul_z':
            eul =  eul_arr[1]
        elif eul_type == 'eul_x':
            eul = eul_arr[2]
        else:
            eul = np.nan

        return eul

    def grab_impact_eul_trial(self,eul_type):

        ## CREATE ARRAY OF ALL EP/RUN COMBINATIONS FROM LAST 3 ROLLOUTS
        ep_df = self.trial_df.iloc[:][['k_ep','k_run']].drop_duplicates()
        ep_arr = ep_df.iloc[-self.n_rollouts*3:].to_numpy() # Grab episode/run listing from past 3 rollouts

        ## ITERATE THROUGH ALL RUNS AND FINDING IMPACT ANGLE 
        list = []
        for k_ep,k_run in ep_arr:
            if self.landing_bool(k_ep, k_run): # IGNORE FAILED LANDINGS
                list.append(self.grab_impact_eul(k_ep,k_run,eul_type))

        ## CONVERT LIST TO NP ARRAY AND CALC MEAN
        arr = np.asarray(list)
        avg_impact = np.mean(arr,axis=0)
        
        return avg_impact
    
    def plot_impact_eul_trial(self,eul_type):

        ## CREATE ARRAY OF ALL EP/RUN COMBINATIONS FROM LAST 3 ROLLOUTS
        ep_df = self.trial_df.iloc[:][['k_ep','k_run']].drop_duplicates()
        ep_arr = ep_df.iloc[:].to_numpy() # Grab episode/run listing from past 3 rollouts

        ## ITERATE THROUGH ALL RUNS AND FINDING IMPACT ANGLE 
        impactList = []
        epList = []
        for k_ep,k_run in ep_arr:
            if self.landing_bool(k_ep, k_run): # IGNORE FAILED LANDINGS
                impactList.append(self.grab_impact_eul(k_ep,k_run,eul_type))
                epList.append(k_ep)


        ## CONVERT LIST TO NP ARRAY AND CALC MEAN
        arr = np.asarray(impactList)
        avg_impact = np.mean(arr,axis=0)
        

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.scatter(epList,impactList,marker="_",color="black",label="4-Leg Landings")
        ax.hlines(avg_impact,0,20,label='Avg Impact Angle Final Landings') # SHOW AVG IMPACT ANGLE ACROSS TRIAL
        

        
        ax.set_xlim(-1,self.k_epMax)
        ax.set_ylim(-220,0)
        ax.set_xticks(range(0,self.k_epMax,5))

        ax.set_title("4 Leg Landings - Impact Angles")
        ax.set_xlabel("Episode Number")
        ax.set_ylabel("Impact Angle [deg]")

        ax.grid()
        ax.legend()
        plt.show()
        




