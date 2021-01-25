import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation



class DataFile:
    def __init__(self,filepath):
        self.trial_df = pd.read_csv(filepath)
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

    def plot_rewardFunc(self,figNum=0):
        """Plot rewards for overall trial

        Args:
            figNum (int, optional): Figure Number. Defaults to 0.
        """        
        num_rollouts = self.trial_df.iloc[-1]['n_rollouts']

        ## CREATE ARRAYS FOR REWARD, K_EP 
        reward_df = self.trial_df.iloc[:][['k_ep','reward']].dropna() # Create df from k_ep/rewards and drop blank reward rows
        rewards_arr = reward_df.to_numpy()
        rewards = rewards_arr[:,1]
        k_ep_r = rewards_arr[:,0]

        ## CREATE ARRAYS FOR REWARD_AVG, K_EP
        rewardAvg_df = reward_df.groupby('k_ep').mean() # Group rows by k_ep value and take their mean 
        rewards_avg = rewardAvg_df.to_numpy()
        k_ep_ravg = reward_df.k_ep.unique() # Drops duplicate values so it matches dimension of rewards_avg (1 avg per ep and n ep)


        fig = plt.figure(figNum)
        ax = fig.add_subplot(111)
        ax.scatter(k_ep_r,rewards,marker='_',color='black',alpha=0.5,label='Reward')
        ax.scatter(k_ep_ravg,rewards_avg,marker='o',color='red',label='Reward_avg')
        


        ax.set_ylabel("Rewards")
        ax.set_xlabel("k_ep")
        ax.set_xlim(-2,25)
        ax.set_ylim(-2,150)
        ax.set_title(f"Reward vs Episode | Rollouts per Episode: {num_rollouts*2}")
        ax.legend()
        ax.grid()

        plt.show()

    # def landing_rate(self): ## NEEDS UPDATED
    #     """Determines successful landing rate from final two episodes

    #     Returns:
    #         [float]: [Landing rate]
    #     """    
    #     ## GRAB REWARD DF ALONG W/ INIT N_ROLLOUTS AND LANDING CUTTOFF CRITERIA
    #     reward_df = self.trial_df.iloc[:][['k_ep','reward']].dropna()
    #     num_rollouts = self.trial_df.iloc[-1]['n_rollouts']
    #     landing_cutoff = 18.5

    #     ## GRAB FINAL N_ROLLOUT REWARDS AND CALC SUCCESSFUL LANDINGS
    #     temp = reward_df.iloc[-int(num_rollouts*4):]['reward'] # Store last [20] rows
    #     landings= temp[temp>landing_cutoff].count()
    #     sim_bug = temp[temp<3.00].count()

    #     attempts = num_rollouts*4-sim_bug
    #     landingRate = landings/attempts

    #     return landingRate


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
        policy = np.fromstring(policy[2:-2], dtype=float, sep=' ')  # Convert str to np array

        return policy

    def plot_policy_convg(self,trialNum=np.nan): ## NEEDS UPDATED
        """Creates subplots to show convergence for policy gains

        Args:
            trialNum ([int], optional): Display trial number. Defaults to np.nan.
        """        

        ## CLEAN AND GRAB DATA FOR MU & SIGMA
        policy_df = self.trial_df.iloc[:][['k_ep','mu','sigma']]
        policy_df = policy_df.dropna().drop_duplicates()
        k_ep_arr = policy_df.iloc[:]['k_ep'].to_numpy()

        

        ## CREATE NP ARRAYS FOR MU & SIGMA OVER TRIAL
        mu_arr = []
        for mu in policy_df.iloc[:]['mu']:
            mu = np.fromstring(mu[2:-2], dtype=float, sep=' ')
            mu_arr.append(mu)
        mu_arr = np.array(mu_arr)

        sigma_arr = []
        for sigma in policy_df.iloc[:]['sigma']:
            sigma = np.fromstring(sigma[2:-2], dtype=float, sep=' ')
            sigma_arr.append(sigma)
        sigma_arr = np.array(sigma_arr)

        

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
        # for jj in range(num_col): # Iterate through gains and plot each
        #     ax.plot(k_ep_arr,sigma_arr[:,jj],label=G_Labels[jj])

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
        mu = np.fromstring(mu[2:-2], dtype=float, sep=' ')  # Convert str to np array
        

        sigma = IC_df.iloc[0]['sigma']
        sigma = np.fromstring(sigma[2:-2], dtype=float, sep=' ')
        

        return mu,sigma



## STATE FUNCTIONS (Working)
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
        time = self.grab_stateData(k_ep,k_run,'t')
        time = time - np.min(time) # Normalize time

        
        ## PLOT STATE/TIME DATA
        fig = plt.figure(figNum)
        ax = fig.add_subplot(111)
        ax.plot(time,state,label=f"{stateName}")


        ax.set_ylabel(f"{stateName}")
        ax.set_xlabel("time [s]")
        ax.legend()
        ax.grid()

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

    def grab_M_d(self,k_ep,k_run):
        """Returns desired M_d as list
        Args:
            k_ep (int): Episode number
            k_run (int): Run number

        Returns:
            [list]: [Mx_d,My_d,Mz_d]
        """        
        run_df,IC_df = self.select_run(k_ep,k_run)

        Mx_d = IC_df.iloc[0]['Mx']
        My_d = IC_df.iloc[0]['My']
        Mz_d = IC_df.iloc[0]['Mz']

        return [Mx_d,My_d,Mz_d]

    def grab_M_d_trial(self):
        """Returns average omega_d over final two episodes

        Returns:
            [list]: [wx_d,wy_d,wz_d]
        """        

        num_rollouts = int(self.trial_df.iloc[-1]['n_rollouts'])

        # Use reward in df just to easily grab IC rows
        M_d_df = self.trial_df.iloc[:][['reward','Mx','My','Mz']].dropna()
        
        Mx_d = M_d_df.iloc[-num_rollouts*4:].mean()['Mx']
        My_d = M_d_df.iloc[-num_rollouts*4:].mean()['My']
        Mz_d = M_d_df.iloc[-num_rollouts*4:].mean()['Mz']


        return [Mx_d,My_d,Mz_d]







    def grab_eulerData(self,k_ep,k_run,degrees=True):

        run_df,IC_df = self.select_run(k_ep,k_run)
        quat_df = run_df.iloc[:][['t','qw','qx','qy','qz']]
        

        quat_arr = quat_df[['qx','qy','qz','qw']].to_numpy()
        rot = Rotation.from_quat(quat_arr)
        eul_arr = rot.as_euler('xyz', degrees=degrees)
        eul_df = pd.DataFrame(eul_arr,columns=['eul_x','eul_y','eul_z'])

        # eul_df['eul_x'] = -eul_df['eul_x']
        eul_df['eul_y'] = -eul_df['eul_y']
        # eul_df['eul_z'] = -eul_df['eul_z']

        eul_x = eul_df['eul_x']
        eul_y = eul_df['eul_y']
        eul_z = eul_df['eul_z']

        return [eul_x,eul_y,eul_z]

    def plot_eulerData(self,k_ep,k_run,eul_type):
        

        eul = 0
        if eul_type == 'eul_x':
            eul = self.grab_eulerData(k_ep,k_run)[0]
        elif eul_type == 'eul_y':
            eul = self.grab_eulerData(k_ep,k_run)[1]
        elif eul_type == 'eul_z':
            eul = self.grab_eulerData(k_ep,k_run)[2]
        else:
            print('Error, please select: ("eul_x","eul_y", or "eul_z")')


        time = self.grab_stateData(k_ep,k_run,'t')
        time = time - np.min(time) # Normalize time

        
        ## PLOT STATE/TIME DATA
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(time,eul,label=f"{eul_type}")


        ax.set_ylabel(f"{eul_type} [deg]")
        ax.set_xlabel("time [s]")
        ax.legend()
        ax.grid()

        plt.show()




    def grab_flip_time(self,k_ep,k_run):
        run_df,IC_df = self.select_run(k_ep,k_run)
        t_flip = run_df.query(f"flip_flag=={True}").iloc[0]['t']
        t_flip_norm = t_flip - run_df.iloc[0]['t']

        return t_flip,t_flip_norm






#Region
## IMPACT FUNCTIONS (ALL NEED UPDATED)
    # def grab_impact_time(self,k_ep,k_run):
    #     _,t_impact,t_impact_norm = self.grab_impact_angle(k_ep,k_run)

    #     return t_impact,t_impact_norm


    # def grab_impact_angle(self,k_ep,k_run):
    #     """Returns pitch angle at time of impact

    #     Args:
    #         k_ep (int): Episode number
    #         k_run (int): Run number

    #     Returns:
    #         List: eul_impact,t_impact,t_impact_normalized
    #     """        
    #     ## NOTE: This is only verified for 2-D case with Pitch maneuver

    #     run_df = self.select_run(k_ep,k_run)
    #     quat_df = run_df.iloc[:-1][['t','qw','qx','qy','qz']]
        

    #     quat_arr = quat_df[['qx','qy','qz','qw']].to_numpy()
    #     rot = Rotation.from_quat(quat_arr)
    #     eul_arr = rot.as_euler('xyz', degrees=True)
    #     eul_df = pd.DataFrame(eul_arr,columns=['eul_x','eul_y','eul_z'])

    #     eul_df['eul_y'] = -eul_df['eul_y']


    #     quat_df = pd.concat([quat_df.reset_index(),eul_df],axis=1)
    #     eul_impact = max(eul_df[f"eul_y"],key=abs)
    #     t_impact = quat_df.query(f'eul_y=={eul_impact}').iloc[0]['t']

    #     t_initial = quat_df.iloc[0]['t']
    #     t_impact_norm = t_impact - t_initial
    #     return eul_impact,t_impact,t_impact_norm

    # def grab_impact_angle_trial(self):
    #     """Returns avg angle at impact for final two episodes

    #     Returns:
    #         Float: eul_impact
    #     """

    #     num_rollouts = int(self.trial_df.iloc[-1]['n_rollouts'])

    #     ep_df = self.trial_df.iloc[:][['k_ep','k_run']].drop_duplicates()
    #     ep_arr = ep_df.iloc[-num_rollouts*4:].to_numpy() # Grab episode/run listing from past 2 rollouts

    #     list = []
    #     for k_ep,k_run in ep_arr:
    #         list.append(self.grab_impact_angle(k_ep,k_run)[0])

    #     arr = np.asarray(list)
    #     eul_impact = np.mean(arr,axis=0)

    #     return eul_impact


    # def grab_impact_vel(self,k_ep,k_run):
    #     """Returns vel at time of impact

    #     Args:
    #         k_ep (int): Episode number
    #         k_run (int): Run number

    #     Returns:
    #         list: [vx_impact,vy_impact,vz_impact]
    #     """ 

    #     run_df = self.select_run(k_ep,k_run)
    #     t_impact,_ = self.grab_impact_time(k_ep,k_run)

    #     vx_impact = run_df.query(f't=={t_impact}').iloc[0]['vx']
    #     vy_impact = run_df.query(f't=={t_impact}').iloc[0]['vy']
    #     vz_impact = run_df.query(f't=={t_impact}').iloc[0]['vz']

    #     return [vx_impact,vy_impact,vz_impact]

    # def grab_impact_vel_trial(self):
    #     """Returns avg vel at impact for final two episodes

    #     Returns:
    #         [np.array]: [vx_impact,vy_impact,vz_impact]
    #     """
    
    #     num_rollouts = int(self.trial_df.iloc[-1]['n_rollouts'])

    #     ep_df = self.trial_df.iloc[:][['k_ep','k_run']].drop_duplicates()
    #     ep_arr = ep_df.iloc[-num_rollouts*4:].to_numpy() # Grab episode/run listing from past 2 rollouts

    #     list = []
    #     for k_ep,k_run in ep_arr:
    #         list.append(self.grab_impact_vel(k_ep,k_run))

    #     arr = np.asarray(list)
    #     vel_impact = np.mean(arr,axis=0)

    #     return vel_impact


    # def grab_impact_omega(self,k_ep,k_run):
    #     """Returns omega at time of impact

    #     Args:
    #         k_ep (int): Episode number
    #         k_run (int): Run number

    #     Returns:
    #         list: [wx_impact,wy_impact,wz_impact]
    #     """        
    #     run_df = self.select_run(k_ep,k_run)
    #     t_impact,_ = self.grab_impact_time(k_ep,k_run)

    #     wx_impact = run_df.query(f't=={t_impact}').iloc[0]['wx']
    #     wy_impact = run_df.query(f't=={t_impact}').iloc[0]['wy']
    #     wz_impact = run_df.query(f't=={t_impact}').iloc[0]['wz']

    #     return [wx_impact,wy_impact,wz_impact]

    # def grab_impact_omega_trial(self):

    #     """Returns avg omega at impact for final two episodes

    #     Returns:
    #         [np.array]: [wx_impact,wy_impact,wz_impact]
    #     """        
    #     num_rollouts = int(self.trial_df.iloc[-1]['n_rollouts'])

    #     ep_df = self.trial_df.iloc[:][['k_ep','k_run']].drop_duplicates()
    #     ep_arr = ep_df.iloc[-num_rollouts*4:].to_numpy() # Grab episode/run listing from past 2 rollouts

    #     list = []
    #     for k_ep,k_run in ep_arr:
    #         list.append(self.grab_impact_omega(k_ep,k_run))

    #     arr = np.asarray(list)
    #     w_impact = np.mean(arr,axis=0)
        
    #     return w_impact

    # ## FLIP FUNCTIONS (ALL NEED UPDATED)
#     def grab_omega_flip(self,k_ep,k_run):
#         """Returns omega at flip for specific ep/run

#         Args:
#             k_ep (int): Episode number
#             k_run (int): Run number

#         Returns:
#             List: [wx_max,wy_max,wz_max]
#         """       
#         run_df = self.select_run(k_ep,k_run)

#         wx_arr = run_df.iloc[:-1]['wx'].ewm(span=10).mean() # Take moving average to smooth outliers from impact
#         wx_max = max(wx_arr,key=abs)                        # Find max regardless of number sign

#         wy_arr = run_df.iloc[:-1]['wy'].ewm(span=10).mean()
#         wy_max = max(wy_arr,key=abs)

#         wz_arr = run_df.iloc[:-1]['wz'].ewm(span=10).mean()
#         wz_max = max(wz_arr,key=abs)

#         return [wx_max,wy_max,wz_max]

#     def grab_omega_flip_trial(self):
#         """Returns avg omega at flip for final two episodes

#         Returns:
#             [np.array]: [wx,wy,wz]
#         """      
#         num_rollouts = int(self.trial_df.iloc[-1]['n_rollouts'])

#         ep_df = self.trial_df.iloc[:][['k_ep','k_run']].drop_duplicates()
#         ep_arr = ep_df.iloc[-num_rollouts*4:].to_numpy() # Grab episode/run listing from past 2 rollouts

#         list = []
#         for k_ep,k_run in ep_arr:
#             list.append(self.grab_omega_flip(k_ep,k_run))

#         arr = np.asarray(list)
#         w_flip = np.mean(arr,axis=0)
        
#         return w_flip


#     def grab_vel_flip(self,k_ep,k_run):
#         """Returns vel at flip for specific ep/run

#         Args:
#             k_ep (int): Episode number
#             k_run (int): Run number

#         Returns:
#             List: [vx_max,vy_max,vz_max]
#         """        
#         run_df = self.select_run(k_ep,k_run)

#         vx_arr = run_df.iloc[:-1]['vx'].ewm(span=10).mean() # Take moving average to smooth outliers from impact
#         vx_max = max(vx_arr,key=abs)                        # Find max regardless of number sign

#         vy_arr = run_df.iloc[:-1]['vy'].ewm(span=10).mean()
#         vy_max = max(vy_arr,key=abs)

#         vz_arr = run_df.iloc[:-1]['vz'].ewm(span=10).mean()
#         vz_max = max(vz_arr,key=abs)

#         return [vx_max,vy_max,vz_max]

#     def grab_vel_flip_trial(self):
#         """Returns avg vel at flip for final two episodes

#         Returns:
#             [np.array]: [vx,vy,vz]
#         """        
#         num_rollouts = int(self.trial_df.iloc[-1]['n_rollouts'])

#         ep_df = self.trial_df.iloc[:][['k_ep','k_run']].drop_duplicates()
#         ep_arr = ep_df.iloc[-num_rollouts*4:].to_numpy() # Grab episode/run listing from past 2 rollouts

#         list = []
#         for k_ep,k_run in ep_arr:
#             list.append(self.grab_vel_flip(k_ep,k_run))

#         arr = np.asarray(list)
#         v_flip = np.mean(arr,axis=0)
        
#         return v_flip
#Endregion

