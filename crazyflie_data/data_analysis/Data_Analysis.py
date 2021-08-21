import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import os
import warnings
import re

# os.system("clear")
np.set_printoptions(suppress=True)

class DataFile:
    def __init__(self,dataPath,fileName,dataType='SIM'):

        ## ORGANIZE FILEPATH AND CREATE TRIAL DATAFRAME
        self.fileName = fileName
        self.dataPath = dataPath
        filepath = self.dataPath + self.fileName

        # self.dataType = re.findall('SIM|EXP',fileName)[0] # FIND 'SIM' OR 'EXP'
        self.trialNum = int(re.findall('trial_(\d+)',fileName)[0])
        self.trial_df = pd.read_csv(filepath,low_memory=False)

        ## CLEAN UP TRIAL DATAFRAME
        # Drop row with "Note: ____________"
        self.dataType = dataType
        # self.trial_df.drop(0,inplace=True)

        # Remove rows past final complete rollout
        final_valid_index = self.trial_df[self.trial_df['Error']=='Impact Data'].index.values[-1]
        self.trial_df = self.trial_df.iloc[:final_valid_index+1]

        # Round values to prevent floating point precision issues
        self.trial_df = self.trial_df.round(3) 

        # Convert string bools to actual bools
        self.trial_df = self.trial_df.replace({'False':False,'True':True})

        ## CREATE BASIC DF OF K_EP,K_RUN, & REWARD
        self.k_df = self.trial_df.iloc[:][['k_ep','k_run','reward']].dropna()
        self.k_df.reset_index(inplace=True)

        if self.dataType=='EXP':
            self.remove_FailedRuns()

        ## COLLECT BASIC TRIAL INFO
        self.n_rollouts = int(self.trial_df.iloc[0]['n_rollouts'])
        # self.n_rollouts = 8
        self.k_epMax = int(self.trial_df.iloc[-1]['k_ep'])


        ## INITIATE CLASS FOR GTC MODEL (UNFINISHED)
        # self.GTC = GTC_Model()
        

        

    def remove_FailedRuns(self):
        """Drop all rollouts in dataframe that were labelled as 'Failed Rollout'
        """        

        failedRuns_list = self.trial_df.query(f"Error=='Failed Rollout'").index.tolist()

        for failed_run in failedRuns_list[::-1]: # Work backwards down list

            idx_prev_run = self.k_df.query(f"index == {failed_run}").index.tolist()[0]-1
            if idx_prev_run < 0: # Catch for very first run
                idx_prev_run = 0

            idx_start = self.k_df.iloc[idx_prev_run]['index'] + 1 # True index of trial_df to start drop
            idx_end = failed_run + 1 # Make sure to include endpoint

            self.trial_df.drop(np.arange(idx_start,idx_end,dtype='int'),inplace=True)

        self.k_df = self.trial_df.iloc[:][['k_ep','k_run','reward']].dropna()
        self.k_df.reset_index(inplace=True)

    def select_run(self,k_ep,k_run): ## Create run dataframe from k_ep and k_run
        """Create run dataframe from selected k_ep and k_run

        Args:
            k_ep (int): Episode number
            k_run (int): Run number

        Returns:
            Experiment:
                tuple: run_df,IC_df,__,__
            Sim:
                tuple: run_df,IC_df,flip_df,impact_df
        """        

        ## CREATE RUN DATAFRAME
        run_df = self.trial_df[(self.trial_df['k_ep']==k_ep) & (self.trial_df['k_run']==k_run)]
        run_df = run_df.replace({'False':False,'True':True})    # Convert all string bools to normal bools


        IC_df = run_df.iloc[[-3]]       # Create DF of initial conditions
        flip_df = run_df.iloc[[-2]]     # Create DF of flip conditions
        impact_df = run_df.iloc[[-1]]   # Create DF of impact conditions
        run_df = run_df[:-3]            # Drop special rows from dataframe

        return run_df,IC_df,flip_df,impact_df

    def grab_rewardData(self):
        """Returns summary of rewards
        
        Data Type: Sim/Exp
            
        Returns:
            tuple: k_ep_r,rewards,k_ep_ravg,rewards_avg

            ===========================
            k_ep_r: reward episodes (np.array)
            rewards: array of rewards (np.array)
            k_ep_ravg: episode of averaged reward (np.array)
            rewards_avg: array of averaged rewards per episode (np.array)
        """        
        ## CREATE ARRAYS FOR REWARD, K_EP 
        reward_df = self.trial_df.iloc[:][['k_ep','mu','reward']].dropna() # Create df from k_ep/rewards and drop blank reward rows
        reward_df = reward_df.iloc[:][["k_ep","reward"]].astype('float')
        reward_df = reward_df.query("reward >= 3.00")
        rewards_arr = reward_df.to_numpy()
        rewards = rewards_arr[:,1]
        k_ep_r = rewards_arr[:,0]

        ## CREATE ARRAYS FOR REWARD_AVG, K_EP
        rewardAvg_df = reward_df.groupby('k_ep').mean() # Group rows by k_ep value and take their mean 
        rewards_avg = rewardAvg_df.to_numpy()
        k_ep_ravg = reward_df.k_ep.unique() # Drops duplicate values so it matches dimension of rewards_avg (1 avg per ep and n ep)

        return k_ep_r,rewards,k_ep_ravg,rewards_avg

    def rewardAvg_trial(self,N:int=3):
        """Returns average reward from the last N episodes of the trial

        Data Type: Sim/Exp

        Args:
            N (int, optional): Last N episodes of the trial. Defaults to 3

        Returns:
            float: average_reward
        """        
               
        ## CREATE ARRAYS FOR REWARD, K_EP 
        reward_df = self.trial_df.iloc[:][['reward']].dropna() # Create df from k_ep/rewards and drop blank reward rows
        reward_df = reward_df.iloc[-int(N)*self.n_rollouts:]['reward'] # Trim to last N episodes
        rewards_arr = reward_df.to_numpy()
        avg_reward = np.nanmean(rewards_arr)
        
        return avg_reward

    def plot_rewardData(self,ymax=200):
        """Plot rewards for entire trial
        Data Type: Sim/Exp
        """        
        k_ep_r,rewards,k_ep_ravg,rewards_avg = self.grab_rewardData()

        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.scatter(k_ep_r,rewards,marker='_',color='black',alpha=0.5,label='Reward')
        ax.scatter(k_ep_ravg,rewards_avg,marker='o',color='red',label='Average Reward')
        
        

        ax.set_ylabel("Reward")
        ax.set_xlabel("k_ep")
        ax.set_xlim(-2,self.k_epMax+5)
        ax.set_ylim(-2,ymax)
        ax.set_title(f"Reward vs Episode | Rollouts per Episode: {self.n_rollouts}")
        ax.legend()
        ax.grid()

        fig.tight_layout()
        plt.show()

    ## POLICY FUNCTIONS
    def grab_policy(self,k_ep,k_run):
        """Returns policy from specific run

        Data Type: Sim/Exp

        Args:
            k_ep (int): Episode number
            k_run (int): Run number

        Returns:
            np.array: [RREV,G1,G2,...]
        """        
        run_df,IC_df,_,_ = self.select_run(k_ep,k_run)

        ## SELECT POLICY
        policy = IC_df.iloc[0]['policy']
        policy = np.fromstring(policy[1:-1], dtype=float, sep=' ')  # Convert str to np array

        return policy

    def grab_convg_data(self):
        """Returns series of arrays for episodes, mu values, and sigma values

        Data Type: Sim/Exp


        Returns:
           k_ep_arr [np.array]: Array of episodes
           mu_arr [np.array]: Array of mu values across trial
           sigma_arr [np.array]: Array of sigma values across trial

        """        
        ## CLEAN AND GRAB DATA FOR MU & SIGMA
        policy_df = self.trial_df.iloc[:][['k_ep','mu','sigma']]
        policy_df = policy_df.dropna().drop_duplicates()
        k_ep_arr = policy_df.iloc[:]['k_ep'].to_numpy()

        

        ## CREATE ARRAYS FOR MU & SIGMA OVER TRIAL
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

    def plot_policy_convg(self): ## NEEDS UPDATED
        """Creates subplots to show convergence for policy gains
        """      

        ## GRAB CONVERGENCE DATA
        k_ep_arr,mu_arr,sigma_arr = self.grab_convg_data()

        

        num_col = mu_arr.shape[1] # Number of policy gains in mu [Currently 3]
        G_Labels = ['RREV_trigger','My','G2','G3','G4','G5'] # List of policy gain names
        vx,vy,vz = self.grab_vel_d()

        Vel = np.sqrt(vx**2 + vz**2)
        phi = np.arctan2(vz,vx)


        ## CREATE SUBPLOT FOR MU 
        fig = plt.figure()
        ax = fig.add_subplot(211)
        for jj in range(num_col): # Iterate through gains and plot each
            ax.plot(k_ep_arr,mu_arr[:,jj],label=G_Labels[jj])


        ax.set_ylabel('Policy Values')
        ax.set_xlabel('K_ep')
        ax.set_ylim(0) # Set lower ylim
        ax.set_title(f'Policy Value vs Episode ($Vel_d$ = {Vel:.2f} | $\phi$ = {np.rad2deg(phi):.2f}$^{{\circ}}$)')
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.25),ncol=num_col)
        ax.grid()
        fig.tight_layout()


        ## CREATE SUBPLOT FOR SIGMA
        ax = fig.add_subplot(212)
        for jj in range(num_col): # Iterate through gains and plot each
            ax.plot(k_ep_arr,sigma_arr[:,jj],label=G_Labels[jj])

        ax.set_ylabel('Standard Deviation')
        ax.set_xlabel('K_ep')
        ax.set_title(f'Policy Std. Dev. vs Episode ($Vel_d$ = {Vel:.2f} | $\phi_d$ = {np.rad2deg(phi):.2f}$^{{\circ}}$)')
        ax.legend(ncol=3,loc='upper right')
        ax.grid()

        fig.tight_layout()
        plt.show()

    def grab_finalPolicy(self):
        """Returns the final policy for a trial

        Data Type: Sim/Exp

        Returns:
           mu [np.array]: Final policy average
           sigma [np.array]: Final policy standard deviation

        """        
        
        ## FIND FINAL RUN DF
        k_ep = self.trial_df.iloc[-1]['k_ep']
        k_run = self.trial_df.iloc[-1]['k_run']
        run_df,IC_df,_,_ = self.select_run(k_ep,k_run)

        ## SELECT MU & SIGMA
        mu = IC_df.iloc[0]['mu']
        mu = np.fromstring(mu[1:-1], dtype=float, sep=' ')  # Convert str to np array
        

        sigma = IC_df.iloc[0]['sigma']
        sigma = np.fromstring(sigma[1:-1], dtype=float, sep=' ')
        

        return mu,sigma

    def grab_RLParams(self):
        """Returns initial RL Parameters

        Data Type: Sim/Exp

        Returns:
            alpha_mu [float]: Mu learning rate (PEPG)
            alpha_sigma [float]: Sigma learning rate (PEPG)
            mu [float]: Mean value
            sigma [float]: Standard deviation
        """        
        
        # CREATE INITIAL PARAMETER DATAFRAME
        param_df = self.trial_df.iloc[:][['alpha_mu','alpha_sig','mu','sigma']].dropna() 

        # GRAB AND CLEAN INITIAL PARAMETER VALUES
        alpha_mu = param_df.iloc[0]['alpha_mu']
        alpha_mu = np.fromstring(alpha_mu[1:-1], dtype=float, sep=' ')  # Convert str to np.array

        alpha_sigma = param_df.iloc[0]['alpha_sig']
        alpha_sigma = np.fromstring(alpha_sigma[1:-1], dtype=float, sep=' ')  

        mu = param_df.iloc[0]['mu']
        mu = np.fromstring(mu[1:-1], dtype=float, sep=' ')  

        sigma = param_df.iloc[0]['sigma']
        sigma = np.fromstring(sigma[1:-1], dtype=float, sep=' ')  


        return alpha_mu,alpha_sigma,mu,sigma


    def plot_state_spread_flip(self,stateName,N:int=3): # Plot histogram showing spread of state values over last N episodes
        
        ## CREATE ARRAYS FOR REWARD, K_EP 
        a = self.trial_df.query(f"Error == 'Flip Data'").iloc[-int(N)*self.n_rollouts:][['RREV']].to_numpy()
        y,x = np.histogram(a,bins=[0,2,4,6,8,10])

        fig = plt.figure(0)
        ax = fig.add_subplot(111)

        ax.hist(a,bins=10)
        ax.grid()
        plt.show()

    def grab_leg_contacts(self):

        ## COLLECT CONTACT LIST
        leg_contacts_df = self.trial_df.query("Error=='Impact Data'").iloc[:][['impact_flag']]

        a = leg_contacts_df.to_numpy().flatten()
        b = np.zeros_like(a)

        ## CONVERT STRING ARRAY TO NP ARRAY AND COUNT NUMBER OF LEG CONTACTS
        for idx, i in enumerate(a): 
            a[idx] = np.fromstring(i[1:-1], dtype=float, sep=' ')
            b[idx] = np.size(a[idx])


        leg_contacts_df = pd.DataFrame({"Contact_Order":a,"Leg_Contacts":b})
        return leg_contacts_df


    


    ## STATE FUNCTIONS 
    def grab_stateData(self,k_ep,k_run,stateName: list):
        """Returns np.array of specified state

        Data Type: Sim/Exp

        Args:
            k_ep (int): Episode number
            k_run (int): Run number
            stateName (str): State name

        Returns:
            state [np.array]: Returned state values
        """        
        run_df,IC_df,_,_ = self.select_run(k_ep,k_run)

        ## GRAB STATE DATA AND CONVERT TO NUMPY ARRAY
        state = run_df.iloc[:][stateName]
        state = state.to_numpy()

        return state

    def plot_state(self,k_ep,k_run,stateName: list): 
        """Plot state data from run vs time

        Data Type: Sim/Exp

        Args:
            k_ep (int): Ep. Num
            k_run (int): Run. Num
            stateName list(str): State name from .csv
        """        

        ## PLOT STATE/TIME DATA
        fig = plt.figure()
        ax = fig.add_subplot(111)

        t = self.grab_stateData(k_ep,k_run,'t')
        t_norm = t - np.min(t) # Normalize time array

        ## GRAB STATE/FLIP DATA
        for state in stateName:

            color = next(ax._get_lines.prop_cycler)['color']

            ## PLOT STATE DATA
            state_vals = self.grab_stateData(k_ep,k_run,state)
            ax.plot(t_norm,state_vals,label=f"{state}",zorder=1,color=color)



            # ## MARK STATE DATA AT FLIP TIME
            # t_flip,t_flip_norm = self.grab_flip_time(k_ep,k_run)
            # state_flip = self.grab_flip_state(k_ep,k_run,state)
            # ax.scatter(t_flip_norm,state_flip,label=f"{state}-Flip",zorder=2,
            #             marker='x',
            #             color=color,
            #             s=100)

            # # ## MARK STATE DATA AT IMPACT TIME
            # t_impact,t_impact_norm = self.grab_impact_time(k_ep,k_run)
            # state_flip = self.grab_impact_state(k_ep,k_run,state)
            # ax.scatter(t_impact_norm,state_flip,label=f"{state}-Impact",zorder=2,
            #             marker='s',
            #             color=color,
            #             s=50)

            ## MARK STATE DATA AT TRAJ START
            # if self.dataType == 'EXP':
            #     t_traj,t_traj_norm = self.grab_traj_start(k_ep,k_run)
            #     state_traj = self.grab_traj_state(k_ep,k_run,state)
            #     ax.scatter(t_traj_norm,state_traj,label=f"{state}-Traj",zorder=2,
            #                 marker='o',
            #                 color=color,
            #                 s=25)


        ax.set_xlabel("time [s]")
        ax.legend()
        ax.grid()

        plt.show()

    def grab_FullState(self,k_ep,k_run,t):
        run_df,IC_df,_,_ = self.select_run(k_ep,k_run)
        df_State = run_df.query(f"t=={t}").iloc[[0]][['x','y','z','qw','qx','qy','qz','vx','vy','vz','wx','wy','wz']]

        ## CREATE DICT OF FULL STATE FROM SELECTED DATA ROW
        Pos = df_State[['x','y','z']].to_numpy().T
        Vel = df_State[['vx','vy','vz']].to_numpy().T
        Omega = df_State[['wx','wy','wz']].to_numpy().T
        Quat = df_State[['qx','qy','qz','qw']].to_numpy()

        FullState = {
                'x': Pos,       # Pos. [x,y,z] - [m]
                'v': Vel,       # Lin. Vel [vx,vy,vz] - [m/s]
                'w': Omega,     # Ang. Vel [wx,wy,wz] - [rad/s]
                'quat': Quat    # Orientation [qx,qy,qz,qw]
            }

        if self.dataType == 'EXP':
        ## CREATE DICT OF DESIRED STATE FROM SELECTED DATA ROW
            df_DC = run_df.query(f"t=={t}").iloc[[0]][['x_d.x','x_d.y','x_d.z','v_d.x','v_d.y','v_d.z','a_d.x','a_d.y','a_d.z']]

            x_d = df_DC[['x_d.x','x_d.y','x_d.z']].to_numpy().T
            v_d = df_DC[['v_d.x','v_d.y','v_d.z']].to_numpy().T
            a_d = df_DC[['a_d.x','a_d.y','a_d.z']].to_numpy().T

            DesiredState = {
                'x_d':x_d,
                'v_d':v_d,
                'a_d':a_d
            }

            return FullState,DesiredState
        
        return FullState,_



    def grab_accel_data(self,k_ep,k_run,External_data=False):
        """Return acceleration data for the system

        Args:
            k_ep (int): Episode number
            k_run (int): Run number

        Returns:
            accel (np.array): Array of system accelerations [ax,ay,az]
            accel_df (pd.DataFrame): Dataframe of acceleration data w/ time column
        """        

        run_df,IC_df,_,_ = self.select_run(k_ep,k_run)


        # CREATE ARRAY OF TIME DIFF VALUES
        dt = np.diff(run_df.iloc[:]['t'],prepend=0)[:,np.newaxis]

        # CREATE ARRAY OF VEL DIFF VALUES
        dv = np.diff(run_df.iloc[:][['vx','vy','vz']],prepend=0,axis=0)
            
        accel = dv/dt
        t = self.grab_stateData(k_ep,k_run,'t')[:,np.newaxis]
        accel_df = pd.DataFrame(np.hstack((t,accel)),columns=['t','a_x','a_y','a_z'])

        return accel,accel_df
  
    def grab_time(self,k_ep,k_run):
        run_df,IC_df,_,_ = self.select_run(k_ep,k_run)

        ## GRAB STATE DATA AND CONVERT TO NUMPY ARRAY
        state = run_df.iloc[:]['t']
        t = state.to_numpy()

        t_normalized = t - np.min(t)

        return t,t_normalized

    def grab_eulerData(self,k_ep,k_run,degrees=True,External_data=False):
        """Returns euler angle data from rollout using [YZX] configuration to allow pitch angles greater than 180 deg

        Data Type: Sim/Exp

        Args:
            k_ep (int): Episode number
            k_run (int): Run/Rollout number
            degrees (bool, optional): Choose between degrees or radians for output. Defaults to True.
            External_data (bool, optional): Choose between internal estimate or external Mocap data. Defaults to False(Internal)

        Returns:
            [np.array]: Returns np.array of euler values in form (n x [eul_X,eul_Y,eul_Z])
        """        
        
        ## CREATE QUAT DATAFRAME FROM RUN_DF
        run_df,IC_df,_,_ = self.select_run(k_ep,k_run)
        
        if External_data==False:
            quat_df = run_df.iloc[:][['qw','qx','qy','qz']]
        else:
            quat_df = run_df.iloc[:][['qw_Ext','qx_Ext','qy_Ext','qz_Ext']]

        ## CONVERT QUATS TO EULER ANGLES AND BACK TO DF
        quat_arr = quat_df.to_numpy()
        R = Rotation.from_quat(quat_arr)
        eul_arr = R.as_euler('YZX', degrees=degrees)
        # Center axis [theta_2] is limited to +- 90deg while [theta_1 & theta_3] can rotate to 180deg 
        # https://graphics.fandom.com/wiki/Conversion_between_quaternions_and_Euler_angles
        # https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/Quaternions.pdf
        eul_df = pd.DataFrame(eul_arr,columns=['eul_y','eul_z','eul_x'])

        ## IF EUL_Y JUMPS FROM PAST -180 T0 +180 THEN BRING IT BACK DOWN AGAIN
        eul_df.loc[eul_df['eul_y'] > 170, 'eul_y'] = eul_df['eul_y']-360
        
        ## REARRANGE DF TO [X,Y,Z] FORMAT TO BE CONSISTENT
        eul_df = eul_df[['eul_x','eul_y','eul_z']]
        eul_arr = eul_df.to_numpy()

        return eul_arr

    def plot_eulerData(self,k_ep,k_run,eul_type):

        # for
        
        if eul_type == 'eul_x':
            eul = self.grab_eulerData(k_ep,k_run)[:,0]
        elif eul_type == 'eul_y':
            eul = self.grab_eulerData(k_ep,k_run)[:,1]
        elif eul_type == 'eul_z':
            eul = self.grab_eulerData(k_ep,k_run)[:,2]
        else:
            print('Error, please select: ("eul_x","eul_y", or "eul_z")')
            eul = np.nan


        t = self.grab_stateData(k_ep,k_run,'t')
        t_norm = t - np.min(t) # Normalize time

        t_flip,t_flip_norm = self.grab_flip_time(k_ep,k_run)
        # eul_flip = self.grab_flip_eul(k_ep,k_run,eul_type)
        

        
        ## PLOT STATE/TIME DATA
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(t_norm,eul,label=f"{eul_type}")
        # ax.scatter(t_norm_flip,eul_flip,label="Flip")
        # ax.scatter(t_norm_impact,eul_impact,label="Impact")
        


        ax.set_ylabel(f"{eul_type} [deg]")
        ax.set_xlabel("time [s]")
        ax.legend()
        ax.grid()

        plt.show()

    def plot_traj_2D(self,k_ep,k_run,ceil_height=2.0,External_data=False):
        
        ## GRAB/MODIFY DATA
        if External_data == False:
            # TRAJECTORY INFO
            x = self.grab_stateData(k_ep,k_run,'x')
            z = self.grab_stateData(k_ep,k_run,'z')
            eul_y = self.grab_eulerData(k_ep,k_run,degrees=False)[:,1]

            # FLIP INFO
            x_flip = self.grab_flip_state(k_ep,k_run,'x')
            z_flip = self.grab_flip_state(k_ep,k_run,'z')
            eul_flip = self.grab_flip_eul(k_ep,k_run)[:,1]

            # IMPACT INFO
            x_impact = self.grab_impact_state(k_ep,k_run,'x')
            z_impact = self.grab_impact_state(k_ep,k_run,'z')
            eul_impact = self.grab_impact_eul(k_ep,k_run)[:,1]

            
        elif External_data == True:
            x = self.grab_stateData(k_ep,k_run,'x_Ext')
            z = self.grab_stateData(k_ep,k_run,'z_Ext')
            eul_y = self.grab_eulerData(k_ep,k_run,External_data=True,degrees=False)[:,1]


        
        ## CALC QUIVER VECTOR POSITIONS
        u_bz = np.sin(eul_y)
        v_bz = np.cos(eul_y)

        u_bx = np.cos(eul_y)
        v_bx = np.sin(eul_y)



        ## PLOT DATA
        fig = plt.figure()
        ax = fig.add_subplot(111,aspect='equal')

        ax.plot(x,z)
        ax.quiver(x[::4],z[::4],u_bz[::4],v_bz[::4],width = 0.005,color='gray')
        # ax.quiver(x[::4],z[::4],u_bx[::4],v_bx[::4],width = 0.005,color='gray')

        ax.scatter(x_flip,z_flip,marker='x',color='purple',label='Flip Location',zorder=3)
        ax.scatter(x_impact,z_impact,marker='x',color='k',label='Impact Location',zorder=3)

        ax.set_xlabel("x-pos [m]")
        ax.set_ylabel("z-pos [m]")
        ax.set_title(f"Rollout Trajectory | K_ep: {k_ep} & K_run: {k_run}")
        

        ax.set_xlim([-2,2])
        ax.set_ylim([0,ceil_height+0.5])

        ax.hlines(ceil_height,-5,5)
        ax.text(-1.98,ceil_height+0.02,"Ceiling Plane")
        ax.grid()
        ax.legend()

        plt.show()

    def plot_VC_traj_2D(self,k_ep,k_run):
        """Plot flight trajectory through Visual_Cue-Space until flip execution

        Args:
            k_ep (int): Episode Number
            k_run (int): Run Number
        """   

        run_df,IC_df,_,_ = self.select_run(k_ep,k_run)

        ## GRAB/MODIFY DATA AND CONVERT TO NUMPY ARRAY
        RREV_traj = run_df.query(f"flip_flag==False & `v_d.z`> {0.0}").iloc[:]['RREV'].to_numpy()
        OF_y_traj = run_df.query(f"flip_flag==False & `v_d.z`> {0.0}").iloc[:]['OF_y'].to_numpy()
        OF_y_traj = np.abs(OF_y_traj) # Convert OF_y to pure magnitude

        OF_x_traj = run_df.query(f"flip_flag==False & `v_d.z`> {0.0}").iloc[:]['OF_x'].to_numpy()
        OF_x_traj = np.abs(OF_x_traj) # Convert OF_y to pure magnitude


        ## PLOT DATA
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
        ax.plot(OF_y_traj,RREV_traj,'k--')
        ax.scatter(OF_y_traj[0],RREV_traj[0],marker='o',color='k',label='Velocity Imparted')
        ax.scatter(OF_y_traj[-1],RREV_traj[-1],marker='x',color='k',label='Flip Executed')

        ax.set_xlim(-1,10)
        ax.set_ylim(-1,10)
       
        ax.set_ylabel("RREV [1/s]")
        ax.set_xlabel("|OF_y| [rad/s]")
        ax.set_title("Flight Trajectory - Visual Cue Space")
        ax.legend()
        ax.grid()

        plt.show()



    ## DESIRED IC FUNCTIONS
    def grab_vel_IC(self):
    
        """Return IC velocities

        Returns:
            vx_IC (float): Desired flip x-velocity
            vy_IC (float): Desired flip y-velocity
            vz_IC (float): Desired flip z-velocity
        """        
        vel_df = self.trial_df[['mu','vx','vy','vz']].dropna()
        vx_IC,vy_IC,vz_IC = vel_df.iloc[0][['vx','vy','vz']]
        
        return vx_IC,vy_IC,vz_IC

    def grab_vel_IC_2D_angle(self):
        """Return IC velocities

        Returns:
            vx_IC (float): Desired flip x-velocity
            vy_IC (float): Desired flip y-velocity
            vz_IC (float): Desired flip z-velocity
        """        
        ## SELECT X,Y,Z LAUNCH VELOCITIES
        vel_df = self.trial_df[['mu','vx','vy','vz']].dropna()
        vx_IC,vy_IC,vz_IC = vel_df.iloc[0][['vx','vy','vz']]

        ## CONVERT CARTESIAN VELOCITIES TO POLAR
        Vel_IC = np.sqrt(vx_IC**2 + vz_IC**2)
        phi_IC = np.rad2deg(np.arctan2(vz_IC,vx_IC))

        

        phi_IC = np.round(phi_IC,1)
        Vel_IC = np.round(Vel_IC,1)
        
        return Vel_IC,phi_IC

    def grab_My_d(self,k_ep,k_run):
        """Returns desired M_d as list
        Args:
            k_ep (int): Episode number
            k_run (int): Run number

        Returns:
            Float: My_d
        """        
        run_df,IC_df,flip_df,_ = self.select_run(k_ep,k_run)

        ## CONVERT MOMENT THRUST IN GRAMS TO N*m
        My_d = (flip_df.iloc[0]['My'])*1e-3*9.81*0.033*2

        ## CONVERT TO N*mm
        My_d = My_d*1e3

        return My_d

    def grab_My_d_trial(self,N:int=3,reward_cutoff:float=3.00):   

        ## CREATE ARRAY OF ALL EP/RUN COMBINATIONS FROM LAST N ROLLOUTS
        # Use reward to extract only the valid attempts and not simulation mishaps
        ep_df = self.trial_df.iloc[:][['k_ep','k_run','reward']].astype('float').query(f'reward >= {reward_cutoff}')
        ep_arr = ep_df.iloc[-self.n_rollouts*N:].to_numpy() # Grab episode/run listing from past N rollouts

        ## ITERATE THROUGH ALL RUNS AND FINDING My_D FOR VALID LANDINGS
        var_list = []
        for k_ep,k_run in ep_arr[:,:2]:

            leg_contacts,_,_,_ = self.landing_conditions(k_ep, k_run)
            if leg_contacts >= 3: # IGNORE FAILED LANDINGS
                var_list.append(self.grab_My_d(k_ep,k_run))

        ## RETURN MEAN AND STD OF STATE
        My_d_mean = np.nanmean(var_list)
        My_d_std = np.nanstd(var_list)
        My_d_arr = var_list

        return My_d_mean,My_d_std,My_d_arr
        

    ## FLIP FUNCTIONS
    def grab_flip_time(self,k_ep,k_run):
        """Returns time of flip

        Data Type: Sim/Exp

        Args:
            k_ep (int): Episode number
            k_run (int): Run number

        Returns:
            float: t_flip,t_flip_norm
        """        

        if self.dataType == 'EXP':

            run_df,IC_df,flip_df,_ = self.select_run(k_ep,k_run)

            ## FIND INDEX OF BEFORE AND AFTER THE FLIP
            flip_index = run_df[run_df['flip_flag']==True].index.values[0]
            flip_index_prev = flip_index - 1

            ## FIND INTERPOLATION FACTOR OF TIME BETWEEN INDEXES
            z_i = run_df.iloc[flip_index_prev]['z']         # z-height before flip
            z_f = run_df.iloc[flip_index]['z']              # z-height after flip
            z_flip = self.grab_flip_state(k_ep,k_run,'z')   # z-height flip
            alpha = (z_flip - z_i)/(z_f - z_i)              # Interpolation factor

            ## FIND THE INTERPOLATED TIME
            t_i = run_df.iloc[flip_index_prev]['t']
            t_f = run_df.iloc[flip_index]['t']
            t_flip = t_i + alpha*(t_f-t_i)
            t_flip_norm = t_flip - run_df.iloc[0]['t'] # Normalize flip time

        elif self.dataType == 'SIM':

            run_df,_,flip_df,_ = self.select_run(k_ep,k_run)
            t_flip = flip_df.iloc[0]['t']
            t_flip_norm = t_flip - run_df.iloc[0]['t'] # Normalize flip time


        return t_flip,t_flip_norm

    def grab_flip_state(self,k_ep,k_run,stateName):
        """Returns desired state at time of flip

        Data Type: Sim/Exp

        Args:
            k_ep (int): Episode number
            k_run (int): Run number
            stateName (str): State name

        Returns:
            float: state_flip
        """        

        _,_,flip_df,_ = self.select_run(k_ep,k_run)
        state_flip = flip_df.iloc[0][stateName]

    
        return state_flip

    def grab_flip_eul(self,k_ep,k_run,degrees=True,External_data=False):
        """Returns euler angle data at flip trigger using [YZX] configuration to allow pitch angles greater than 180 deg

        Data Type: Sim/Exp

        Args:
            k_ep (int): Episode number
            k_run (int): Run/Rollout number
            degrees (bool, optional): Choose between degrees or radians for output. Defaults to True.
            External_data (bool, optional): Choose between internal estimate or external Mocap data. Defaults to False(Internal)

        Returns:
            eul_arr [np.array]: Returns np.array of euler values in form (n x [eul_X,eul_Y,eul_Z])
        """     
        
        ## CREATE QUAT DATAFRAME FROM RUN_DF
        
        if External_data==False:
            quat_list = ['qw','qx','qy','qz']
        else:
            quat_list = ['qw_Ext','qx_Ext','qy_Ext','qz_Ext']

        run_df,IC_df,flip_df,_ = self.select_run(k_ep,k_run)
        quat_df = flip_df.iloc[0][quat_list]

        ## CONVERT QUATS TO EULER ANGLES AND BACK TO DF
        quat_arr = quat_df.to_numpy()
        R = Rotation.from_quat(quat_arr)
        eul_arr = R.as_euler('YZX', degrees=degrees)
        eul_arr = eul_arr[np.newaxis,:] # Convert from 1-D array to 2-D array
        # Center axis [theta_2] is limited to +- 90deg while [theta_1 & theta_3] can rotate to 180deg 
        # https://graphics.fandom.com/wiki/Conversion_between_quaternions_and_Euler_angles
        # https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/Quaternions.pdf
        eul_df = pd.DataFrame(eul_arr,columns=['eul_y','eul_z','eul_x'])

        ## IF EUL_Y JUMPS FROM PAST -180 T0 +180 THEN BRING IT BACK DOWN AGAIN
        eul_df.loc[eul_df['eul_y'] > 170, 'eul_y'] = eul_df['eul_y']-360
        
        ## REARRANGE DF TO [X,Y,Z] FORMAT TO BE CONSISTENT
        eul_df = eul_df[['eul_x','eul_y','eul_z']]
        eul_arr = eul_df.to_numpy()

        return eul_arr

    def grab_flip_eul_trial(self,N:int=3,eul_type:str='eul_y',reward_cutoff:float=3.00):
        """Returns the summarized flip angle information from a given trial by finding the
        mean and standard deviation of the final 'N' episodes

        Args:
            N (int, optional): Final [N] episodes to analyze. Defaults to 3.
            eul_type (str, optional): Euler angle to analyze. Defaults to 'eul_y'.

        Returns:
            eul_flip_mean (float): Average flip angle
            eul_flip_std (float): Standard Deviation of the flip angle data
            eul_flip_arr (np.array): Array of the flip angles
        """        

        ## CREATE ARRAY OF ALL EP/RUN COMBINATIONS FROM LAST 3 ROLLOUTS
        # Use reward to extract only the valid attempts and not simulation mishaps
        ep_df = self.trial_df.iloc[:][['k_ep','k_run','reward']].astype('float').query(f'reward >= {reward_cutoff}')
        ep_arr = ep_df.iloc[-self.n_rollouts*N:].to_numpy() # Grab episode/run listing from past N rollouts

        ## ITERATE THROUGH ALL RUNS AND FINDING IMPACT ANGLE 
        var_list = []
        for k_ep,k_run in ep_arr[:,:2]:

            leg_contacts,_,_,_ = self.landing_conditions(k_ep, k_run)
            if leg_contacts >= 3: # IGNORE FAILED LANDINGS
                var_list.append(self.grab_flip_eul(k_ep,k_run))

        ## EXTRACT LIST OF DESIRED IMPACT VALUES
        if eul_type == 'eul_x':
            var_list = np.asarray(var_list)[:,0,0]
        
        elif eul_type == 'eul_y':
            var_list = np.asarray(var_list)[:,0,1]

        elif eul_type == 'eul_z':
            var_list = np.asarray(var_list)[:,0,2]

        ## COVERT NEGATIVE ANGLES INTO POSITIVE WRAP AROUND ONES (-10 deg = 350 deg)
        eul_flip_arr = var_list #np.array([360+i if i <=0 else i for i in var_list])
            
        ## RETURN MEAN AND STD AND ARRAY
        eul_flip_mean = np.nanmean(eul_flip_arr,axis=0)
        eul_flip_mean = np.nanstd(eul_flip_arr,axis=0)
        
        return eul_flip_mean,eul_flip_mean,eul_flip_arr
    
    def grab_flip_dRREV(self,k_ep,k_run,h_ceiling=2.1):
        _,_,flip_df,_ = self.select_run(k_ep,k_run)
        vz_flip = flip_df.iloc[0]['vz']
        d_flip = h_ceiling - flip_df.iloc[0]['z']

        dRREV = vz_flip**2/d_flip**2

        return dRREV
        

    ## IMPACT FUNCTIONS
    def grab_impact_time(self,k_ep,k_run,accel_threshold=-4.0):
        """Return time at impact

        Args:
            k_ep (int): Episode number
            k_run (int): Run number
            accel_threshold (float, optional): Threshold at which to declare impact. Defaults to -4.0.

        Returns:
            t_impact [float]: Time at impact
            t_impact_norm [float]: Normalized impact time 
        """        


        if self.dataType == 'EXP':
            run_df,IC_df,_,_ = self.select_run(k_ep,k_run)
            t_traj = run_df.query(f"`v_d.z` > {0.0}").iloc[0]['t']
            _,accel_df = self.grab_accel_data(k_ep,k_run)

            t_impact = accel_df.query(f"t >= {t_traj} & a_z <= {accel_threshold}").iloc[0]['t']
            t_impact_norm = t_impact - run_df.iloc[0]['t'] # Normalize impact time

        elif self.dataType == 'SIM':

            run_df,_,_,impact_df = self.select_run(k_ep,k_run)
            if impact_df.iloc[0]['reward'] == True: # If Impact Detected
                t_impact = impact_df.iloc[0]['t']
                t_impact_norm = t_impact - run_df.iloc[0]['t'] # Normalize flip time
            else:
                t_impact = np.nan
                t_impact_norm = np.nan

        return t_impact,t_impact_norm

    def grab_impact_state(self,k_ep,k_run,stateName):
        """Returns desired state at time of impact

        Data Type: Sim/Exp

        Args:
            k_ep (int): Episode number
            k_run (int): Run number
            stateName (str): State Name

        Returns:
            state_impact [float]: State at impact
        """        


        if self.dataType == 'EXP':

            run_df,IC_df,_,_ = self.select_run(k_ep,k_run)
            t_impact,t_impact_norm = self.grab_impact_time(k_ep,k_run)
            state_impact = run_df.query(f"t == {t_impact}").iloc[0][stateName]

        elif self.dataType == 'SIM':
            _,_,_,impact_df = self.select_run(k_ep,k_run)
            if impact_df.iloc[0]['reward'] == True: # If Impact Detected

                state_impact = impact_df.iloc[0][stateName]
            else:
                state_impact = np.nan


        return state_impact

    def grab_impact_eul(self,k_ep,k_run,degrees=True,External_data=False):
        """Returns euler angle data at impact using [YZX] configuration to allow pitch angles greater than 180 deg

        Data Type: Sim/Exp

        Args:
            k_ep (int): Episode number
            k_run (int): Run/Rollout number
            degrees (bool, optional): Choose between degrees or radians for output. Defaults to True.
            External_data (bool, optional): Choose between internal estimate or external Mocap data. Defaults to False(Internal)

        Returns:
            eul_arr [np.array]: Returns array of euler values in form (n x [eul_X,eul_Y,eul_Z])
        """        

        ## CREATE QUAT DATAFRAME FROM RUN_DF
        
        if External_data==False:
            quat_list = ['qw','qx','qy','qz']
        else:
            quat_list = ['qw_Ext','qx_Ext','qy_Ext','qz_Ext']


        if self.dataType == 'EXP':

            run_df,IC_df,_,_ = self.select_run(k_ep,k_run)

            ## GRAB QUAT AT T_IMPACT
            t_impact,_ = self.grab_impact_time(k_ep,k_run)
            quat_df = run_df.query(f"t=={t_impact}").iloc[0][quat_list]
            
        elif self.dataType == 'SIM':

            
            _,_,_,impact_df = self.select_run(k_ep,k_run)

            if impact_df.iloc[0]['reward'] == True: # If Impact Detected

                quat_df = impact_df.iloc[0][quat_list]
            else:
                return np.array([[np.nan,np.nan,np.nan]])
        


        ## CONVERT QUATS TO EULER ANGLES AND BACK TO DF
        quat_arr = quat_df.to_numpy()
        R = Rotation.from_quat(quat_arr)
        eul_arr = R.as_euler('YZX', degrees=degrees)
        eul_arr = eul_arr[np.newaxis,:] # Convert from 1-D array to 2-D array
        # Center axis [theta_2] is limited to +- 90deg while [theta_1 & theta_3] can rotate to 180deg 
        # https://graphics.fandom.com/wiki/Conversion_between_quaternions_and_Euler_angles
        # https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/Quaternions.pdf
        eul_df = pd.DataFrame(eul_arr,columns=['eul_y','eul_z','eul_x'])

        ## IF EUL_Y JUMPS FROM PAST -180 T0 +180 THEN BRING IT BACK DOWN AGAIN
        eul_df.loc[eul_df['eul_y'] > 170, 'eul_y'] = eul_df['eul_y']-360
        
        ## REARRANGE DF TO [X,Y,Z] FORMAT TO BE CONSISTENT
        eul_df = eul_df[['eul_x','eul_y','eul_z']]
        eul_arr = eul_df.to_numpy()

        return eul_arr

    def grab_impact_eul_trial(self,N:int=3,eul_type:str='eul_y',reward_cutoff:float=3.00,landing_cutoff:int=3):
        """Returns the summarized impact angle information from a given trial by finding the
        mean and standard deviation of the final 'N' episodes

        Args:
            N (int, optional): Final [N] episodes to analyze. Defaults to 3.
            eul_type (str, optional): Euler angle to analyze. Defaults to 'eul_y'.

        Returns:
            eul_impact_mean (float): Average impact angle
            eul_impact_std (float): Standard Deviation of the impact angle data
            eul_impact_arr (np.array): Array of the impact angles
        """        

        ## CREATE ARRAY OF ALL EP/RUN COMBINATIONS FROM LAST 3 ROLLOUTS
        # Use reward to extract only the valid attempts and not simulation mishaps
        ep_df = self.trial_df.iloc[:][['k_ep','k_run','reward']].astype('float').query(f'reward >= {reward_cutoff}')
        ep_arr = ep_df.iloc[-self.n_rollouts*N:].to_numpy() # Grab episode/run listing from past N rollouts

        ## ITERATE THROUGH ALL RUNS AND FINDING IMPACT ANGLE 
        var_list = []
        for k_ep,k_run in ep_arr[:,:2]:

            leg_contacts,_,_,_ = self.landing_conditions(k_ep, k_run)
            if leg_contacts >= landing_cutoff: # IGNORE FAILED LANDINGS
                var_list.append(self.grab_impact_eul(k_ep,k_run))

        if len(var_list) != 0:
            
            ## EXTRACT LIST OF DESIRED IMPACT VALUES
            if eul_type == 'eul_x':
                var_list = np.asarray(var_list)[:,0,0]
            
            elif eul_type == 'eul_y':
                var_list = np.asarray(var_list)[:,0,1]

            elif eul_type == 'eul_z':
                var_list = np.asarray(var_list)[:,0,2]

        ## COVERT NEGATIVE ANGLES INTO POSITIVE WRAP AROUND ONES (-10 deg = 350 deg)
        eul_impact_arr =np.array([360+i if i <=0 else i for i in var_list])
            
        ## RETURN MEAN AND STD AND ARRAY
        eul_impact_mean = np.nanmean(eul_impact_arr,axis=0)
        eul_impact_std = np.nanstd(eul_impact_arr,axis=0)
        eul_impact_list = eul_impact_arr.tolist()
        
        return eul_impact_mean,eul_impact_std,eul_impact_arr

    def grab_impact_force(self,k_ep,k_run,forceDirection='x'):
        """Returns the summarized impact force information from a given trial by finding the
        mean and standard deviation of the final 'N' episodes

        Args:
            forceDirection (str): Force direction label
            N (int, optional): Final [N] episodes to analyze. Defaults to 3.
            reward_cutoff (float, optional): The reward cutoff set to ignore failures caused directly by the simulation
            

        Returns:
            force_impact_mean (float): Max force experienced at impact

        """  
        ## CONVERT DIRECTION TO DATA LABEL
        if forceDirection == 'x':
            stateName = 'OF_x'
        elif forceDirection == 'y':
            stateName = 'OF_y'
        elif forceDirection == 'z':
            stateName = 'RREV'

        ## FIND IMPACT FORCES
        force_impact = self.grab_impact_state(k_ep,k_run,stateName)
        return force_impact


    def trigger2impact(self,k_ep,k_run):
        t_flip,_ = self.grab_flip_time(k_ep,k_run)
        t_impact,_ = self.grab_impact_time(k_ep,k_run)

        t_delta = t_impact-t_flip
        return t_delta


    def landing_conditions(self,k_ep,k_run):
        """Returns landing data for number of leg contacts, the impact leg, contact list, and if body impacted

        Args:
            k_ep (int): Episode number
            k_run (int): Run number

        Returns:
            leg_contacts (int): Number legs that joined to the ceiling
            impact_leg (int): First leg to impact the ceiling
            contact_list (list): List of the order legs impacted the ceiling
            body_impact (bool): True if body impacted the ceiling
        """        
        _,_,_,impact_df = self.select_run(k_ep,k_run)
        impact_flag = impact_df.iloc[0]['reward']
        body_impact = impact_df.iloc[0]['flip_flag']
        contact_list = impact_df.iloc[0]['impact_flag']
        contact_list = np.fromstring(contact_list[1:-1], dtype=float, sep=' ')

        leg_contacts = len(contact_list)
        if impact_flag == True and len(contact_list) > 0:
            impact_leg = contact_list[0].astype('int')
        else:
            impact_leg = 0

        return leg_contacts,impact_leg,contact_list,body_impact

    def landing_rate(self,N:int=3,reward_cutoff:float=3.00):
        """Returns the landing rate for the final [N] episodes

        Args:
            N (int, optional): Final [N] episodes to analyze. Defaults to 3.
            reward_cutoff (float, optional): Reward values below this cutoff come from sim errors and not performance. Defaults to 3.00.

        Returns:
            landing_rate_4_leg (float): Four leg landing success percentage
            landing_rate_2_leg (float): Two leg landing success percentage
            contact_rate (float): Combined landing success percentage for either two leg or four leg landing
            contact_list (list,int): List of each conctact case for each run recorded
        """        

        ## CREATE ARRAY OF ALL EP/RUN COMBINATIONS FROM LAST 3 ROLLOUTS
        # Use reward to extract only the valid attempts and not simulation mishaps
        ep_df = self.trial_df.iloc[:][['k_ep','k_run','reward']].astype('float').query(f'reward >= {reward_cutoff}')
        ep_arr = ep_df.iloc[-self.n_rollouts*N:].to_numpy() # Grab episode/run listing from past N rollouts

        ## ITERATE THROUGH ALL RUNS AND RECORD VALID LANDINGS
        four_leg_landing = 0
        two_leg_landing = 0
        one_leg_landing = 0
        missed_landing = 0
        contact_list = []
        
        for k_ep,k_run in ep_arr[:,:2]:
            
            ## RECORD LANDING CONDITIONS
            leg_contacts,_,_,_ = self.landing_conditions(k_ep, k_run)

            if leg_contacts >= 3: 
                four_leg_landing += 1

            elif leg_contacts == 2:
                two_leg_landing += 1

            elif leg_contacts == 1:
                one_leg_landing += 1

            else:
                missed_landing += 1
            
            contact_list.append(leg_contacts)

        ## CALC LANDING PERCENTAGE
        landing_rate_4leg = four_leg_landing/(N*self.n_rollouts)
        landing_rate_2leg = two_leg_landing/(N*self.n_rollouts)
        contact_rate = (four_leg_landing + two_leg_landing)/(N*self.n_rollouts)
        

        return landing_rate_4leg,landing_rate_2leg,contact_rate,np.array(contact_list)

    def grab_trial_data(self,func,*args,N:int=3,reward_cutoff:float=3.00,landing_cutoff:int=3,**kwargs):

        ## CREATE ARRAY OF ALL EP/RUN COMBINATIONS FROM LAST 3 ROLLOUTS
        # Use reward to extract only the valid attempts and not simulation mishaps
        ep_df = self.trial_df.iloc[:][['k_ep','k_run','reward']].astype('float').query(f'reward >= {reward_cutoff}')
        ep_arr = ep_df.iloc[-self.n_rollouts*N:].to_numpy() # Grab episode/run listing from past N rollouts

        ## ITERATE THROUGH ALL RUNS AND FINDING IMPACT ANGLE 
        var_list = []
        for k_ep,k_run in ep_arr[:,:2]:

            leg_contacts,_,_,_ = self.landing_conditions(k_ep, k_run)
            if leg_contacts >= landing_cutoff: # IGNORE FAILED LANDINGS
                var_list.append(func(k_ep,k_run,*args,**kwargs))

        ## RETURN MEAN AND STD OF STATE
        trial_mean = np.nanmean(var_list)
        trial_std = np.nanstd(var_list)
        trial_arr = var_list

        return trial_mean,trial_std,trial_arr
    
    def plot_state_correlation(self,stateList:list,typeList=['flip','impact'],N:int=3):

        # state_df = self.trial_df.query("Error=='Flip Data'").iloc[-int(N*self.n_rollouts):][stateList].reset_index()
        # df = self.grab_leg_contacts()
        # state_df = state_df.join(df)
        # groups = state_df.groupby("Leg_Contacts")
        # fig = plt.figure()
        # ax = fig.add_subplot(111)

        # for name,group in groups:
        #     ax.scatter(group[stateList[0]],group[stateList[1]],label=name)
        # # ax.scatter(state_df.iloc[:,0],state_df.iloc[:,1],c=df['Leg_Contacts'],marker="o", cmap="bwr_r")
        # ax.set_xlabel(stateList[0])
        # ax.set_ylabel(stateList[1])
        # ax.grid()
        # ax.legend()

        # plt.show()
        pass


        


## TRAJECTORY FUNCTIONS
    def grab_traj_start(self,k_ep,k_run):

        if self.dataType == 'EXP':
            run_df,IC_df,_,_ = self.select_run(k_ep,k_run)

            # ASSUME TRAJ STARTS WHEN PRESCRIBED VERTICAL VEL
            t_traj = run_df.query(f"`v_d.z` > {0.0}").iloc[0]['t']
            t_traj_norm = t_traj - run_df.iloc[0]['t'] # Normalize traj. start time

            return t_traj, t_traj_norm

    def grab_traj_state(self,k_ep,k_run,stateName):

        if self.dataType == 'EXP':
            run_df,IC_df,_,_ = self.select_run(k_ep,k_run)

            # ASSUME TRAJ STARTS WHEN PRESCRIBED VERTICAL VEL
            state_traj = run_df.query(f"`v_d.z` > {0.0}").iloc[0][stateName]

            return state_traj

## STUFF
    def predictAction(self,k_ep,k_run,t,IC=None,DC=None,K=None):
        stateData,DesiredState = self.grab_FullState(k_ep,k_run,t)
        
        val = self.GTC.predict(FullState=stateData,DesiredState=DesiredState,K=K)
        print(val)


    

## CONTROLLER FUNCTIONS

class GTC_Model():
    def predict(self,FullState=None,DesiredState=None,K=None):
        
        ## DEFINE SYSTEM CONSTANTS
        m = 0.037               # Mass [kg]
        g = 9.8066              # Gravity [m/s^2]

        d = 0.04                # Absolute distance from CoM to prop [m]
        dp = d*np.sin(np.pi/4)  # Projected prop distance onto x-axis [m]

        kf = 2.2e-8             # Thrust coefficient [N/(rad/s)^2] - Source: Forster
        c_tf = 0.0061           # Thrust-Moment coefficient [Nm/N]

        J = np.array([[1.65717e-5, 0, 0], # Inertia Matrix [kg*m^2]
                    [0, 1.66556e-5, 0],
                    [0, 0, 2.92617e-5]])

        e_3 = np.array([[0,0,1]]).T # Define vertical z-axis


        

        ## DEFAULT GAIN VALUES
        if K == None: 
            K = {
                "P_kp_xy": 0.4,
                "P_kd_xy": 0.245,
                "P_ki_xy": 0.3,

                "P_kp_z": 1.2,
                "P_kd_z": 0.35,
                "P_ki_z": 0.3,

                "R_kp_xy": 0.001,
                "R_kd_xy": 0.0005,
                "R_ki_xy": 0.0,

                "R_kp_z": 30e-5,
                "R_kd_z": 10e-5,
                "R_ki_z": -20e-5,
            }

            # K = {
            #     "P_kp_xy": 0.7,
            #     "P_kd_xy": 0.25,
            #     "P_ki_xy": 0.0,

            #     "P_kp_z": 0.7,
            #     "P_kd_z": 0.25,
            #     "P_ki_z": 0.0,

            #     "R_kp_xy": 0.004,
            #     "R_kd_xy": 0.0008,
            #     "R_ki_xy": 0.0,

            #     "R_kp_z": 0.004,
            #     "R_kd_z": 0.0008,
            #     "R_ki_z": 0,
            # }

        Kp_P = np.array([[K['P_kp_xy'],K['P_kp_xy'],K['P_kp_z']]]).T
        Kd_P = np.array([[K['P_kd_xy'],K['P_kd_xy'],K['P_kd_z']]]).T
        Ki_P = np.array([[K['P_ki_xy'],K['P_ki_xy'],K['P_ki_z']]]).T


        Kp_R = np.array([[K['R_kp_xy'],K['R_kp_xy'],K['R_kp_z']]]).T
        Kd_R = np.array([[K['R_kd_xy'],K['R_kd_xy'],K['R_kd_z']]]).T
        Ki_R = np.array([[K['R_ki_xy'],K['R_ki_xy'],K['R_ki_z']]]).T


        ## DEFAULT INITIAL CONDITIONS
        if FullState == None:

            FullState = {
                'x': np.array([[0.,0.,0.]]).T,       # Pos. [x,y,z] - [m]
                'v': np.array([[0.,0.,0.]]).T,       # Lin. Vel [vx,vy,vz] - [m/s]
                'w': np.array([[0.,0.,0.]]).T,       # Ang. Vel [wx,wy,wz] - [rad/s]
                'quat': np.array([[0,0,0,1.]])  # Orientation [qx,qy,qz,qw]
            }



        ## DEFAULT DESIRED STATES
        if DesiredState == None:

            DesiredState = {
                'x_d': np.array([[0.,0.,0.]]).T,        # Desired Pos. [x,y,z] - [m]
                'v_d': np.array([[0.,0.,0.]]).T,        # Desired Vel. [vx,vy,vz] - [m/s]
                'a_d': np.array([[0.,0.,0.]]).T,        # Desired Acc. [ax,ay,az] - [m/s^2]

                'omega_d': np.array([[0.,0.,0.]]).T,    # Desired Ang. Vel [wx,wy,wz] - rad/s
                'b1_d': np.array([[1.,0.,0.]]).T        # Desired body x-axis (in world FoR) [x,y,z] - rad
            }

    
        ## REDEFINE STATE VALUES
        statePos = FullState['x']
        stateVel = FullState['v']
        stateOmega = FullState['w']
        stateQuat = FullState['quat']

        x_d = DesiredState['x_d']
        v_d = DesiredState['v_d']
        a_d = DesiredState['a_d']

        # b1_d = DesiredState['b1_d']
        b1_d = np.array([[1.,0.,0.]]).T
        # omega_d = DesiredState['omega_d']
        omega_d = np.array([[0.,0.,0.]]).T
        domega_d = np.array([[0.,0.,0.]]).T

        # ROTATION MATRIX
        R = Rotation.from_quat(stateQuat).as_matrix()[0,:,:] # Trim off extra axis
        b3 = R.dot(e_3)

        # TRANSLATIONAL ERRORS AND DESIRED BODY-FIXED AXES
        e_x = statePos - x_d
        e_v = stateVel - v_d

        P_effort = -Kp_P*e_x + -Kd_P*e_v
        F_thrust_ideal = P_effort + m*g*e_3 + m*a_d

        # ROTATIONAL ERRORS 
        
        b3_d = F_thrust_ideal/np.linalg.norm(F_thrust_ideal)
        b2_d = np.cross(b3_d,b1_d,axis=0).T
        b2_d = b2_d.T/np.linalg.norm(b2_d)

        R_d = np.hstack((b1_d,b2_d,b3_d))

        e_R = 0.5*self.dehat(R_d.T @ R - R.T @ R_d).T
        e_w = stateOmega - R.T @ R_d @ omega_d


        # CONTROL EQUATIONS
        R_effort = -Kp_R*e_R + -Kd_R*e_w
        Gyro_dyn = np.cross(stateOmega,J @ stateOmega,axis=0)
        Gyro_dyn += J @ (self.hat(stateOmega) @ R.T @ R_d @ omega_d - R.T @ R_d @ domega_d)
        

        F_thrust = F_thrust_ideal.T.dot(b3)
        M = R_effort + Gyro_dyn

        

        ## CONVERT THRUST AND MOMENTS TO PWM
        f_thrust = F_thrust/4
        f_roll = M[0]/(4*dp)
        f_pitch = M[1]/(4*dp)
        f_yaw = M[2]/(4*c_tf) 

        f_thrust_pwm = self.thrust2PWM(f_thrust)
        f_thrust_pwm = np.clip(f_thrust_pwm,0,65535*0.85)
        
        f_roll_pwm = self.thrust2PWM(f_roll)
        f_pitch_pwm = self.thrust2PWM(f_pitch)
        f_yaw_pwm = self.thrust2PWM(f_yaw)
        


        f_thrust_percent = f_thrust_pwm/65535
        f_roll_percent = f_roll_pwm/65535
        f_pitch_percent = f_pitch_pwm/65535
        f_yaw_percent = f_yaw_pwm/65535



        M1_pwm = np.clip(f_thrust_pwm + f_roll_pwm - f_pitch_pwm + f_yaw_pwm,0,65535)
        M2_pwm = np.clip(f_thrust_pwm + f_roll_pwm + f_pitch_pwm - f_yaw_pwm,0,65535)
        M3_pwm = np.clip(f_thrust_pwm - f_roll_pwm + f_pitch_pwm + f_yaw_pwm,0,65535)
        M4_pwm = np.clip(f_thrust_pwm - f_roll_pwm - f_pitch_pwm - f_yaw_pwm,0,65535)

        return [F_thrust,M]



    def thrust2PWM(self,f):
        """Converts a given PWM to the corresponding thrust values

        Args:
            f (float): Thrust value for a given motor [g]

        Returns:
            f_pwm (float): PWM value
        """        
        a = 2.108e-11
        b = 1.06e-6

        s = np.sign(f)
        f = np.abs(f)

        f_pwm = s*(np.sqrt(4*a*f+b**2)/(2*a) - b/(2*a))

        return f_pwm


    def hat(self,vec):
        arr = np.zeros((3,3))
        vec = np.squeeze(vec)

        arr[0,1] = -vec[2]
        arr[0,2] =  vec[1]
        arr[1,0] =  vec[2]
        arr[1,2] = -vec[0]
        arr[2,0] = -vec[1]
        arr[2,1] =  vec[0]

        return arr

    def dehat(self,arr):
        vec = np.array([[arr[2,1],arr[0,2],arr[1,0]]])

        return vec


        



        

if __name__ == "__main__":
    import os,sys

    # sys.path.insert(1, os.path.join(sys.path[0], '/home/bhabas/catkin_ws/src/crazyflie_experiment/rl_experiment/utility'))
    # from data_analysis import DataFile

    dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/logs/"
    fileName = "EM_PEPG--Vd_2.65--phi_90.00--trial_03--EXP.csv"
    # fileName = "EM_PEPG--Vd_2.50--phi_90.00--trial_00--EXP.csv"

    trial = DataFile(dataPath,fileName)

    k_ep = 0
    k_run = 1

    np.set_printoptions(suppress=True)

    trial.plot_rewardData()

    # trial.predictAction(0,0,13.5)

    # accel,_ = trial.grab_accel_data(k_ep,k_run)
    # t,t_normalized = trial.grab_time(k_ep,k_run)

    # a_mag = np.sqrt(accel[:,0]**2 + accel[:,1]**2 + accel[:,2]**2)

    # fig = plt.figure()
    # ax = fig.add_subplot(111)

    # ax.plot(t_normalized,a_mag)
    # ax.grid()

    # plt.show()
