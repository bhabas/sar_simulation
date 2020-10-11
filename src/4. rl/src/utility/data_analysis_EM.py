import pandas as pd
import numpy as np
import pandas
from math import isnan
import matplotlib.pyplot as plt
import os
print(os.getcwd())
import time
import numpy as np
import ast
def load_csv(self,datapath,k_ep):
    ## Load csv and seperate first run of the selected episode
    df = pd.read_csv(datapath)
    ep_data = df.loc[ (df['k_ep'] == k_ep) & (df['k_run'] == 0)]
    
    ## Create a list of the main values to be loaded
    vals = []
    for k in range(2,6):
        val = ep_data.iloc[0,k]
        val_edited = np.fromstring(val[2:-2], dtype=float, sep=' ')
        val_array = val_edited.reshape(1,-1).T
        vals.append(val_array)

    alpha_mu,alpha_sig, mu, sigma = vals
    return alpha_mu,alpha_sig,mu,sigma

def clean_data(data):
    end = data[data['reward'] !=0].reset_index()
    #print(end)
    for index, row in end.iterrows(): # clean data
        if isnan(row['reward']):
            if row['k_run'] > 0:
                pass
                #end.drop(index-1,inplace=True)
            end.drop(index,inplace = True)
    end = end.reset_index()
    end['k_ep'] = end['k_ep'].astype(int)
    return end
        
def get_average_reward(data,maxindex):
    
    k_ep = (max(data['k_ep']))
    reward_avg =[]
    #print(data)
    for i in range(0,maxindex):#k_ep): # 1 less than final
        rr = data[data['k_ep']==i]
        #print(rr['reward'])
        for j in range(10-len(rr)):
            rr = rr.append({'reward': 0}, ignore_index=True)
        #print(rr['reward'])
        reward_avg.append(rr['reward'].mean()) 
    return reward_avg

def get_mu(data,maxindex):
    k_ep = max(data['k_ep'])

    mudata = data['mu']
    mu = []
    for i in range(0,maxindex):
        mu.append(mudata[i*10])
    #print(mu)
    return mu

def get_sigma(data,maxindex):
    k_ep = max(data['k_ep'])
    sigma = []
    sigmadata = data['sigma']
    for i in range(0,maxindex):
        sigma.append(sigmadata[i*10])
    #print(mu)
    return sigma

def extract_sigma(sigma,xy=0):
    sx = []
    sy = []
    sxy = []
    for count,s in enumerate(sigma):
        #print(s)
        x = np.fromstring(s[2:-2], dtype=float, sep=' ')
        sx.append(x[0])
        sy.append(x[1])
        if xy==1:
            sxy.append(x[2])
    if xy ==1:
        return sx,sy,sxy
    else:
        return sx,sy
        

def extract_mu(mu):
    mux = []
    muy = []
    for count,m in enumerate(mu):
        #print(s)
        x = np.fromstring(m[2:-2], dtype=float, sep=' ')
        mux.append(x[0])
        muy.append(x[1])

    return mux,muy
#f_EM = 'EM_929'
f_EM = 'em_sameIC'
f_EM = 'em_55good'
f_EM = 'EM_final'
f_EM = 'PEPGadaptive55'

#f_EM = 'PEPGadaptive1_exp'




path_EM = '/home/bader/catkin_ws/src/crazyflie_simulation/src/4. rl/src/log/%s.csv' %(f_EM)

#f_EMsym = 'EM_sys_929'
f_EMsym = 'emsym_sameIC'
f_EMsym = 'emsym_55good'
f_EMsym = 'EMsym_final'
#f_EMsym = 'PEPGadaptive1baseline_exp'

path_EMsym = '/home/bader/catkin_ws/src/crazyflie_simulation/src/4. rl/src/log/%s.csv' %(f_EMsym)

#f_EMcov = 'EM_cov_fixed929'
f_EMcov = 'emcov_sameIC'
f_EMcov = 'emcov_55_good'
f_EMcov = 'EMcov_finalmidfail'
f_EMcov = 'EMadaptivecov55'
#f_EMcov = 'EMcovadaptive_rightexp'


path_EMcov = '/home/bader/catkin_ws/src/crazyflie_simulation/src/4. rl/src/log/%s.csv' %(f_EMcov)

df_EM = pd.read_csv(path_EM)
df_EMsym = pd.read_csv(path_EMsym)
df_EMcov = pd.read_csv(path_EMcov)
#print(df_EM)
'''
end = df_EM[df_EM['reward'] !=0].reset_index()
for index, row in end.iterrows(): # clean data
    if isnan(row['reward']):
        if row['k_run'] > 0:
            end.drop(index-1,inplace=True)
        end.drop(index,inplace = True)
        
end = end.reset_index()'''

end = clean_data(df_EM)
#print(end)
end_EMsym = clean_data(df_EMsym)
end_EMcov = clean_data(df_EMcov)

max_index = 10
r = get_average_reward(end,max_index)
rsym = get_average_reward(end_EMsym,max_index)
rcov = get_average_reward(end_EMcov,max_index)


sigma = get_sigma(end,max_index)
sigmasym = get_sigma(end_EMsym,max_index)
sigmacov = get_sigma(end_EMcov,max_index)

s1x,s1y = extract_sigma(sigma)
s2x,s2y = extract_sigma(sigmasym)
s3x,s3y,s3xy = extract_sigma(sigmacov,1)

mu = get_mu(end,max_index)
musym = get_mu(end_EMsym,max_index)
mucov = get_mu(end_EMcov,max_index)

mu1x ,mu1y = extract_mu(mu)
mu2x ,mu2y = extract_mu(musym)
mu3x ,mu3y = extract_mu(mucov)

'''
mu = [[[ 5. -5.]], [[ 4.09 -4.55]], [[ 4.95 -4.86]], [[ 5.31 -4.66]], [[ 5.56 -4.77]], '[[ 5.51 -4.68]]', '[[ 5.7  -4.75]]', '[[ 5.52 -4.6 ]]', '[[ 5.36 -4.51]]', '[[ 5.58 -4.39]]', '[[ 5.56 -4.51]]', '[[ 5.56 -4.54]]', '[[ 5.64 -4.58]]', '[[ 5.66 -4.67]]', '[[ 5.62 -4.73]]', '[[ 5.48 -4.74]]', '[[ 5.36 -4.73]]', '[[ 5.4  -4.68]]', '[[ 5.69 -4.66]]', '[[ 5.65 -4.72]]', '[[ 5.98 -4.67]]', '[[ 6.14 -4.59]]', '[[ 6.29 -4.59]]', '[[ 6.43 -4.54]]', '[[ 6.44 -4.52]]', '[[ 6.33 -4.56]]', '[[ 6.17 -4.6 ]]']

mu1x = np.array([5,4.09,4.95,5.31,5.56,5.51,5.7,5.52,5.36,5.58,5.56,5.56,5.64,5.66,5.62,5.48,5.36,5.4,5.69,5.65,5.98])#,6.14,6.29,6.43,6.44,6.33,6.17]
mu1y = np.array([5,4.55,4.86,4.66,4.77,4.68,4.75,4.6,4.51,4.39,4.51,4.54,4.58,4.67,4.73,4.74,4.73,4.68,4.66,4.72,4.67])#,4.59,4.59,4.54,4.52,4.56,4.6]

mu2x = np.array([5,4.91,4.83,4.85,4.88,4.91,4.91,4.92,4.92,4.92,4.92,4.94,4.94,4.95,4.94,4.95,4.95,4.96,4.97,4.97,4.97])
mu2y = np.array([5,5.11,5.14,4.92,5.41,5.35,5.39,5.4,5.41,5.42,5.41,5.42,5.44,5.43,5.43,5.43,5.44,5.43,5.43,5.43,5.43])

mu3x = np.array([5,4.89,4.86,5.02,5.26,5.31,5.27,5.14,5.1,4.95,4.92,4.79,4.81,5.02,5.01,5.13,5.02,5.03,5.09,5.13,5.15])
mu3y = np.array([5,5.14,5.26,5.73,6.08,6.17,6.13,5.94,5.88,5.68,5.63,5.45,5.48,5.76,5.74,5.91,5.76,5.77,5.87,5.92,5.94])


s1x = np.array([1,1.15,0.93,0.41,0.34,0.26,0.25,0.28,0.26,0.3,0.19,0.25,0.24,0.27,0.39,0.3,0.24,0.34,0.56,0.48,0.47])
s1y = np.array([1,1.1,0.84,0.38,0.17,0.16,0.13,0.19,0.21,0.2,0.14,0.12,0.14,0.13,0.2,0.25,0.14,0.09,0.12,0.16,0.12])

s2x = np.array([1,0.86,0.43,0.32,0.23,0.06,0.08,0.04,0.01,0.02,0.03,0.05,0.03,0.02,0.02,0.03,0.05,0.05,0.03,0.04,0.03])
s2y = np.array([1,0.49,0.9,0.69,0.68,0.56,0.3,0.07,0.05,0.05,0.04,0.04,0.04,0.02,0.01,0.01,0.01,0.009,0.009,0.01,0.01])

s3x = np.array([1,0.63,0.44,0.42,0.67,0.61,0.26,0.29,0.35,0.3,0.18,0.18,0.15,0.24,0.16,0.14,0.18,0.11,0.1,0.09,0.11])
s3y =np.array( [1,0.61,0.54,0.75,1.03,0.9,0.37,0.4,0.49,0.43,0.25,0.26,0.22,0.34,0.23,0.2,0.26,0.16,0.14,0.14,0.16])
s3xy = np.array([0,-0.35,-0.19,-0.27,-0.69,-0.55,-0.09,-0.12,-0.17,-0.13,-0.05,-0.05,-0.03,-0.08,-0.04,-0.03,-0.05,-0.02,-0.01,-0.01,-0.02])

# cant get mu??
#print(mucov)
mu1x = [3 , 3.15  , 3.58,3.84, 3.84  , 3.9,3.92,3.9,3.88,3.86,3.85,3.87,3.88,3.9,3.93,3.96,3.98,3.95,3.96,3.96,3.99]
mu1y = [-3 , -3.46 ,-3.42 ,-3.09,-3.04,-2.8,-3.22,-4.06,-4.57,-4.06,-4.45,-4.44,-4.44,-4.42,-4.42,-4.45,-4.41,-4.45,-4.48,-4.49,-4.51]

mu2x = [3,3.22,3.57,3.69,3.74,3.9,3.89,3.93,3.84,3.87,3.86,3.85,3.84,3.82,3.82,3.82,3.81,3.81,3.82,3.81,3.81]   
mu2y = [-3,-3.34,-3.31,-3.65,-3.54,-4,-4.02,-4.15,-4.16,-4.2,-4.15,-4.13,-4.14,-4.1,-4.18,-4.21,-4.22,-4.23,-4.23,-4.24,-4.26]


mu3x = [3,3.23,3.49,3.71,3.78,3.8,3.89,3.81,3.78,3.74,3.75,3.92,4.19,4.12,4.14,4.15,4.13,4.21,4.24,4.24,4.26  ]
mu3y = [-3,-3.37,-3.46,-3.73,-3.81,-3.85,-3.79,-3.87,-3.95,-3.96,-3.97,-4.11,-4.35,-4.28,-4.3,-4.31,-4.29,-4.37,-4.4,-4.4,-4.43]

mu1x = [3,3.12,3.38,4.39,4.61,5.12,4.9,5.03,5.03,5.07]
mu1y = [3,3.06,3.1,3.4,3.43,3.55,3.48,3.73,3.76,3.83]

mu2x = [3,4.08,4.29,4.31,4.44,4.97,4.74,4.73,4.81,4.78,4.76,4.79,4.8,4.8,4.79,4.79,4.81,4.8,4.79,4.79,4.79,4.79,4.79,4.79]
mu2y = [3,2.71,3.25,4.12,5.23,5.27,5.11,5.04,5.03,4.95,5.27,5.14,5.08,5.09,5.06,5.08,5.08,5.11,5.1,5.13,5.11,5.11,5.1,5.09]

mu3x = [3,3.42,3.46,3.77,3.94,3.9,3.93,3.94,3.95,3.95,3.95,3.97]
mu3y = [3,3.69,3.48,4.02,4.24,4.17,4.33,4.38,4.4,4.43,4.4,4.46]'''

#print(mu[1].size)
#sigma
#sigmasym
#sigmacov
plt.figure(1)

plt.subplot(231)

plt.plot(r,'b-')
#plt.plot(rsym,'r-')
plt.plot(rcov,'g-')
plt.title('Reward')
#plt.legend(('EM','EMsym', 'EMcov'))
plt.legend(('Adaptive PEPG', 'Adaptive EMCOV'))
#plt.legend(('Adaptive PEPG', 'Adaptive PEPG b', 'Adaptive EMCOV'))



plt.subplot(232)
plt.plot(mu1x,'b-.')
#plt.plot(mu2x,'r-.')
plt.plot(mu3x,'g-.')
plt.title('$\mu_{RREV}$')

#plt.legend(('EM','EM Symmetric', 'EM Covariance'))

#plt.figure(3)
plt.subplot(233)
plt.plot((mu1y),'b-.')
#plt.plot((mu2y),'r-.')
plt.plot((mu3y),'g-.')
plt.title('$\mu_{gain rate}$')


plt.subplot(234)
plt.plot((s1x),'b--')
#plt.plot((s2x),'r--')
plt.plot((s3x),'g--')
plt.title('$\sigma_x$')


plt.subplot(235)
plt.plot((s1y),'b--')
#plt.plot((s2y),'r--')
plt.plot((s3y),'g--')
plt.title('$\sigma_y$')

plt.subplot(236)
plt.plot(s3xy,'g--')
plt.title('$\sigma_{xy}$')

'''
plt.figure(2)
figure, axes = plt.subplots()
draw_circle1 = plt.Circle((mu1x[-1], mu1y[-1]), 0.3,fill=False)

axes.set_aspect(1)
axes.add_artist(draw_circle)
plt.title('Circle')'''
plt.show()#plt.legend(('EM','EM Symmetric', 'EM Covariance'))
