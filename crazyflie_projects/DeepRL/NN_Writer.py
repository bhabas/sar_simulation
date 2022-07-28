import torch
import numpy as np
import datetime
from stable_baselines3 import SAC

from CF_Env_2D import CF_Env_2D
from CF_Env_2D_dTau import CF_Env_2D_dTau
from CF_Env_2D_Simple import CF_Env_2D_Simple


def save_NN_Params(self,SavePath,FileName):
    f = open(SavePath,'a')
    f.truncate(0) ## Clears contents of file

    date_time = datetime.now().strftime('%m/%d-%H:%M')
    f.write(f"// Filename: {FileName} Time: {date_time}\n")
    f.write("static char NN_Params_Flip[] = {\n")
    
    NN_size = np.array([4]).reshape(-1,1)

    ## SAVE SCALER ARRAY VALUES
    np.savetxt(f,NN_size,
                fmt='"%.0f,"',
                delimiter='\t',
                comments='',
                header=f'"{NN_size.shape[0]},"\t"{NN_size.shape[1]},"',
                footer='"*"\n')

    ## EXTEND SCALER ARRAY DIMENSIONS
    scaler_means = self.scaler.mean_.reshape(-1,1)
    scaler_stds = self.scaler.scale_.reshape(-1,1)
    
    ## SAVE SCALER ARRAY VALUES
    np.savetxt(f,scaler_means,
                fmt='"%.5f,"',
                delimiter='\t',
                comments='',
                header=f'"{scaler_means.shape[0]},"\t"{scaler_means.shape[1]},"',
                footer='"*"\n')

    np.savetxt(f,scaler_stds,
                fmt='"%.5f,"',
                delimiter='\t',
                comments='',
                header=f'"{scaler_stds.shape[0]},"\t"{scaler_stds.shape[1]},"',
                footer='"*"\n')

    ## SAVE NN LAYER VALUES
    with torch.no_grad():
        ii = 0
        for name, layer in self.NN_model.named_modules():
            if ii > 0: # Skip initialization layer

                W = layer.weight.numpy()
                np.savetxt(f,W,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{W.shape[0]},"\t"{W.shape[1]},"',
                    footer='"*"\n')


                b = layer.bias.numpy().reshape(-1,1)
                np.savetxt(f,b,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{b.shape[0]},"\t"{b.shape[1]},"',
                    footer='"*"\n')

            ii+=1

    f.write("};")
    f.close()

if __name__ == '__main__':

    ## INITIATE ENVIRONMENT
    env = CF_Env_2D()

    ## CREATE MODEL AND LOG DIRECTORY
    log_dir = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/DeepRL/logs/{env.env_name}"
    log_name = f"SAC-10-37_0"
    model_path = os.path.join(log_dir,log_name,f"models/{96}000_steps.zip")
    model = SAC.load(model_path,env=env,device='cpu')