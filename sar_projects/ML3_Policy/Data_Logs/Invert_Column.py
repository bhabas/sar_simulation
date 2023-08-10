import pandas as pd

def invert_column_values(csv_file, column_name):
    # Load the CSV file into a DataFrame
    df = pd.read_csv(csv_file)
    
    # Multiply the specified column by -1
    df[column_name] = df[column_name] * -1
    
    # Save the updated DataFrame back to the CSV file
    df.to_csv(csv_file, index=False)

# Example usage:

path = "/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/ML3_Policy/Data_Logs/WS_LR_Trials.csv"
column_name = "Theta_x_trg_mean"
invert_column_values(path,column_name)
