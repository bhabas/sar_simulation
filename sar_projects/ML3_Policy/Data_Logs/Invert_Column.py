import pandas as pd
import os

def invert_column_values(csv_file, column_name):
    # Load the CSV file into a DataFrame
    df = pd.read_csv(csv_file)
    
    # Check if column exists in dataframe
    if column_name not in df.columns:
        print(f"'{column_name}' not found in {csv_file}. Skipping...")
        return
    
    def invert_value(val):
        try:
            return -float(val)
        except (ValueError, TypeError):
            return val

    df[column_name].iloc[:-3] = df[column_name].iloc[:-3].apply(invert_value)
    
    
    
    # Save the updated DataFrame back to the CSV file
    df.to_csv(csv_file, index=False)

def process_all_csv_files_in_folder(folder_path, column_name):
    # List all files in the folder
    files = os.listdir(folder_path)
    
    # Filter out only CSV files
    csv_files = [f for f in files if f.endswith('.csv')]
    
    # Process each CSV file
    for csv_file in csv_files:
        full_path = os.path.join(folder_path, csv_file)
        invert_column_values(full_path, column_name)
# Example usage:

dir_path = "/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/ML3_Policy/Data_Logs/ML3_Experimental_Data/A_Two-Leg/No_Body_Collsion"
column_name = "Theta_x"
process_all_csv_files_in_folder(dir_path, column_name)


# csv_path = "/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/ML3_Policy/Data_Logs/ML3_Experimental_Data/A_Four-Leg/No_Body_Collsion/ML3--NL_2.25_40.00_10-26_17:23.csv"
# column_name = "Theta_x"
# invert_column_values(csv_path,column_name)