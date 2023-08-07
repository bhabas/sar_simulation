
import pandas as pd
import numpy as np
from scipy.signal import find_peaks

def calculate_expected_values(I, K, C):
    # Natural Frequency
    omega_n = (K / I)**0.5
    # Damping Ratio
    zeta = C / (2 * (K * I)**0.5)
    return omega_n, zeta

def calculate_from_data(filename):
    # Load the data
    data = pd.read_csv(filename)
    
    # Find peaks in the data to determine period
    peaks, _ = find_peaks(data.iloc[:, 1])
    
    if len(peaks) < 2:
        return None, None
    
    # Calculating the average period using the time between all successive peaks
    periods = np.diff(data['sim_time'][peaks])
    T_avg = np.mean(periods)
    
    # Natural Frequency
    omega_n = 2 * np.pi / T_avg
    
    # Logarithmic decrement using the first two peaks for simplicity
    delta_ln = np.log(data.iloc[:, 1][peaks[0]] / data.iloc[:, 1][peaks[1]])
    
    # Damping Ratio
    zeta = 1 / np.sqrt(1 + (2 * np.pi / delta_ln)**2)
    
    return omega_n, zeta

if __name__ == "__main__":
    # Input parameters
    I = float(input("Enter Moment of Inertia (I): "))
    K = float(input("Enter Spring Stiffness (K): "))
    C = float(input("Enter Damping Coefficient (C): "))

    # Calculate expected values
    omega_n_expected, zeta_expected = calculate_expected_values(I, K, C)
    print(f"Expected Natural Frequency: {omega_n_expected:.3f} rad/s")
    print(f"Expected Period: {2*np.pi/omega_n_expected:.3f} s")
    print(f"Expected Damping Ratio: {zeta_expected:.3f}")

    
    # Get the filename for the data
    filename = input("Enter the path to the CSV data file (or 'skip' to bypass): ")
    if filename.lower() != 'skip':
        omega_n_observed, zeta_observed = calculate_from_data(filename)
        print(f"Observed Natural Frequency: {omega_n_observed:.3f} rad/s")
        print(f"Observed Damping Ratio: {zeta_observed:.3f}")