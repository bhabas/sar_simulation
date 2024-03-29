import numpy as np
import matplotlib.pyplot as plt
import csv

# Define the radial and angular ranges
V_mag_min = 1.2
V_mag_max = 5.2
Plane_Angle = 180


# Function to calculate the number of samples for a given radial distance
def samples_for_radius(radius, min_radius, base_radial_samples):
    # Adjust the number of samples linearly based on the radius
    return int(base_radial_samples * (radius / min_radius))


initial_test_points = []
detailed_test_points = []


## INITIAL TEST POINTS
V_angle_min = -90
V_angle_max = -10  # Degrees

base_angular_samples = 7
base_radial_samples = 10

# Sample points
for V_mag in np.linspace(V_mag_min, V_mag_max, base_radial_samples):
    num_samples = samples_for_radius(V_mag, V_mag_min, base_angular_samples)
    angles = np.linspace(V_angle_min, V_angle_max, num_samples)
    
    for V_angle in angles:

        initial_test_points.append((Plane_Angle, V_angle, V_mag))

## DETAILED TEST POINTS
V_angle_min = 0
V_angle_max = 35  # Degrees

# Base number of samples for the minimum radial distance
base_angular_samples = 10
base_radial_samples = 15

# Sample points
for V_mag in np.linspace(V_mag_min, V_mag_max, base_radial_samples):
    num_samples = samples_for_radius(V_mag, V_mag_min, base_angular_samples)
    angles = np.linspace(V_angle_min, V_angle_max, num_samples)
    
    for V_angle in angles:

        detailed_test_points.append((Plane_Angle, V_angle, V_mag))

test_points = initial_test_points + detailed_test_points
test_points = np.array(test_points)
test_points = test_points[test_points[:,2].argsort()]
test_points = test_points[test_points[:,1].argsort(kind='mergesort')]

# Convert the list of points to a NumPy array for easy plotting
initial_test_points = np.array(initial_test_points)
initial_test_points = initial_test_points[initial_test_points[:,2].argsort()]
initial_test_points = initial_test_points[initial_test_points[:,1].argsort(kind='mergesort')]

detailed_test_points = np.array(detailed_test_points)
detailed_test_points = detailed_test_points[detailed_test_points[:,2].argsort()]
detailed_test_points = detailed_test_points[detailed_test_points[:,1].argsort(kind='mergesort')]



# Writing points to a CSV file
with open('polar_points.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Plane_Angle", "V_angle", "V_mag"])  # Writing header
    test_points = np.round(test_points,3)
    writer.writerows(test_points)

# print("Points have been written to 'polar_points.csv'")

# Plot the points
plt.figure(figsize=(8, 8))
# plt.scatter(detailed_test_points[:, 2]*np.cos(np.radians(detailed_test_points[:,1])), detailed_test_points[:, 2]*np.sin(np.radians(detailed_test_points[:,1])), alpha=0.3)
plt.scatter(initial_test_points[:, 2]*np.cos(np.radians(initial_test_points[:,1])), initial_test_points[:, 2]*np.sin(np.radians(initial_test_points[:,1])))
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Uniformly Distributed Points in Polar Coordinates')
plt.axis('equal')  # Ensure equal aspect ratio for correct visualization
plt.show(block=True)
