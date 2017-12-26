import numpy as np

# define coordinates and theta
x_part= 4
y_part= 5
x_obs= 0
y_obs= -4
theta= -np.pi/2 # -90 degrees

# transform to map x coordinate
x_map= x_part + (np.cos(theta) * x_obs) - (np.sin(theta) * y_obs)

# transform to map y coordinate
y_map= y_part + (np.sin(theta) * x_obs) + (np.cos(theta) * y_obs)

print(int(x_map), int(y_map)) # (0,5)