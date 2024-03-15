import numpy as np
from math import cos, sin, pi
from numpy.linalg import inv

def sph_to_cart(epsilon = 5, alpha = 10, r = 4):
  """
  Transform sensor readings to Cartesian coordinates in the sensor
  frame. The values of epsilon and alpha are given in radians, while 
  r is in metres. Epsilon is the elevation angle and alpha is the
  azimuth angle (i.e., in the x,y plane).
  """
  p = np.zeros(3)  # Position vector 

  # transform sperical coordinates to cartesian coordinates
  p[0] = r * cos(alpha) * cos(epsilon) # x
  p[1] = r * sin(alpha) * cos(epsilon) # y
  p[2] = r * sin(epsilon) # z
  
  return p
  
def estimate_params(P):
  """
  Estimate parameters from sensor readings in the Cartesian frame.
  Each row in the P matrix contains a single 3D point measurement;
  the matrix P has size n x 3 (for n points). The format is:
  
  P = [[x1, y1, z1],
       [x2, x2, z2], ...]
       
  where all coordinate values are in metres. Three parameters are
  required to fit the plane, a, b, and c, according to the equation
  
  z = a + bx + cy
  
  The function should return the parameters as a NumPy array of size
  three, in the order [a, b, c]
  """
  param_est = np.zeros(3)
  a = np.ones_like(P)
  b = np.zeros((P.shape[0], 1))

  # matrixfy the sensor readings
  for i, measurement in enumerate(P):
    a[i, 1], a[i, 2], b[i] = measurement

  z = inv(a.T @ a) @ a.T @ b

  # Your code here
  param_est[0] = z[0,0]
  param_est[1] = z[1,0]
  param_est[2] = z[2,0]
  
  return param_est

sample_measurement = np.array([[pi/3, 0, 5],
                 [pi/4, pi/4, 7],
                 [pi/6, pi/2, 4],
                 [pi/5, 3*pi/4, 6],
                 [pi/8, pi, 3]])
P = np.array([sph_to_cart(*row) for row in sample_measurement])
print(estimate_params(P))

# convert degrees to radians! 10, and 5
x = 4 * cos(0.17453292) * cos(0.08726646) # x
y = 4 * sin(0.17453292) * cos(0.08726646) # y
z = 4 * sin(0.08726646) # z

print(x, y, z)