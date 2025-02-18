import numpy as np
import control
import matplotlib.pyplot as plt

Ct = 0.13602
Cq = 0.055555
R = 0.127 # meter
A = 0.0507 # meter^2
rho = 1.184 # kg/m^3
l = 0.225 # meter

b = Ct * rho * A * R**2
k = Cq * rho * A * R**3

u1 = 0.0
u2 = 173.0
u3 = 0.0
u4 = 0.0
ch_throttle = 1000

conversion_matrix = np.array([[b, b, b, b],
                              [-l*b, l*b, l*b, -l*b],
                              [-l*b, -l*b, l*b, l*b],
                              [k, -k, k, -k]])

conversion_matrix_thrust = ([[  1,        1,      1,       1     ],
                             [ -1,        1,      1,      -1     ],
                             [  1,        1,     -1,      -1     ],
                             [  0.0519,  -0.0519, 0.0519, -0.0519]])

conversion_matrix_a = np.array([[292600, -1300300,  1300300,  6283300],
                                [292600, -1300300, -1300300, -6283300],
                                [292600,  1300300, -1300300,  6283300],
                                [292600,  1300300,  1300300, -6283300]])

conversion_matrix_b = np.array([[ 8.54408749e-07,  8.54408749e-07,  8.54408749e-07,  8.54408749e-07],
                                [ -1.92263324e-07,  1.92263324e-07, 1.92263324e-07, -1.92263324e-07],
                                [ -1.92263324e-07, -1.92263324e-07, 1.92263324e-07,  1.92263324e-07],
                                [ 3.97880095e-08, -3.97880095e-08,  3.97880095e-08, -3.97880095e-08]])


conversion_matrix_invers = np.linalg.inv(conversion_matrix_b)

omega2 = (np.dot(conversion_matrix_invers, [u1, u2/1000000, u3/1000000, u4/1000000]))

# omega = np.sqrt(omega2)

pwm = np.clip((ch_throttle + omega2), 1000, 1800)

# print(conversion_matrix)

# print(k)
# print(b)

print(omega2)
# print(omega)
print(pwm)

# print(conversion_matrix)

print(conversion_matrix_invers)

identity_matrix = np.dot(conversion_matrix, conversion_matrix_invers)
print(identity_matrix)
