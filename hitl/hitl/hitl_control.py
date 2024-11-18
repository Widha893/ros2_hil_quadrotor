import numpy as np
import control


m = 1.477
g = 9.8065
# Ixx = 0.014641566666666668
# Iyy = 0.014641566666666668
# Izz = 0.026640733333333336
Ixx = 0.1152
Iyy = 0.1152
Izz = 0.218

A = np.array([[0, 1, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 1, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 1],
              [0, 0, 0, 0, 0, 0, 0, 0]])

B = np.array([[0, 0, 0, 0],
              [1/m, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 1/Ixx, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 1/Iyy, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 1/Izz]])

C = np.identity(8)

D = np.array([[0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0],
              [0, 0, 0, 0]])

sys = control.ss(A, B, C, D)

def LQR():
    Q = np.diag([100, 1, 25, 17, 30, 8, 9, 1])
    R = np.diag([1, 1, 1, 1])
    K, S, E = control.lqr(sys, Q, R)
    return K   

def main(args=None):
    K = LQR()
    print(f"Gain alt: {K[0][0]:.3f}")
    print(f"Gain vz: {K[0][1]:.3f}")
    print(f"Gain roll: {K[1][2]:.3f}")
    print(f"Gain p: {K[1][3]:.3f}")
    print(f"Gain pitch: {K[2][4]:.3f}")
    print(f"Gain q: {K[2][5]:.3f}")
    print(f"Gain yaw: {K[3][6]:.3f}")
    print(f"Gain r: {K[3][7]:.3f}")

if __name__ == '__main__':
    main() 