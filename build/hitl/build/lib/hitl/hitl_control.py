import numpy as np
import control
import matplotlib.pyplot as plt

# Constants
m = 1.477
g = 9.8065
Ixx = 0.1152
Iyy = 0.1152
Izz = 0.218
# Ixx = 0.014891
# Iyy = 0.015712
# Izz = 0.027364271

# State-space matrices
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

# Define the system
sys = control.ss(A, B, C, D)

# LQR Controller
def LQR():
    Q = np.diag([100, 1, 50, 1, 40, 0.5, 50, 0.5])  # State cost
    R = np.diag([1, 1, 1, 1])  # Input cost
    K, S, E = control.lqr(sys, Q, R)
    print("\nLQR Gain Matrix (K):")
    print("alt : ", K[0, 0])
    print("vz : ", K[0, 1])
    print("roll : ", K[1, 2])
    print("p : ", K[1, 3])
    print("pitch : ", K[2, 4])
    print("q : ", K[2, 5])
    print("yaw : ", K[3, 6])
    print("r : ", K[3, 7])
    return K

# Simulate the system response
def simulate():
    K = LQR()

    # Create the closed-loop system (A - B*K)
    Ac = A - B @ K
    Bc = B
    Cc = C
    Dc = D

    sys_cl = control.ss(Ac, Bc, Cc, Dc)
    
    # Initial conditions: Small perturbations in roll, pitch, and yaw (in degrees)
    roll_init_deg = 15  # initial roll in degrees
    pitch_init_deg = 15  # initial pitch in degrees
    yaw_init_deg = 15  # initial yaw in degrees
    
    # Convert the initial conditions from degrees to radians
    x0 = np.array([0, 0, np.radians(roll_init_deg), 0, np.radians(pitch_init_deg), 0, np.radians(yaw_init_deg), 0])
    
    # Simulate the step response for roll, pitch, and yaw
    time = np.linspace(0, 10, 1000)  # Set simulation time to 20 seconds
    T, yout = control.forced_response(sys_cl, T=time, X0=x0, U=np.zeros((4, len(time))))
    
    # Extract the roll, pitch, and yaw (states 3, 4, and 5)
    roll = np.degrees(yout[3, :])  # Convert roll from radians to degrees
    pitch = np.degrees(yout[4, :])  # Convert pitch from radians to degrees
    yaw = np.degrees(yout[5, :])  # Convert yaw from radians to degrees

    # Plot the results
    plt.figure(figsize=(10, 6))
    plt.subplot(3, 1, 1)
    plt.plot(T, roll, label="Roll")
    plt.axhline(0, color='r', linestyle='--', label="Setpoint (0)")
    plt.title("Roll Response")
    plt.xlabel("Time [s]")
    plt.ylabel("Roll [deg]")
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(T, pitch, label="Pitch")
    plt.axhline(0, color='r', linestyle='--', label="Setpoint (0)")
    plt.title("Pitch Response")
    plt.xlabel("Time [s]")
    plt.ylabel("Pitch [deg]")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(T, yaw, label="Yaw")
    plt.axhline(0, color='r', linestyle='--', label="Setpoint (0)")
    plt.title("Yaw Response")
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw [deg]")
    plt.legend()

    plt.tight_layout()
    plt.show()

def main(args=None):
    simulate()

if __name__ == '__main__':
    main()
