import numpy as np
import matplotlib.pyplot as plt
import control as ctl

def compute_parameters(K, T, Td, ωc):
    a1 = 1 + T**2 * ωc**2 + 2 * T * Td * ωc**2
    a2 = ωc**2 * (T - Td) * (T * Td * ωc**2 - 1)
    
    # Solve for Ti using equation (44)
    Ti_possibilities = (-a1 + np.sqrt(a1**2 + 4 * a2 * T)) / (2 * a2), \
                       (-a1 - np.sqrt(a1**2 + 4 * a2 * T)) / (2 * a2)
    
    # Choose the positive Ti as it represents a time constant
    Ti = np.max(Ti_possibilities)
    
    # Solve for Kp using equation (43) and the positive Ti
    Kp = Ti * ωc * np.sqrt(1 + T**2 * ωc**2) / (K * np.sqrt((1 - Ti * Td * ωc**2)**2 + Ti**2 * ωc**2))
    
    return Kp, Ti

def validate_parameters(Kp, Ti, K, T, Td):
    try:
        # Validate conditions for controller stability and performance
        if not (0 < Td <= T) or Kp <= 0 or Ti <= 0:
            raise ValueError("Inequality (17a) not satisfied.")
        if not (Ti >= 2 * K * Kp * T / (1 + 2 * Kp)):
            raise ValueError("Inequality (17b) not satisfied.")
        if not (Ti >= 2 * T * Td * (T + K * Kp * Td) / ((T - Td) * (T + Td + 2 * K * Kp * Td))):
            raise ValueError("Inequality (17c) not satisfied.")
    except ValueError as e:
        print(e)
        return False
    return True

def plot_response(K, T, Td, ωc):
    Kp, Ti = compute_parameters(K, T, Td, ωc)
    if validate_parameters(Kp, Ti, K, T, Td):
        numG = [K]
        denG = [T, 1]
        plant = ctl.tf(numG, denG)
        
        # PID Controller C(s) = Kp * (1 + 1/(Ti*s) + Td*s)
        numC = [Kp*Td, Kp, Kp/Ti]
        denC = [1, 0]
        controller = ctl.tf(numC, denC)
        
        # Closed-loop system from the feedback of plant and controller
        closed_loop = ctl.feedback(controller*plant)
        
        print("Parameters Used:")
        print(f"K = {K}, T = {T}, Td = {Td}, ωc = {ωc}")
        print("Calculated Kp:", Kp)
        print("Calculated Ti:", Ti)
        print("Plant Transfer Function:", plant)
        print("Controller Transfer Function:", controller)
        print("Closed-loop Transfer Function:", closed_loop)
        
        # Step response of the closed-loop system
        time, response = ctl.step_response(closed_loop)
        plt.figure()
        plt.plot(time, response)
        plt.title('Step Response of the Closed-Loop System')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Output')
        plt.grid(True)
        plt.show()
    else:
        print("Failed to validate parameters. Adjust your inputs and try again.")

if __name__ == "__main__":
    print("Enter the system parameters:")
    K = float(input("Gain K (e.g., 1.0): "))
    T = float(input("Time constant T (e.g., 1.0): "))
    Td = float(input("Derivative time Td (e.g., 0.2): "))
    ωc = float(input("Cutoff frequency ωc (e.g., 1.0): "))
    plot_response(K, T, Td, ωc)
