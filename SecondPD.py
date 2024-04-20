import numpy as np
import control as ctl
import matplotlib.pyplot as plt

def calculate_controller_parameters(K, T, Td, ωc):
    # Calculate Kp using the given formula (equation 46)
    Kp = ωc * np.sqrt(1 + T**2 * ωc**2) / (K * np.sqrt(1 + Td**2 * ωc**2))
    return Kp

def check_td_condition(K, Kp, T, Td):
    # Check if the PD controller parameters satisfy the condition (26)
    return Td >= (2 * K * Kp * T - 1) / (2 * K * Kp)

def plot_step_response(K, T, Td, ωc):
    Kp = calculate_controller_parameters(K, T, Td, ωc)
    if check_td_condition(K, Kp, T, Td):
        # Define the transfer functions for the plant and controller
        numG = [K]
        denG = [T, 1, 0]
        plant = ctl.tf(numG, denG)
        
        numC = [Kp * Td, Kp]
        denC = [1]
        controller = ctl.tf(numC, denC)
        
        # The closed-loop transfer function from the feedback of plant and controller
        closed_loop = ctl.feedback(controller * plant)
        
        # Print all the parameters and transfer functions
        print("Parameters Used:")
        print(f"K = {K}, T = {T}, Td = {Td}, ωc = {ωc}")
        print("Calculated Kp:", Kp)
        print("Plant Transfer Function:", plant)
        print("Controller Transfer Function:", controller)
        print("Closed-loop Transfer Function:", closed_loop)
        
        # Simulate step response
        time, response = ctl.step_response(closed_loop)
        
        # Plotting
        plt.figure()
        plt.plot(time, response)
        plt.title('Step Response of the Closed-Loop System')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Output')
        plt.grid(True)
        plt.show()
    else:
        print(f"The chosen Td does not satisfy the condition Td >= (2*K*Kp*T - 1)/(2*K*Kp). Adjust Td.")

if __name__ == "__main__":
    print("Enter the system parameters:")
    K = float(input("Gain K (e.g., 1.0): "))
    T = float(input("Time constant T (e.g., 1.0): "))
    Td = float(input("Derivative time Td (e.g., 2.0): "))
    ωc = float(input("Cutoff frequency ωc (e.g., 2): "))
    plot_step_response(K, T, Td, ωc)
