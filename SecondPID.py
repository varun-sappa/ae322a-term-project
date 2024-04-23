# import warnings
# warnings.filterwarnings('ignore')

import numpy as np
import control as ctl
import matplotlib.pyplot as plt
import random

def validate_pid_parameters(K, A, B, Kp, Ti, Td, ωc):
    try:
        if Kp is None or Ti is None:
            return False  # Skip validation if parameters are None

        # Check if Kp, Ti, and Td are positive
        if Kp <= 0 or Ti <= 0 or Td <= 0:
            raise ValueError("All parameters Kp, Ti, and Td must be positive.")

        # Additional inequalities for validation
        if Kp <= (A**2 - 2 * B) / (2 * K):
            raise ValueError("Parameter Kp does not satisfy the required inequality.")
        if Ti <= (2 * A * K * Kp / (B**2 + 2 * B * K * Kp)):
            raise ValueError("Parameter Ti does not satisfy the required inequality.")

        # Specific condition for Td
        calculated_Td = (A * K * Kp / (B**2 + 2 * B * K * Kp)) + (
            np.sqrt(A**2 * K**2 * Kp**2 + B * (B + 2 * K * Kp) * (A**2 - 2 * B - 2 * K * Kp)) / (B**2 + 2 * B * K * Kp))
        if Td >= calculated_Td:
            raise ValueError("Parameter Td does not satisfy the required inequality.")

    except ValueError as error:
        # print(error)
        return False
    
    return True

def calculate_Kp_Ti(ωc, K, A, B, Td):
    b0 = A * (B + ωc**2) / ((B - ωc**2)**2 + A**2 * ωc**2)
    b1 = -1 * (1 + 2 * b0 * Td * ωc**2)
    b2 = ωc**2 * (b0 * (1 + Td * ωc**2) - Td)

    # Calculate discriminant to check for real solutions
    discriminant = b1**2 - 4 * b0 * b2
    if discriminant < 0 or b2 == 0:
        return None, None  # No valid Ti can be calculated if discriminant is negative or b2 is zero

    Ti_candidates = (-b1 + np.array([1, -1]) * np.sqrt(discriminant)) / (2 * b2)
    Ti = max(Ti_candidates)

    if Ti > 0:
        Kp = Ti * ωc * np.sqrt((B - ωc**2)**2 + A**2 * ωc**2) / (K * np.sqrt((1 - Ti * Td * ωc**2)**2 + Ti**2 * ωc**2))
    else:
        return None, None

    return Kp, Ti

def main():
    K = float(input("Enter the value of K (e.g., 1.0): "))
    A = float(input("Enter the value of A (e.g., 2.0): "))
    B = float(input("Enter the value of B (e.g., 1.0): "))
    Td = float(input("Enter the value of Td (e.g., 0.5): "))
    wc_start = float(input("Enter the start value for ωc range (e.g., 0.1): "))
    wc_end = float(input("Enter the end value for ωc range (e.g., 5.0): "))
    wc_step = 0.1
    valid_wc = []

    for wc in np.arange(wc_start, wc_end + wc_step, wc_step):
        Kp, Ti = calculate_Kp_Ti(wc, K, A, B, Td)
        if Kp is not None and Ti is not None and validate_pid_parameters(K, A, B, Kp, Ti, Td, wc):
            valid_wc.append(wc)

    if valid_wc:
        print("Valid ωc values:", valid_wc)
        choice = input("Select a specific ωc from the list above (or press enter to select randomly): ")
        selected_wc = float(choice) if choice.strip() else random.choice(valid_wc)

        selected_Kp, selected_Ti = calculate_Kp_Ti(selected_wc, K, A, B, Td)
        print("Selected ωc:", selected_wc)
        print("Selected Kp:", selected_Kp)
        print("Selected Ti:", selected_Ti)

        plant = ctl.tf([K], [1, A, B])
        controller = ctl.tf([selected_Kp * selected_Ti*Td,selected_Kp * selected_Ti, selected_Kp], [selected_Ti, 0])
        system = ctl.feedback(controller * plant)

        print("Plant Transfer Function:", plant)
        print("Controller Transfer Function:", controller)
        print("Closed-loop Transfer Function:", system)

        time, response = ctl.step_response(system)

        plt.figure()
        plt.plot(time, response)
        plt.title('Step Response of the Closed-Loop System')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Output')
        plt.grid(True)
        plt.show()
    else:
        print("No valid ωc found. Try adjusting the range or parameters.")



if __name__ == "__main__":
    main()
