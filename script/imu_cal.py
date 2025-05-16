# ================================================================================
# 
# File Name           : imu_cal.py
# Project Name        : LSE2
# Description         : 
#     Script used to calculate the Bias Vector of the Accelerometer as well as the
#     calibration matrix based on the Accelerometer 6-Point Tumble Calibration 
#     method and a least squares algorithm.
# 
# Author              : Adrià Babiano Novella
# Last Modified       : May 2025
#
# ================================================================================

import numpy as np
from termcolor import colored, cprint
import sympy

# Initialize sympy for pretty printing (ensures nice mathematical output)
sympy.init_printing(use_unicode=True)

def get_user_input_for_orientation(orientation_name):
    """
    Prompts the user to enter the averaged raw LSB values for X, Y, and Z axes
    for a given orientation. Uses termcolor for colored prompts.
    """
    cprint(f"\nEnter averaged RAW LSB values for orientation: {orientation_name}", "yellow", attrs=["bold"])
    while True:
        try:
            ax_str = input(colored("  Average Raw X LSB: ", "cyan"))
            ay_str = input(colored("  Average Raw Y LSB: ", "cyan"))
            az_str = input(colored("  Average Raw Z LSB: ", "cyan"))
            raw_vector = np.array([float(ax_str), float(ay_str), float(az_str)])
            return raw_vector
        except ValueError:
            cprint("Invalid input. Please enter numeric values for all axes.", "red", attrs=["bold"])
        except Exception as e:
            cprint(f"An unexpected error occurred during input: {e}. Please try again.", "red", attrs=["bold"])

def calculate_calibration_parameters(measurements):
    """
    Calculates the accelerometer calibration parameters (bias and transformation matrix)
    using the least-squares method.
    """
    num_orientations = len(measurements)
    if num_orientations < 4:
        raise ValueError(
            "At least 4 independent measurement orientations are needed for 12-parameter calibration. "
            f"Provided: {num_orientations}"
        )

    H_matrix_stacked = np.zeros((num_orientations * 3, 12))
    Y_vector_stacked = np.zeros((num_orientations * 3, 1))

    for i in range(num_orientations):
        A_raw_avg, A_true_physical = measurements[i]
        ax_raw, ay_raw, az_raw = A_raw_avg
        ax_true, ay_true, az_true = A_true_physical

        Y_vector_stacked[i*3 : i*3+3] = np.array([[ax_raw], [ay_raw], [az_raw]])

        H_row1 = [ax_true, ay_true, az_true, 0,       0,       0,       0,       0,       0,       1, 0, 0]
        H_row2 = [0,       0,       0,       ax_true, ay_true, az_true, 0,       0,       0,       0, 1, 0]
        H_row3 = [0,       0,       0,       0,       0,       0,       ax_true, ay_true, az_true, 0, 0, 1]
        H_matrix_stacked[i*3 : i*3+3, :] = np.array([H_row1, H_row2, H_row3])

    X_params_vector, residuals, rank_from_lstsq, singular_values = np.linalg.lstsq(
        H_matrix_stacked, Y_vector_stacked, rcond=None
    )
    # Explicit conversion of rank to a standard Python int is good practice,
    # even without type hints, for consistency if used later.
    rank = int(rank_from_lstsq)
    X_params_flat = X_params_vector.flatten()

    K_matrix = np.array([
        [X_params_flat[0], X_params_flat[1], X_params_flat[2]],
        [X_params_flat[3], X_params_flat[4], X_params_flat[5]],
        [X_params_flat[6], X_params_flat[7], X_params_flat[8]]
    ])

    B_LSB_vector = np.array([X_params_flat[9], X_params_flat[10], X_params_flat[11]])

    try:
        M_cal_matrix = np.linalg.inv(K_matrix)
    except np.linalg.LinAlgError:
        cprint("\nError: K_matrix is singular and cannot be inverted.", "red", attrs=["bold"])
        cprint("This can happen if the input data does not provide enough independent equations "
               "(e.g., orientations are not distinct or don't sufficiently span 3D space).", "red")
        cprint("Ensure your 6 orientations are distinct and cover all axes.", "red")
        M_cal_matrix = np.identity(3)
        B_LSB_vector = np.zeros(3)

    debug_info = {
        "K_matrix": K_matrix,
        "X_params_vector": X_params_vector,
        "residuals": residuals,
        "rank": rank,
        "singular_values": singular_values
    }
    return M_cal_matrix, B_LSB_vector, debug_info

def main():
    """
    Main function to run the accelerometer calibration script.
    Collects user input, calculates parameters, and prints results.
    """
    g = 9.80665

    cprint("Accelerometer 6-Point Tumble Calibration Data Input", "green", attrs=["bold", "underline"])
    print("----------------------------------------------------")
    cprint(f"Please provide averaged RAW LSB accelerometer readings for each of the 6 orientations.", "blue")
    cprint(f"The true acceleration reference magnitude for each primary axis will be g = {g:.5f} m/s^2.", "blue")

    orientations = [
        ("+X up (against gravity)", np.array([g, 0, 0])),
        ("-X up (X along gravity)", np.array([-g, 0, 0])),
        ("+Y up (against gravity)", np.array([0, g, 0])),
        ("-Y up (Y along gravity)", np.array([0, -g, 0])),
        ("+Z up (against gravity)", np.array([0, 0, g])),
        ("-Z up (Z along gravity)", np.array([0, 0, -g]))
    ]

    collected_measurements = []

    for orientation_name, true_accel_vector in orientations:
        raw_lsb_data = get_user_input_for_orientation(orientation_name)
        collected_measurements.append((raw_lsb_data, true_accel_vector))

    cprint("\nCalculating calibration parameters...", "magenta", attrs=["bold"])
    try:
        M_cal, B_LSB, dbg_info = calculate_calibration_parameters(collected_measurements)

        cprint("\n--- Calibration Results ---", "green", attrs=["bold", "underline"])

        Bx_sym, By_sym, Bz_sym = sympy.symbols('B_x, B_y, B_z')
        S_x_sym, S_y_sym, S_z_sym = sympy.symbols('S_x, S_y, S_z')
        M_xy_sym, M_xz_sym, M_yx_sym, M_yz_sym, M_zx_sym, M_zy_sym = sympy.symbols(
            'M_xy, M_xz, M_yx, M_yz, M_zx, M_zy'
        )

        bias_symbolic_matrix = sympy.Matrix([Bx_sym, By_sym, Bz_sym])
        m_cal_symbolic_matrix = sympy.Matrix([
            [S_x_sym, M_xy_sym, M_xz_sym],
            [M_yx_sym, S_y_sym, M_yz_sym],
            [M_zx_sym, M_zy_sym, S_z_sym]
        ])

        print(colored("Bias Vector (B_LSB, to subtract from Raw LSB readings):", "yellow", attrs=["bold"]))
        bias_numeric_sympy_matrix = sympy.Matrix(B_LSB).applyfunc(lambda x: sympy.Float(f"{x:.4f}"))
        sympy.pprint(sympy.Eq(bias_symbolic_matrix, bias_numeric_sympy_matrix, evaluate=False), use_unicode=True)
        print(colored(f"  (Numeric: Bx: {B_LSB[0]:.4f} LSB, By: {B_LSB[1]:.4f} LSB, Bz: {B_LSB[2]:.4f} LSB)", 'light_grey'))

        print(colored("\nCalibration Matrix (M_cal, scales and corrects axes for (Raw_LSB - B_LSB)):", "yellow", attrs=["bold"]))
        m_cal_numeric_sympy_matrix = sympy.Matrix(M_cal).applyfunc(lambda x: sympy.Float(f"{x:.6e}"))
        sympy.pprint(sympy.Eq(m_cal_symbolic_matrix, m_cal_numeric_sympy_matrix, evaluate=False), use_unicode=True)
        print("  (Numeric M_cal for copy-pasting):")
        print(colored(np.array2string(M_cal, separator=', ', formatter={'float_kind':lambda x: "%.8e" % x}), 'light_grey'))

        print(colored("\nTo get calibrated physical acceleration (e.g., in m/s^2):", "blue"))
        cprint("  Accel_calibrated_physical = M_cal @ (Accel_raw_LSB - B_LSB)", "white")

        cprint("\n--- Verification (applying calculated calibration to your input data) ---", "green", attrs=["bold", "underline"])
        for i, (raw_data, true_data) in enumerate(collected_measurements):
            calibrated_data = M_cal @ (raw_data - B_LSB)
            error_vector = calibrated_data - true_data
            error_magnitude = np.linalg.norm(error_vector)

            print(colored(f"\nOrientation: {orientations[i][0]}", "yellow"))
            print(f"  Raw LSB Input:    {colored(np.array2string(raw_data, formatter={'float_kind':lambda x: f'{x:.2f}'}), 'light_blue')}")
            print(f"  True Physical:    {colored(np.array2string(true_data, formatter={'float_kind':lambda x: f'{x:.4f}'}), 'green')}")
            print(f"  Calibrated Phys:  {colored(np.array2string(calibrated_data, formatter={'float_kind':lambda x: f'{x:.4f}'}), 'cyan')}")
            print(f"  Error Vector:     {colored(np.array2string(error_vector, formatter={'float_kind':lambda x: f'{x:.4f}'}), 'red')}")
            print(f"  Error Magnitude:  {colored(f'{error_magnitude:.4f} m/s^2', 'red', attrs=['bold'] if error_magnitude > 0.1 else None)}")

    except ValueError as e:
        cprint(f"\nValueError during calculation: {e}", "red", attrs=["bold"])
    except Exception as e:
        cprint(f"\nAn unexpected error occurred: {e}", "red", attrs=["bold"])

main()
