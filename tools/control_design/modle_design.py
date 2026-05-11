import numpy as np
import control as ctrl
import matplotlib.pyplot as plt
from typing import  Tuple


# =========================================================
# MOTOR MODELS
# =========================================================
def motor_velocity_tf(J:float, b:float, K:float, R:float, L:float) -> ctrl.TransferFunction:
    """Voltage -> Velocity (continuous)

    Args:
        J (float): _description_
        b (float): _description_
        K (float): _description_
        R (float): _description_
        L (float): _description_

    Returns:
        ctrl.TransferFunction: _description_
    """
    num = [K]
    den = [J * L, J * R + b * L, b * R + K**2]

    return ctrl.TransferFunction(num, den)


def motor_position_tf(J:float, b:float, K:float, R:float, L:float) -> ctrl.TransferFunction:
    """Voltage -> Position (continuous)

    Args:
        J (float): _description_
        b (float): _description_
        K (float): _description_
        R (float): _description_
        L (float): _description_

    Returns:
        ctrl.TransferFunction: _description_
    """
    s = ctrl.TransferFunction.s

    Gv_s = motor_velocity_tf(J, b, K, R, L)
    Gp_s = Gv_s * (1 / s)

    return Gp_s
# =========================================================
# PID
# =========================================================
def pid_tf(Kp:float, Ki:float, Kd:float, tau:float) -> ctrl.TransferFunction:
    """Continuous PID with derivative filter

    Args:
        Kp (float): _description_
        Ki (float): _description_
        Kd (float): _description_
        tau (float): _description_

    Returns:
        ctrl.TransferFunction: _description_
    """

    s = ctrl.TransferFunction.s
    return Kp + Ki/s + (Kd*s)/(tau*s + 1)


# =========================================================
# DISCRETIZATION
# =========================================================
def discretize_system(Gv_s:ctrl.TransferFunction, 
                      Gp_s:ctrl.TransferFunction, 
                      Cs:ctrl.TransferFunction, 
                      Ts: float) -> Tuple[ctrl.TransferFunction]:
    """Discretize each block independently (matches C)

    Args:
        Gv_s (ctrl.TransferFunction): _description_
        Gp_s (ctrl.TransferFunction): _description_
        Cs (ctrl.TransferFunction): _description_
        Ts (float): _description_

    Returns:
        tuple[ctrl.TransferFunction]: _description_
    """
    

    Gv_z = ctrl.c2d(Gv_s, Ts, method='zoh')       # plant (correct)
    Gp_z = ctrl.c2d(Gp_s, Ts, method='zoh')       # optional
    Cz   = ctrl.c2d(Cs,   Ts, method='tustin')    # controller

    return Gv_z, Gp_z, Cz


# =========================================================
# CLOSED LOOP
# =========================================================
def closed_loop(Cz:ctrl.TransferFunction, Gv_z:ctrl.TransferFunction)->ctrl.TransferFunction:
    """Position loop: PID(angle error) -> voltage -> velocity -> integrated externally

    Args:
        Cz (ctrl.TransferFunction): _description_
        Gv_z (ctrl.TransferFunction): _description_

    Returns:
        ctrl.TransferFunction: _description_
    """

    return ctrl.feedback(Cz * Gv_z, 1)

# =========================================================
# CLOSED LOOP SIGNALS
# =========================================================
def closed_loop_signals(Cz: ctrl.TransferFunction,
                        Gp_z: ctrl.TransferFunction) -> tuple[
                            ctrl.TransferFunction,
                            ctrl.TransferFunction,
                            ctrl.TransferFunction
                        ]:
    """
    Builds transfer functions for the internal closed-loop signals.

    r = setpoint/reference
    e = error into controller
    u = controller output / voltage into plant
    y = plant output / position
    """

    L = ctrl.minreal(Cz * Gp_z, verbose=False)

    one = ctrl.TransferFunction([1.0], [1.0], Cz.dt)

    E_over_R = ctrl.minreal(ctrl.feedback(one, L), verbose=False)
    U_over_R = ctrl.minreal(Cz * E_over_R, verbose=False)
    Y_over_R = ctrl.minreal(Gp_z * U_over_R, verbose=False)

    return E_over_R, U_over_R, Y_over_R


# =========================================================
# SIMULATION
# =========================================================
def simulate(Cz: ctrl.TransferFunction,
             Gp_z: ctrl.TransferFunction,
             Ts: float,
             sim_time: float = 10.0,
             setpoint: float = 90.0,
             title: str = "Closed-loop Simulation") -> None:
    """
    Simulates and plots:

    setpoint
    controller output / voltage command
    plant output / position
    error
    """

    t = np.arange(0, sim_time, Ts)
    r = np.ones_like(t) * setpoint

    E_over_R, U_over_R, Y_over_R = closed_loop_signals(Cz, Gp_z)

    # Simulate internal signals
    _, e = ctrl.forced_response(E_over_R, T=t, U=r)
    _, u = ctrl.forced_response(U_over_R, T=t, U=r)
    _, y = ctrl.forced_response(Y_over_R, T=t, U=r)

    poles = ctrl.poles(Y_over_R)

    print("\nClosed-loop Poles:", poles)
    if np.any(np.abs(poles) > 1.0):
        print("[WARNING] Unstable: pole outside unit circle")

    try:
        info = ctrl.step_info(Y_over_R * setpoint, T=t)
        print("\nPlant Output Step Info:")
        for k, v in info.items():
            print(f"{k}: {v}")
    except Exception as err:
        print(f"\nStep info failed: {err}")

    # One window, multiple views
    fig, axs = plt.subplots(4, 1, sharex=True, figsize=(10, 8))

    axs[0].plot(t, r, label="setpoint r")
    axs[0].plot(t, y, label="plant output y")
    axs[0].set_ylabel("Position")
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(t, u, label="controller output u")
    axs[1].set_ylabel("Voltage")
    axs[1].grid(True)
    axs[1].legend()

    axs[2].plot(t, e, label="error e")
    axs[2].set_ylabel("Error")
    axs[2].grid(True)
    axs[2].legend()

    axs[3].plot(t, r, label="setpoint r")
    axs[3].plot(t, y, label="plant output y")
    axs[3].plot(t, u, label="controller output u")
    axs[3].set_xlabel("Time [s]")
    axs[3].set_ylabel("All Signals")
    axs[3].grid(True)
    axs[3].legend()

    fig.suptitle(title)
    plt.tight_layout()
    plt.show()


# =========================================================
# EXPORT (C)
# =========================================================
def export_tf(sys:ctrl.TransferFunction, name:str ="TF") -> None:
    """_summary_

    Args:
        sys (ctrl.TransferFunction): _description_
        name (str, optional): _description_. Defaults to "TF".

    Returns:
        _type_: _description_
    """
    num = np.squeeze(sys.num[0][0])
    den = np.squeeze(sys.den[0][0])

    num = np.atleast_1d(num) / den[0]
    den = np.atleast_1d(den) / den[0]

    def fmt(arr):
        return "{" + ", ".join([f"{x:.8f}f" for x in arr]) + "};"

    print(f"\n// ===== {name} =====")
    print("// NUM:")
    print(fmt(num))
    print("// DEN:")
    print(fmt(den))


# =========================================================
# MAIN
# =========================================================
if __name__ == "__main__":

    # -------------------------
    # Motor params
    # -------------------------
    J:float  = 3.2284e-6 
    b:float  = 3.5077e-6
    K:float  = 0.0274
    R:float  = 4.0
    L:float  = 2.75E-6

    Ts:float  = 0.001

    # -------------------------
    # PID params
    # -------------------------
    Kp:float  = 0.205
    Ki:float  = 0.02
    Kd:float  = 0.00

    D_cutoff:float  = 20
    tau = 1 / (2 * np.pi * D_cutoff)

    # -------------------------
    # Build continuous systems
    # -------------------------
    Gv_s:ctrl.TransferFunction = motor_velocity_tf(J, b, K, R, L)
    Gp_s:ctrl.TransferFunction = motor_position_tf(J, b, K, R, L)
    Cs:ctrl.TransferFunction   = pid_tf(Kp, Ki, Kd, tau)

    # -------------------------
    # Discretize
    # -------------------------
    Gv_z, Gp_z, Cz = discretize_system(Gv_s, Gp_s, Cs, Ts)

    # -------------------------
    # Closed loop (position plant)
    # -------------------------
    Tz:ctrl.TransferFunction = closed_loop(Cz, Gp_z)

    # -------------------------
    # Simulate
    # -------------------------
    simulate(
        Cz=Cz,
        Gp_z=Gp_z,
        Ts=Ts,
        sim_time=1.00,
        setpoint=1.0,
        title="Closed-loop Position Response"
    )

    # -------------------------
    # EXPORTS (clean + separate)
    # -------------------------
    #export_tf(Gv_z, "Motor Velocity (z)   // voltage -> velocity")
    export_tf(Gp_z, "Motor Position (z)  // voltage -> position")
    export_tf(Cz,   "PID (z)             // error -> voltage ")