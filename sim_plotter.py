"""Plot WBC/MPC telemetry.json (expanded channels)."""

import json
import numpy as np
import matplotlib.pyplot as plt


def _arr(tel, key, sort_idx, default=None):
    if key not in tel or not tel[key]:
        return default
    a = np.array(tel[key])
    if sort_idx is not None and len(a) == len(sort_idx):
        a = a[sort_idx]
    return a


def plot_telemetry(filename):
    with open(filename, "r") as f:
        telemetry = json.load(f)

    t = np.array(telemetry.get("t", []))
    if len(t) == 0:
        print("Empty telemetry")
        return

    sort_idx = np.argsort(t)
    t = t[sort_idx]

    roll = _arr(telemetry, "roll", sort_idx)
    pitch = _arr(telemetry, "pitch", sort_idx)
    z = _arr(telemetry, "z", sort_idx)
    vx = _arr(telemetry, "vx", sort_idx)
    vx_cmd = _arr(telemetry, "vx_cmd", sort_idx)
    vx_truth = _arr(telemetry, "vx_truth", sort_idx)
    fc_des = _arr(telemetry, "fc_des", sort_idx)
    tau_opt = _arr(telemetry, "tau_opt", sort_idx)
    c_state = _arr(telemetry, "contact_state", sort_idx)
    force_scale = _arr(telemetry, "force_scale", sort_idx)
    foot_z = _arr(telemetry, "foot_z", sort_idx)
    foot_pos = _arr(telemetry, "foot_pos_actual", sort_idx)
    dtau = _arr(telemetry, "dtau_max", sort_idx)
    amp_f = _arr(telemetry, "amp_front", sort_idx)
    amp_r = _arr(telemetry, "amp_rear", sort_idx)
    base_acc_des = _arr(telemetry, "base_acc_des", sort_idx)

    fig, axs = plt.subplots(5, 2, figsize=(15, 20))

    axs[0, 0].plot(t, roll, label="Roll")
    axs[0, 0].plot(t, pitch, label="Pitch")
    axs[0, 0].set_title("Base Orientation (rad)")
    axs[0, 0].legend()
    axs[0, 0].grid()

    axs[0, 1].plot(t, z, label="Z Height", color="purple")
    axs[0, 1].set_title("Base Z Position (m)")
    axs[0, 1].legend()
    axs[0, 1].grid()

    if fc_des is not None and fc_des.ndim == 2:
        axs[1, 0].plot(t, fc_des[:, 2], label="FL Fz")
        axs[1, 0].plot(t, fc_des[:, 5], label="FR Fz")
        axs[1, 0].plot(t, fc_des[:, 8], label="RL Fz")
        axs[1, 0].plot(t, fc_des[:, 11], label="RR Fz")
    axs[1, 0].set_title("MPC Desired Z Forces (N)")
    axs[1, 0].legend()
    axs[1, 0].grid()

    axs[1, 1].plot(t, vx, label="Vx est")
    if vx_cmd is not None:
        axs[1, 1].plot(t, vx_cmd, label="Vx cmd", linestyle="--")
    if vx_truth is not None:
        axs[1, 1].plot(t, vx_truth, label="Vx truth", linestyle=":")
    axs[1, 1].set_title("Base Vx: cmd / est / truth")
    axs[1, 1].legend()
    axs[1, 1].grid()

    if force_scale is not None and force_scale.ndim == 2:
        for i, leg in enumerate(("FL", "FR", "RL", "RR")):
            axs[2, 0].plot(t, force_scale[:, i], label=leg)
    elif c_state is not None and c_state.ndim == 2:
        axs[2, 0].plot(t, c_state[:, 0], label="FL")
        axs[2, 0].plot(t, c_state[:, 2] + 1.1, label="RL")
    axs[2, 0].set_title("force_scale (soft contact)")
    axs[2, 0].legend()
    axs[2, 0].grid()

    if foot_z is not None and foot_z.ndim == 2:
        for i, leg in enumerate(("FL", "FR", "RL", "RR")):
            axs[2, 1].plot(t, foot_z[:, i], label=leg)
        axs[2, 1].set_title("Foot Z world (m)")
    elif foot_pos is not None and foot_pos.ndim == 2:
        axs[2, 1].plot(t, foot_pos[:, 0], label="FL X")
        axs[2, 1].plot(t, foot_pos[:, 3], label="FR X")
        axs[2, 1].plot(t, foot_pos[:, 6], label="RL X")
        axs[2, 1].plot(t, foot_pos[:, 9], label="RR X")
        axs[2, 1].set_title("Foot Actual X")
    axs[2, 1].legend()
    axs[2, 1].grid()

    if dtau is not None:
        axs[3, 0].plot(t, dtau, label="|Δτ|_max", color="crimson")
    elif tau_opt is not None and tau_opt.ndim == 2:
        for i in range(min(4, tau_opt.shape[1])):
            axs[3, 0].plot(t, tau_opt[:, i], label=f"Tau {i}")
    axs[3, 0].set_title("|Δτ|_max per tick (Nm)")
    axs[3, 0].legend()
    axs[3, 0].grid()

    if amp_f is not None:
        axs[3, 1].plot(t, np.abs(amp_f) * 100, label="amp_front cm")
    if amp_r is not None:
        axs[3, 1].plot(t, np.abs(amp_r) * 100, label="amp_rear cm")
    axs[3, 1].set_title("Gait amp (cm)")
    axs[3, 1].legend()
    axs[3, 1].grid()

    if base_acc_des is not None and base_acc_des.ndim == 2:
        axs[4, 0].plot(t, base_acc_des[:, 0], label="Ax Des")
        axs[4, 0].plot(t, base_acc_des[:, 1], label="Ay Des")
        axs[4, 0].plot(t, base_acc_des[:, 2], label="Az Des")
    axs[4, 0].set_title("Base Accel Desired")
    axs[4, 0].legend()
    axs[4, 0].grid()

    if c_state is not None and c_state.ndim == 2:
        axs[4, 1].plot(t, c_state[:, 0], label="FL")
        axs[4, 1].plot(t, c_state[:, 1] + 1.1, label="FR")
        axs[4, 1].plot(t, c_state[:, 2] + 2.2, label="RL")
        axs[4, 1].plot(t, c_state[:, 3] + 3.3, label="RR")
    axs[4, 1].set_title("Contact State (offset)")
    axs[4, 1].legend()
    axs[4, 1].grid()

    plt.tight_layout()
    plt.savefig("wbc_mpc_telemetry.png", dpi=150)
    print("Saved plot to wbc_mpc_telemetry.png")


if __name__ == "__main__":
    plot_telemetry("telemetry.json")
