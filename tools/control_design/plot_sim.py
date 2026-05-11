from pathlib import Path
import argparse
import csv
import matplotlib.pyplot as plt


def read_pid_csv(csv_path: Path) -> dict[str, list[float]]:
    """
    Read PID simulation CSV output from the C program.
    """

    if not csv_path.exists():
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    data = {
        "time": [],
        "setpoint": [],
        "measured_position": [],
        "error": [],
        "raw_voltage": [],
        "voltage_command": [],
    }

    with csv_path.open("r", newline="") as file:
        reader = csv.DictReader(file)

        for row in reader:
            data["time"].append(float(row["time"]))
            data["setpoint"].append(float(row["setpoint"]))
            data["measured_position"].append(float(row["measured_position"]))
            data["error"].append(float(row["error"]))
            data["raw_voltage"].append(float(row["raw_voltage"]))
            data["voltage_command"].append(float(row["voltage_command"]))

    return data


def plot_pid_data(data: dict[str, list[float]], csv_path: Path) -> None:
    """
    Plot position, error, and voltage data in one window.
    """

    time = data["time"]

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

    fig.suptitle(f"PID Simulation Output\n{csv_path}")

    # Position response
    axes[0].plot(time, data["setpoint"], label="Setpoint")
    axes[0].plot(time, data["measured_position"], label="Measured Position")
    axes[0].set_ylabel("Position")
    axes[0].grid(True)
    axes[0].legend()

    # Position error
    axes[1].plot(time, data["error"], label="Error")
    axes[1].set_ylabel("Error")
    axes[1].grid(True)
    axes[1].legend()

    # Voltage command
    #axes[2].plot(time, data["raw_voltage"], label="Raw Voltage")
    axes[2].plot(time, data["voltage_command"], label="Clamped Command")
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("Voltage [V]")
    axes[2].grid(True)
    axes[2].legend()

    plt.tight_layout()
    plt.show()


def main() -> None:
    project_root = Path(__file__).resolve().parents[2]

    parser = argparse.ArgumentParser(
        description="Plot floating-point and fixed-point PID simulation CSV output."
    )

    parser.add_argument(
        "--float-csv",
        default=project_root / "output" / "pid_floating_output.csv",
        type=Path,
        help="Path to floating-point PID CSV file. Default: output/pid_floating_output.csv",
    )

    parser.add_argument(
        "--fixed-csv",
        default=project_root / "output" / "pid_fixed_output.csv",
        type=Path,
        help="Path to fixed-point PID CSV file. Default: output/pid_fixed_output.csv",
    )

    args = parser.parse_args()

    print("\nPlotting floating-point PID demo...")
    float_data = read_pid_csv(args.float_csv)
    plot_pid_data(float_data, args.float_csv)

    print("\nPlotting fixed-point PID demo...")
    fixed_data = read_pid_csv(args.fixed_csv)
    plot_pid_data(fixed_data, args.fixed_csv)


if __name__ == "__main__":
    main()
