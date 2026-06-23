"""
Main temperature-control loop using simple-pid.

- Reads temperature from the LS240
- Runs a PI loop
- Drives a heater via Raspberry Pi PWM
"""

import time
from gpiozero import PWMOutputDevice
from simple_pid import PID
from ls240_interface import open_ls240, read_value
import matplotlib.pyplot as plt
import datetime as dt
import os
from ziegler_nichols_autotuner import ZieglerNicholsAutotuner


from config import (
    CHANNEL,
    USE_KELVIN_READING,
    SETPOINT_K,
    LOOP_DT_S,
    KP_STEP,
    KP_CHANGE,
    WINDOW_SIZE,
    OSC_THRESH,
    MIN_OSC,
    KP,
    KI,
    KD,
    OUTPUT_MIN,
    OUTPUT_MAX,
    PWM_PIN,
    PWM_FREQ_HZ,
    DUR,
)


def run_control_loop():
    
    # open connection to LakeShore
    inst = open_ls240()

    # Kd=0 to make it a PI controller
    pid = PID(KP, KI, KD, setpoint=SETPOINT_K)

    # set output limits so that PID output = PWM duty cycle
    # between 0.0 (off) and 1.0 (full power)
    pid.output_limits = (OUTPUT_MIN, OUTPUT_MAX)

    pid.sample_time = LOOP_DT_S

    # PWM
    pwm = PWMOutputDevice(
        pin=PWM_PIN,
        frequency=PWM_FREQ_HZ,
    )

    print("Starting control loop...")
    print(f"Setpoint           : {SETPOINT_K}")
    print(f"Loop interval (s)  : {LOOP_DT_S}")
    print(f"Kp, Ki             : {KP}, {KI}")
    print(f"PWM pin, freq (Hz) : {PWM_PIN}, {PWM_FREQ_HZ}")
    print()

    try:
        while True:
            # read temp
            measured = read_value(
                inst,
                channel=CHANNEL,
                use_kelvin=USE_KELVIN_READING,
            )

            # get PID output
            u = pid(measured)

            # PID output (0.0-1.0) -> duty cycle (0–100%)
            duty_cycle = u

            # PWM
            pwm.value = duty_cycle

            # debug print statement:
            error = pid.setpoint - measured
            print(
                f"T_meas={measured:.3f} K, "
                f"error={error:.3f}, "
                f"u={u:.3f}, "
                f"duty={duty_cycle:.3f}"
            )

            time.sleep(LOOP_DT_S)
            
    except KeyboardInterrupt:
        print("Stopping control loop.")

    finally:
        pwm.value = 0.0
        pwm.off()
        pwm.close()
        
        

def tune_pid():

    
    inst = open_ls240()
    
    pwm = PWMOutputDevice(
        pin=PWM_PIN,
        frequency=PWM_FREQ_HZ,
    )

    def read_temperature():
        return read_value(inst, channel=CHANNEL, use_kelvin=USE_KELVIN_READING)

    def set_heater_power(power):
        pwm.value = max(0.0, min(1.0, power))
    
    # run the PID autotuner
    autotuner = ZieglerNicholsAutotuner(
        sensor_func=read_temperature,
        actuator_func=set_heater_power,
        setpoint=SETPOINT_K,
        min_output=OUTPUT_MIN,
        max_output=OUTPUT_MAX,
        kp_step=KP_STEP,
        max_test_time=DUR,
        oscillation_threshold=OSC_THRESH,
        window_size=WINDOW_SIZE,
        min_oscillations=MIN_OSC,
        kp_change_interval_pct=KP_CHANGE,
        start_kp=2.0
    )
    
    results = autotuner.run(verbose=True, plot_results=True)
    
    # the recommended PID parameters:
    if results:
        pid_params = results['PID']  # or 'No Overshoot', 'Some Overshoot', etc.
        print(f"Kp = {pid_params['Kp']}")
        print(f"Ki = {pid_params['Ki']}")
        print(f"Kd = {pid_params['Kd']}")


# start with 10 minutes
def run_step_test(duration_s=DUR):
    
    inst = open_ls240()

    pid = PID(KP, KI, KD, setpoint=SETPOINT_K)
    pid.output_limits = (OUTPUT_MIN, OUTPUT_MAX)  # (0.0, 1.0)
    pid.sample_time = LOOP_DT_S

    # PWM
    pwm = PWMOutputDevice(
        pin=PWM_PIN,
        frequency=PWM_FREQ_HZ,
    )

    t0 = time.time()

    times = []
    temps = []
    duties = []

    print(f"Starting step test for {duration_s} s with Kp={KP}, Ki={KI}, Kd={KD}")
    print(f"Setpoint: {SETPOINT_K} K")

    try:
        while True:
            now = time.time()
            elapsed = now - t0
            if elapsed >= duration_s:
                break

            measured = read_value(inst, channel=CHANNEL, use_kelvin=USE_KELVIN_READING)

            u = pid(measured)  # in [OUTPUT_MIN, OUTPUT_MAX]

            duty_cycle = u
            pwm.value = duty_cycle

            times.append(elapsed)
            temps.append(measured)
            duties.append(duty_cycle)

            print(
                f"t={elapsed:6.1f}s  T={measured:.3f}K  "
                f"error={pid.setpoint - measured:.3f}  "
                f"u/duty={duty_cycle:.3f}"
            )

            time.sleep(LOOP_DT_S)

    finally:
        pwm.value = 0.0
        pwm.off()
        pwm.close()

    # PLOT:
    fig, ax1 = plt.subplots()

    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Temperature (K)")
    ax1.plot(times, temps, color="tab:red", label="Temperature")
    ax1.axhline(SETPOINT_K, color="tab:red", linestyle="--", alpha=0.5, label="Setpoint")
    ax1.tick_params(axis="y", labelcolor="tab:red")

    plt.title(f"Step test: Kp={KP}, Ki={KI}, Kd={KD}")
    fig.tight_layout()
    plt.show(block=True)

    # timestamped filename to avoid overwrites
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"output"
    filename = f"temp_step_Kp{KP:.2f}_{timestamp}.png"
    filepath = os.path.join(output_dir, filename)
    fig.tight_layout()
    fig.savefig(filepath, dpi=150)

    print(f"Plot saved to {os.path.abspath(filename)}")


if __name__ == "__main__":
    #run_control_loop()
    run_step_test()
    #tune_pid()
