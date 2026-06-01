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


from config import (
    TEMP_CHANNEL,
    USE_KELVIN_READING,
    SETPOINT_K,
    LOOP_DT_S,
    KP,
    KI,
    OUTPUT_MIN,
    OUTPUT_MAX,
    PWM_PIN,
    PWM_FREQ_HZ,
)


def run_control_loop():
    inst = open_ls240()

    # Kd=0 to make it a PI controller
    pid = PID(KP, KI, 0.0, setpoint=SETPOINT_K)

    # set output limits so that PID output = PWM duty cycle
    # between 0.0 (off) and 1.0 (full power)
    pid.output_limits = (OUTPUT_MIN, OUTPUT_MAX)

    pid.sample_time = LOOP_DT_S

    # PWM
    heater_pwm = PWMOutputDevice(
        pin=PWM_PIN,
        active_high=True,
        initial_value=0.0,
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
                channel=TEMP_CHANNEL,
                use_kelvin=USE_KELVIN_READING,
            )

            # get PID output
            u = pid(measured)

            # PID output -> duty cycle
            duty_cycle = u

            # PWM
            heater_pwm.value = duty_cycle  # 0.0–1.0 maps to 0–100% duty

            # debug print statement:
            error = pid.setpoint - measured
            print(
                f"T_meas={measured:.3f} K, "
                f"error={error:.3f}, "
                f"u={u:.3f}, "
                f"duty={duty_cycle:.3f}"
            )

            time.sleep(LOOP_DT_S)

    finally:
        heater_pwm.value = 0.0
        heater_pwm.close()


if __name__ == "__main__":
    run_control_loop()