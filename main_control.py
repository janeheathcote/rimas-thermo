"""
Main temperature-control loop.

Currently:
- Reads temperature (or raw sensor value) from the LS240 temp sensor

"""

import time

from config import (
    TEMP_CHANNEL,
    USE_KELVIN_READING,
    SETPOINT_K,
    LOOP_DT_S,
    KP,
    KI,
    OUTPUT_MIN,
    OUTPUT_MAX,
    MAX_SAFE_TEMP_K,
)

from ls240_interface import open_ls240, read_value
from pi_controller import PIController # <- TEMP! THIS MIGHT BE WRONG
#from heater_output import 


def run_control_loop() -> None:
    """
    Open the Lake Shore 240, repeatedly read temperature,
    compute a PI control output, and send that output to the heater.
    """
    
    inst = open_ls240()
    
    # heater = 

    controller = PIController(
        kp=KP,
        ki=KI,
        setpoint=SETPOINT_K,
        u_min=OUTPUT_MIN,
        u_max=OUTPUT_MAX,
    )
    

    print("Starting control loop...")
    print(f"Setpoint           : {SETPOINT_K}")
    print(f"Loop interval (s)  : {LOOP_DT_S}")
    print(f"Kp, Ki             : {KP}, {KI}")
    print()

    while True:
        measured = read_value(inst, channel=TEMP_CHANNEL, use_kelvin=USE_KELVIN_READING)

        # get value from PI control loop
        u = controller.update(measured, LOOP_DT_S)
        
        # send that to the heater

        # print the values

        time.sleep(LOOP_DT_S)



if __name__ == "__main__":
    run_control_loop()
