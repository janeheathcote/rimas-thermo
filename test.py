"""
Test code for LS240.
"""


import time

from config import TEMP_CHANNEL, USE_KELVIN_READING
from ls240_interface import open_ls240, read_value


def main() -> None:
    
    inst = open_ls240()

    print(f"Reading from channel {TEMP_CHANNEL}")
    print(f"Mode: {'Kelvin' if USE_KELVIN_READING else 'Raw sensor'}")
    print()

    for i in range(10):
        value = read_value(inst, channel=TEMP_CHANNEL, use_kelvin=USE_KELVIN_READING)
        print(f"[{i:02d}] value = {value}")
        time.sleep(1.0)



if __name__ == "__main__":
    main()

