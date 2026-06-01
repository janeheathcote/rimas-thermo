from lakeshore.model_240 import Model240Enums

CHANNEL = 2   # LS240 has channels 1-8
SENSOR_TYPE = Model240Enums.SensorTypes.PLATINUM_RTD   # PT100
UNITS = Model240Enums.Units.KELVIN 
USE_KELVIN_READING = True
LS240_COM_PORT = "/dev/ttyUSB0"
#LS240_COM_PORT = "COM4"


SETPOINT_K = 295 #TEMP
LOOP_DT_S = 5 #TEMP
KP = 1 #TEMP
KI = 0 #TEMP
OUTPUT_MIN = 0.0
OUTPUT_MAX = 1.0
PWM_PIN = 13        # GPIO 13 (PWM1), physical pin 33
PWM_FREQ_HZ = 1000
