"""
Reading from a LakeShore 240 (LS240).

You'll need the official LakeShore Python driver:
    pip install lakeshore

And download LakeShore's MeasureLINK utility
- then pick appropriate calibration curve (e.g. “IEC PT100 RTD” for a PT100 sensor)

References:
- LS240 is documented in the driver docs.
- https://lake-shore-python-driver.readthedocs.io/en/1.5.1/model_240.html
- https://lake-shore-python-driver.readthedocs.io/_/downloads/en/latest/pdf/ (pg 252)

"""


from lakeshore import Model240, Model240InputParameter
from config import CHANNEL, UNITS, USE_KELVIN_READING, LS240_COM_PORT, SENSOR_TYPE


def open_ls240():
    """
    Open a connection to the LS240.
    Returns float.
    """
        
    my_model_240 = Model240(com_port=LS240_COM_PORT)

    config = Model240InputParameter(
        SENSOR_TYPE,
        True,
        False,
        UNITS,
        True,
        0
    )
    
    my_model_240.set_input_parameter(CHANNEL, config)

    return my_model_240


def read_kelvin(inst, channel=CHANNEL):
    """
    Read a Kelvin temperature.
    Returns float.
    """
    return float(inst.get_kelvin_reading(channel))


def read_sensor(inst, channel=CHANNEL):
    """
    Read the raw sensor value.
    Returns float.
    """
    return float(inst.get_sensor_reading(channel))



def read_value(inst, channel=CHANNEL, use_kelvin=USE_KELVIN_READING):
    """
    Read value from LS240:
    - reads Kelvin if use_kelvin=True
    - otherwise reads raw sensor value

    Returns temperature (float).
    """
    if use_kelvin:
        return read_kelvin(inst, channel)
    else:
        return read_sensor(inst, channel)
    
