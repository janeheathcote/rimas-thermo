"""
Reading from a Lake Shore 240 with a PT100.

You'll need the official Lake Shore Python driver:
    pip install lakeshore

And download Lake Shore's MeasureLINK utility
- then pick appropriate calibration curve (“IEC PT100 RTD”)

References:
- LS240 is documented in the driver docs.
- https://lake-shore-python-driver.readthedocs.io/en/1.5.1/model_240.html
- https://lake-shore-python-driver.readthedocs.io/_/downloads/en/latest/pdf/ (pg 252)

"""



from config import CHANNEL, USE_KELVIN_READING, LS240_COM_PORT
from serial.tools.list_ports import comports




def open_ls240():
    """
    Open a connection to the Lake Shore 240.
    Returns float.
    """
    
    from lakeshore import Model240, Model240InputParameter
    
    com_port = LS240_COM_PORT
    my_model_240 = Model240(com_port=com_port)
    
    # set PROFIBUS address to 123 (arbitrary, 1-125)
    my_model_240.set_profibus_address("123")
    
    # set sensor type = PT100
    config = Model240InputParameter(my_model_240.SensorTypes.PLATINUM_RTD, True, False, my_model_240.Units.KELVIN, True, 0)
    my_model_240.set_input_parameter(CHANNEL, config)

    
    print("IDN: {}".format(my_model_240.query('*IDN?')))
    #print("Profibus connection status: {}".format(my_model_240.get_profibus_connection_status()))
    #header_info = my_model_240.get_curve_header(1)
    #print("Curve Name: ", header_info.curve_name)

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

    Returns float.
    """
    if use_kelvin:
        return read_kelvin(inst, channel)
    else:
        return read_sensor(inst, channel)
    
