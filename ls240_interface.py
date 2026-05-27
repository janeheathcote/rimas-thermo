"""
Reading from a Lake Shore 240.

You'll need the official Lake Shore Python driver:
    pip install lakeshore

References:
- LS240 is documented in the driver docs.
- https://lake-shore-python-driver.readthedocs.io/en/1.5.1/model_240.html

"""



from config import CHANNEL, SENSOR_TYPE, USE_KELVIN_READING, LS240_COM_PORT

def open_ls240():
    """
    Open a connection to the Lake Shore 240.
    Returns float.
    """
    
    from lakeshore import Model240
    
    com_port = LS240_COM_PORT
    my_model_240 = Model240(com_port=com_port)
    
    # set the sensor type
    my_model_240.set_input_sensor(CHANNEL, SENSOR_TYPE)
    
    print("IDN: {}".format(my_model_240.query('*IDN?')))
    print("Profibus connection status: {}".format(my_model_240.get_profibus_connection_status()))

    return my_model_240


def read_kelvin(inst, channel: int = CHANNEL) -> float:
    """
    Read a Kelvin temperature.
    Returns float.
    """
    return float(inst.get_kelvin_reading(channel))


def read_sensor(inst, channel: int = CHANNEL) -> float:
    """
    Read the raw sensor value.
    Returns float.
    """
    return float(inst.get_sensor_reading(channel))



def read_value(inst, sensor: string = SENSOR_TYPE, channel: int = CHANNEL, use_kelvin: bool = USE_KELVIN_READING) -> float:
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
    
    
    
