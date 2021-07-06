import serial
import struct


def pin_mode(pin, mode, ser):
    """
    Sets a GPIO pin to be able to read or be written to.

    Example:
        # pin 12 will be set as an OUTPUT pin
        # the instruction is sent through a Serial port to the Arduino
        ser = serial.Serial('COM15', 115200)
        def start():
                pin_mode(12, 'OUTPUT', ser)

    Args:
        pin: (int) The GPIO pin on the Arduino you want to set.
        mode: (string) The pin mode, 'INPUT' or 'OUTPUT'.
        ser: (Serial) The open, USB connection to the Arduino.

    Note:
        The pin value ranges between 0 and 19, this includes the digital
        pins and the analog pins.
    """
    if pin >= 0 and pin < 20:
        # set the pin to be readable
        if mode == "INPUT":
            write = bytes([pin + 1, 114, 0])
            ser.write(write)
        # set the pin to be writeable
        elif mode == "OUTPUT":
            write = bytes([pin + 1, 119, 0])
            ser.write(write)
        # pins can only be INPUT or OUTPUT
        else:
            raise Exception("Pin mode must be set as 'INPUT' or 'OUTPUT'")
    else:
        raise Exception("Invalid pin")


def pin_write(pin, value, ser):
    """
    Sets an OUTPUT pin to 5 volts (HIGH) or 0 volts (LOW).

    Example:
        # pin 12 will be set to 5 volts.
        ser = serial.Serial('COM15', 115200)
        def update():
                pin_mode(12, 'HIGH', ser)

    Args:
        pin: (int) The GPIO pin on the Arduino you want to set.
        value: (string) The pin value, 'HIGH' or 'LOW'.
        ser: (Serial) The open, USB connection to the Arduino.

    Note:
        The pin value ranges between 0 and 19, this includes the digital
        pins and the analog pins.
    """
    # turn the pin on
    if value == "HIGH":
        write = bytes([pin + 1, 104, 0])
        ser.write(write)
    # turn the pin off
    elif value == "LOW":
        write = bytes([pin + 1, 108, 0])
        ser.write(write)
    else:
        raise Exception("Pin value must be 'HIGH' or 'LOW'")
