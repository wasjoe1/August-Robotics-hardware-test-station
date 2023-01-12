#!/usr/bin/env python
from __future__ import print_function

import sys
from serial import SerialException

from boothbot_dtu.dtu_e90_config import DTUE90Config
from boothbot_dtu.dtu_as69_config import DTUAS69Config

def eprint(*args, **kwargs):
    """
    Print to stderr.

    Parameters
    ----------
    args: list
       Arguments to print
    kwargs: dict
       Keyword arguments to print
    """
    print(*args, file=sys.stderr, **kwargs)

def get_config(module = "e90", port="/dev/dtu"):
    """
    Get the configuration of the DTU.
    """
    module = module.lower()
    assert module in ["e90"], "Module has to be 'e90'"

    DTUClass = DTUE90Config

    try:
        dtu = DTUClass(port=port)
        config = {}
        config["address"] = dtu.address
        config["net_id"] = dtu.net_id
        config["parity"] = dtu.parity
        config["baudrate"] = dtu.baudrate
        config["channel"] = dtu.channel
        config["transmitting_power"] = dtu.transmitting_power
        config["air_speed"] = dtu.air_speed
        config["pack"] = dtu.pack
        del(dtu)
        return (config, '')

    except SerialException as e:
        eprint(str(e))
        return (
            None,
            "SerialException. Cannot open port {} or could not configure port".format(port)
        )
    except IndexError as e:
        return (None, "IndexError. Please make sure the dtu on config mode.")
    except Exception as e:  # TODO Add all unknown exception to be explicit
        eprint("Unknown Exception")
        eprint(str(e))
        return (None, "Unknown Exception. Please confirm port and set the dtu on config mode.")


def set_channel(channel, module="e90", port="/dev/dtu"):
    """
    Put both E90 and AS90 in a function to make API simple.

    Parameters
    ----------
    channel: int
       Target channel for DTU
    module: str
       e90/as69G
    port: str
       DTU port. Default is "/dev/dtu"

    Return
    ------
    int/None
       DTU channel after setup. If exception happens, return None.
    str
       Error msg if exception failed. If all good, return empty str.
    """

    module = module.lower()
    assert module in ["e90", "as69"], "Module has to be 'e90' or 'as69'"

    DTUClass = DTUE90Config if module == "e90" else DTUAS69Config

    try:
        dtu = DTUClass(port=port)
        dtu.set_params_default()
        dtu.set_channel(channel)
        dtu.reset()
        dtu.close()
        result = (dtu.channel, '')
        del(dtu)
        return result
    except SerialException as e:
        eprint(str(e))
        return (
            None,
            "SerialException. Cannot open port {} or could not configure port".format(port)
        )
    except IndexError as e:
        return (None, "IndexError. Please make sure the dtu on config mode.")
    except Exception as e:  # TODO Add all unknown exception to be explicit
        eprint("Unknown Exception")
        eprint(str(e))
        return (None, "Unknown Exception. Please confirm port and set the dtu on config mode.")


set_channel(17)