#! /usr/bin/env python

""" Directly interface with motion capture system from Qualisys. """

__author__ = "Philipp Rothenhäusler"
__maintainer__ = "Philipp Rothenhäusler"
__email__ = "philipp.rothenhaeusler@gmail.com"
__copyright__ = "Copyright 2020"
__status__ = "Production"
__license__ = "BSD"
__version__ = "1.0"


import time
import numpy
import transforms3d

from mopy import base_stream as stream


if __name__ == "__main__":
    # Create external vision instance
    vision_client = stream.ExternalVision(debug_is_enabled=True)
    ## Start blocking stream in main thread
    # vision_client.run()
    ## Start multiprocessed stream
    #vision_client.run_multiprocessed()
    vision_client.run_multithreaded()
    # Change the following name to a rigid body exposed by your QTM configuration 
    testname = "pixi"

    import time
    time.sleep(2)
    while vision_client.is_ok:
        print('Continue activity')
        print(vision_client.bodies)
        rb = vision_client.bodies[testname]
        print('Received rigid body:', rb)
        time.sleep(1)

