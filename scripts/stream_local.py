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


from mopy import base_stream as stream


if __name__ == "__main__":
    # Create external vision instance
    vision_client = stream.ExternalVisionLocal()
    ## Start blocking stream in main thread
    # vision_client.run()
    ## Start multiprocessed stream
    vision_client.run_multiprocessed()

    while vision_client.is_ok:
        print('Continue activity')
        rb = vision_client.bodies['SampleBodyName']
        print('Received rigid body:', rb)
        time.sleep(1)

