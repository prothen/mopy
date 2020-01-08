#! /usr/bin/env python

""" Directly interface with motion capture system from Qualisys. """

__author__ = "Philipp Rothenhäusler"
__maintainer__ = "Philipp Rothenhäusler"
__email__ = "philipp.rothenhaeusler@gmail.com"
__copyright__ = "Copyright 2020"
__status__ = "Production"
__license__ = "BSD"
__version__ = "1.0"

from mopy import local_stream as stream

if __name__ == "__main__":
    vision_client = stream.ExternalVisionLocal()
    vision_client.run()