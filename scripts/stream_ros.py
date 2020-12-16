#! /usr/bin/env python

""" Interface with motion capture system from Qualisys using ROS. """

__author__ = "Philipp Rothenhäusler"
__maintainer__ = "Philipp Rothenhäusler"
__email__ = "philipp.rothenhaeusler@gmail.com"
__copyright__ = "Copyright 2020"
__status__ = "Production"
__license__ = "BSD"
__version__ = "1.0"

from mopy import ros_stream as stream

if __name__ == "__main__":
    vision_client = stream.ExternalVisionROS()
    vision_client.run()
