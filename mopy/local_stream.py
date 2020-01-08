#! /usr/bin/env python

""" Module to directly interface with motion capture system from Qualisys. """

__author__ = "Philipp Rothenhäusler"
__maintainer__ = "Philipp Rothenhäusler"
__email__ = "philipp.rothenhaeusler@gmail.com"
__copyright__ = "Copyright 2020"
__status__ = "Production"
__license__ = "BSD"
__version__ = "1.0"

import transforms3d.quaternions as tf
import concurrent.futures

from mopy.base_stream import ExternalVision

MM2M = 1.e-3

class ExternalVisionLocal(ExternalVision):

    def __init__(self, model_names=None, debug_is_enabled=False):
        super().__init__(model_names, debug_is_enabled)
        self.executor = concurrent.futures.ProcessPoolExecutor()

    def _define_streams(self):
        pass

    def _stream_models(self):
        pass



class ExternalVisionLocalFiltered(ExternalVisionLocal):
    def __init__(self, model_names=None, debug_is_enabled=False):
        super().__init__(model_names, debug_is_enabled)

        # Configure filter parameters
        # todo

    def _apply_filter(self):
        # apply filter to complete rigid body information
        # todo
        pass
