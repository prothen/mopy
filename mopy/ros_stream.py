#! /usr/bin/env python

""" Module to interface with motion capture system from Qualisys using ROS. """

__author__ = "Philipp Rothenhäusler"
__maintainer__ = "Philipp Rothenhäusler"
__email__ = "philipp.rothenhaeusler@gmail.com"
__copyright__ = "Copyright 2020"
__status__ = "Production"
__license__ = "BSD"
__version__ = "1.0"


from mopy.base_stream import ExternalVision

import rospy as rp

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


MM2M = 1.e-3

class ExternalVisionROS(ExternalVision):

    def __init__(self, model_names=None, debug_is_enabled=False):
        super().__init__(model_names, debug_is_enabled)
        # Create ROS node
        rp.init_node('mopy', anonymous=False)

        # Fetch ros parameters from launch file configurations
        self.models_requested = rp.get_param('~models', '').split(',')
        self.models_requested = [None if self.models_requested == [''] else
                                 self.models_requested][0]
        self.filter_active = rp.get_param('~filter_measurements', False)
        self._debug_streams = rp.get_param('~debug_streams', False)
        self._debug_is_enabled = rp.get_param('~debug_is_enabled', False)
        self._ip = rp.get_param('~ip', "11.0.0.10")

        # Configure placeholder messages and publishers
        self.messages = dict()
        self.publishers = dict()
        self._frame_id = "odom"
        self._message_type = PoseStamped

    def _apply_filter(self):
        """ Update model states and apply filtering algorithms."""
        pass


    def _define_streams(self):
        """ Initialise the interface-dependent stream objects. """
        for body_name in self.body_streams.keys():
            self.publishers.update(**{body_name: rp.Publisher('~' + body_name,
                                      PoseStamped,
                                      queue_size=1)})
            tmp_msg = self._message_type()
            tmp_msg.header.frame_id = self._frame_id

            self.messages.update(**{body_name: tmp_msg})

    def _stream_models(self):
        """ Iterate through requested model list and stream to interface. """
        for body_name, body in self.bodies.items():
            if self._debug_streams:
                print('Stream start: \t{0}'.format(body_name))
            # Update message
            msg = self.messages[body_name]
            msg.header.stamp = rp.Time.now()
            p = msg.pose.position
            q = msg.pose.orientation
            # Parse rigid body information and stream
            p.x, p.y, p.z = body.p.tolist()
            q.w, q.x, q.y, q.z = body.q.tolist()
            self.publishers[body_name].publish(msg)
            if self._debug_streams:
                print('Stream complete: \t{0}'.format(body_name))


class ExternalVisionROSFiltered(ExternalVisionROS):
    def __init__(self, model_names=None, debug_is_enabled=False):
        super().__init__(model_names, debug_is_enabled)
        self._message_type = PoseWithCovarianceStamped

        # Configure filter parameters
        # todo

    def _define_filters(self):
        pass

    def _apply_filter(self):
        # apply filter to complete rigid body information
        # todo
        pass
