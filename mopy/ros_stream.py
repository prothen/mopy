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

import numpy as np
import rospy as rp

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

MM2M = 1.e-3


class ExternalVisionROS(ExternalVision):
    """ Provides essential methods for the """

    def __init__(self, model_names=None, debug_is_enabled=False):
        super().__init__(model_names, debug_is_enabled)
        # Create ROS node
        rp.init_node('mopy', anonymous=False)

        # Fetch ros parameters from launch file configurations
        self.models_requested = rp.get_param('~models', '').split(',')
        self.models_requested = [None if self.models_requested == [''] else
                                 self.models_requested][0]
        self.filter_active = rp.get_param('~filter', False)
        use_odom = rp.get_param('~odom', False)
        use_covariance = rp.get_param('~covariance', False)
        # todo: future tf publisher
        publish_tf = rp.get_param('~publish_tf', False)
        self._frame_id = rp.get_param('~frame_id', False)
        self._debug_is_enabled = rp.get_param('~debug_is_enabled', False)
        self._debug_streams = rp.get_param('~debug_streams', False)
        self._debug_is_enabled = self._debug_is_enabled or self._debug_streams
        self._ip = rp.get_param('~ip', "11.0.0.10")

        # Configure placeholder messages and publishers
        self.messages = dict()
        self.publishers = dict()
        self._debug('\nCONFIGURATION')
        if use_odom:
            self._debug('\t--> Using Odometry message.')
            self._message_type = Odometry
            self._message_conversion = self._to_odom_message
            if self.filter_active:
                self._debug('\t\tActivated filter and parsing velocities '
                            'to Odometry message.')
                self._message_conversion = self._to_odom_message_filtered

        else:
            self._debug('\t--> Using Pose message.')
            self._message_type = PoseStamped
            self._message_conversion = self._to_pose_message
            if self.filter_active:
                self._debug('\t\tActivated filer but requesting pose message,'
                            'no velocities published.')

        if use_covariance:
            self._debug('\t--> Parsing covariance.')
            self._fill_message_pose = self._fill_pose_cov
            self._fill_message_twist = self._fill_twist_cov
        else:
            self._debug('\t--> Not parsing covariance.')
            self._fill_message_pose = self._fill_pose
            self._fill_message_twist = self._fill_twist

    def _apply_filter(self):
        """ Update model states and apply filtering algorithms."""
        pass

    def _fill_pose(self, body, pose):
        """ Fill pose with body measurements. """
        p = pose.position
        q = pose.orientation
        p.x, p.y, p.z = body.p.tolist()
        q.w, q.x, q.y, q.z = body.q.tolist()

    def _fill_twist(self, body, twist):
        """ Fill twist with body measurements. """
        v = twist.linear
        w = twist.angular
        v.x, v.y, v.z = body.v.tolist()
        w.x, w.y, w.z = body.w.tolist()

    def _fill_pose_cov(self, body, pose):
        """ Fill pose with body measurements. """
        p = pose.pose.position
        q = pose.pose.orientation
        p.x, p.y, p.z = body.p.tolist()
        q.w, q.x, q.y, q.z = body.q.tolist()
        # cov = pose.covariance

    def _fill_twist_cov(self, body, twist):
        """ Fill twist with body measurements and covariance. """
        v = twist.twist.linear
        w = twist.twist.angular
        v.x, v.y, v.z = body.v.tolist()
        w.x, w.y, w.z = body.w.tolist()
        # cov = twist.covariance

    def _to_pose_message(self, msg, body):
        """ Parse body measurement to ROS pose message. """
        pose = msg.pose
        self._fill_pose(body, pose)

    def _to_odom_message(self, msg, body):
        """ Parse body measurements to ROS odometry message. """
        pose = msg.pose.pose
        self._fill_message_pose(body, pose)

    def _to_odom_message_filtered(self, msg, body):
        """ Parse body measurements and filtered velocities to ROS
        odometry message. """
        pose = msg.pose.pose
        twist = msg.twist.twist
        self._fill_message_pose(body, pose)
        self._fill_message_twist(body, twist)

    def _define_streams(self):
        """ Initialise the interface-dependent stream objects. """
        for body_name in self.body_streams.keys():
            self.publishers.update(**{body_name: rp.Publisher('~' + body_name,
                                                              self._message_type,
                                                              queue_size=0)})
            tmp_msg = self._message_type()
            tmp_msg.header.frame_id = self._frame_id
            tmp_msg.child_frame_id = 'base_link'
            tmp_msg.pose.covariance = np.diag(np.ones(6)*1.e-3).flatten()
            #'_'.join(["base_link", body_name])

            self.messages.update(**{body_name: tmp_msg})

    def _stream_models(self):
        """ Iterate through requested model list and stream to interface. """
        for body_name, body in self.bodies.items():
            self._debug('Stream start: \t{0}'.format(body_name))

            msg = self.messages[body_name]
            msg.header.stamp = rp.Time.now()
            self._message_conversion(msg, body)
            self.publishers[body_name].publish(msg)

            self._debug('Stream complete: \t{0}'.format(body_name))


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
