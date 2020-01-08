#! /usr/bin/env python

""" Module to interface with motion capture system from Qualisys. """

__author__ = "Philipp Rothenhäusler"
__maintainer__ = "Philipp Rothenhäusler"
__email__ = "philipp.rothenhaeusler@gmail.com"
__copyright__ = "Copyright 2020"
__status__ = "Production"
__license__ = "BSD"
__version__ = "1.0"

import qtm
import asyncio
import numpy as np
import signal as sg
import transforms3d.quaternions as tf

import xml.etree.ElementTree as ET

MM2M = 1.e-3

class RigidBody:
    def __init__(self, name):
        self.name = name
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.zeros(4)
        self.w = np.zeros(3)


class ExternalVision:
    def __init__(self, model_names=None, debug_is_enabled=False):
        """ Initialise the external vision class.

            If a list of model names (list of strings) is provided, the
            selected models will be streamed. Otherwise all visible models
            (at the time of the node execution) will be streamed.

        """
        self._debug_is_enabled = debug_is_enabled
        self.models_requested = model_names
        self.body_streams = dict()
        self.bodies = dict()
        self._ip = "11.0.0.10"

        # Define event loop
        self.event_loop = asyncio.get_event_loop()
        self.is_active = True

    @staticmethod
    def create_body_index(xml_string):
        """ Extract a name to index dictionary from 6dof settings xml. """
        xml = ET.fromstring(xml_string)
        body_to_index = {}
        for index, body in enumerate(xml.findall("*/Body/Name")):
            body_to_index[body.text.strip()] = index

        return body_to_index

    def _define_bodies(self):
        """ Define body dictionary with measurement state placeholders. """
        for body_name in self.body_streams.keys():
            self.bodies.update(**{body_name: RigidBody(body_name)})

    def _define_streams(self):
        """ Defines output stream specific interfaces.

            Needs to be subclassed for any streaming implementation.
        """
        pass

    def _define_filters(self):
        """ Configure and initialise filter states.

            Needs to be subclassed for problem specific filter.
        """
        pass

    def _parse_packets(self, packet):
        """ Parse packets into corresponding body placeholder objects. """
        info_received, bodies_received = packet.get_6d()
        for body_name, body_idx in self.body_streams.items():
            p, R = bodies_received[body_idx]
            p_m = np.array([p.x, p.y, p.z]) * MM2M
            R_m = np.array(R.matrix).reshape((3, 3), order='F')
            if any(np.isnan(p_m)) or any(np.isnan(R_m.flatten())):
                continue

            # Parse measured states p_m and q_m
            body = self.bodies[body_name]
            q_m = tf.mat2quat(R_m)
            body.p, body.q = p_m, q_m

    def _apply_filter(self):
        """ Update model states and apply filtering algorithms."""
        pass

    def _stream_models(self):
        """ Define the streaming interface and update output values.

            Subclassed in interface-dependent implementation.
        """
        pass

    async def parse_packets(self, queue):
        """ Process all measurements in queue. """
        while self.is_active:
                self._parse_packets(await queue.get())
                self._apply_filter()
                self._stream_models()

    async def loop(self):
        """ Execute the main event loop."""
        try:
            print('Executing event loop now.')
            # Return QRT Connection object (see qtm -> qrt.py)
            connection = await qtm.connect(self._ip)

            # Terminate loop in case of unsuccessful connection attempt
            if connection is None:
                print("Unsuccessful attempt to establish connection to QTM.")
                return

            # Get 6DOF settings from QTM
            xml_string = await connection.get_parameters(parameters=["6d"])
            dict_bodies_advertised = self.create_body_index(xml_string)

            if self._debug_is_enabled:
                print('Advertised models: {0}'.format(dict_bodies_advertised))
                print('Requested models: {0}'.format(self.models_requested))

            if self.models_requested is not None:
                for name, idx in dict_bodies_advertised.items():
                    if name in self.models_requested:
                        if self._debug_is_enabled:
                            print('Found requested model: {0}'.format(name))
                        self.body_streams.update(**{name: idx})
                for model in self.models_requested:
                    if model not in self.body_streams.keys():
                        if self._debug_is_enabled:
                            print('Body: {0} is not advertised by QTM. '
                                  'Revise Setup.'.format(model))
            else:
                if self._debug_is_enabled:
                    print('No model names provided. Stream all models.')
                self.body_streams = dict_bodies_advertised
            if not self.body_streams:
                print('No models advertised by Qualisys')
                self.event_loop.call_soon_threadsafe(self.event_loop.stop)

            self._define_bodies()
            self._define_filters()
            self._define_streams()

            queue = asyncio.Queue()
            asyncio.create_task(self.parse_packets(queue))

            # Start streaming frames
            await connection.stream_frames(components=["6d"],
                                           on_packet=queue.put_nowait)

            # Idle event loop and keep streaming data
            await self.publish_stream()

        except asyncio.CancelledError:
            print('Execution of loop interrupted.')
            self.event_loop.stop()

    async def publish_stream(self):
        """ Execute idle loop to maintain streaming in background. """
        print('Streaming is active for the models:\n--> {0}'.
              format(list(self.body_streams.keys())))
        while self.is_active:
            await asyncio.sleep(0.5)

        return self.event_loop.create_future()

    def run(self):
        """ Execute the event loop. """
        main_loop = asyncio.ensure_future(self.loop())
        self.event_loop.add_signal_handler(sg.SIGINT, main_loop.cancel)
        self.event_loop.add_signal_handler(sg.SIGTERM, main_loop.cancel)
        self.event_loop.run_until_complete(main_loop)


class RigidBodyFilter:
    def __init__(self):
        self.placeholder = None
        self.v_prev = np.zeros((3, 1))
        self.w_prev = np.zeros((3, 1))
        # todo generic filter for v and w
