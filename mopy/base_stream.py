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
import attr
import numpy
import signal
import typing
import asyncio
import concurrent.futures
import xml.etree.ElementTree as ET
import transforms3d.quaternions as tf

MM2M = 1.e-3

@attr.s
class RigidBody:
    name = attr.ib(type=str)
    p = attr.ib(default=numpy.zeros(3), type=numpy.ndarray)
    v = attr.ib(default=numpy.zeros(3), type=numpy.ndarray)
    R = attr.ib(default=numpy.zeros((3, 3)), type=numpy.ndarray)
    q = attr.ib(default=numpy.zeros((4,1)), type=numpy.ndarray)
    w = attr.ib(default=numpy.zeros(3), type=numpy.ndarray)

    def __str__(self):
        return ("\t Position: {}\n".format(self.p) + 
                "\t Velocity: {}\n".format(self.v) + 
                "\t Attitudeq: {}\n".format(self.q) + 
                "\t Angular velocity: {}\n".format(self.w)) 


@attr.s
class ExternalVision:

    """ Initialise the external vision class.

        If a list of model names (list of strings) is provided, the
        selected models will be streamed. Otherwise all models
        configured in the Qualisys server will be streamed.

    """
    stream_models = attr.ib(default=None, type=typing.Optional[list])
    body_streams = attr.ib(default=None, type=typing.Optional[dict])
    bodies = attr.ib(default=None, type=typing.Optional[dict])
    ip = attr.ib(default="11.0.0.10", type=str)
    callback_handle = attr.ib(default=None, type=typing.Optional[float])
    _debug_is_enabled = attr.ib(default=False, type=bool)

    event_loop = attr.ib(default=None, type=typing.Optional[asyncio.BaseEventLoop])
    is_ok = False

    def __attrs_post_init__(self):
        self.bodies = dict()
        self.body_streams = dict()

    @staticmethod
    def create_body_index(xml_string):
        """ Extract a name to index dictionary from 6dof settings xml. """
        xml = ET.fromstring(xml_string)
        body_to_index = {}
        for index, body in enumerate(xml.findall("*/Body/Name")):
            body_to_index[body.text.strip()] = index

        return body_to_index

    def _debug(self, msg):
        """ Print debug message if debug_is_enabled is true. """
        if self._debug_is_enabled:
            print(msg)

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
            p_m = numpy.array([p.x, p.y, p.z]) * MM2M
            R_m = numpy.array(R.matrix).reshape((3, 3), order='F')

            if any(numpy.isnan(p_m)) or any(numpy.isnan(R_m.flatten())):
                continue

            # Parse measured states p_m and R_m
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
        loop = asyncio.get_event_loop()
        while self.is_ok:
            print('wait for conection')
            self._parse_packets(await queue.get())
            print('aeu')
            # self._parse_packets(await queue.get())
            self._apply_filter()
            self._stream_models()
            print('received')
            await self.callback_handle(self.bodies)

    async def loop(self):
        """ Execute the main event loop."""
        try:
            print('Executing event loop now.')
            # Return QRT Connection object (see qtm -> qrt.py)
            connection = await qtm.connect(self.ip)

            # Terminate loop in case of unsuccessful connection attempt
            if connection is None:
                print("Unsuccessful attempt to establish connection to QTM.")
                return

            # Get 6DOF settings from QTM
            xml_string = await connection.get_parameters(parameters=["6d"])
            dict_bodies_advertised = self.create_body_index(xml_string)

            if self._debug_is_enabled:
                print('Advertised models: {0}'.format(dict_bodies_advertised))
                print('Requested models: {0}'.format(self.stream_models))

            if self.stream_models is not None:
                for name, idx in dict_bodies_advertised.items():
                    if name in self.stream_models:
                        if self._debug_is_enabled:
                            print('Found requested model: {0}'.format(name))
                        self.body_streams.update(**{name: idx})
                for model in self.stream_models:
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

            self.is_ok = True
            # Start streaming frames
            await connection.stream_frames(components=["6d"],
                                           on_packet=queue.put_nowait)

            # Idle event loop and keep streaming data
            await self.idle()

        except asyncio.CancelledError:
            print('Execution of loop interrupted.')
            self.event_loop.stop()

    async def idle(self):
        """ Execute idle loop to maintain streaming in background. """
        print('Streaming is active for the models:\n--> {0}'.
              format(list(self.body_streams.keys())))
        while self.is_ok:
            await asyncio.sleep(3600)
        ## NOTE: Remove blocking placeholder
        #return self.event_loop.create_future()

    def run(self):
        """ Execute the event loop. """
        loop = asyncio.get_event_loop()
        task = asyncio.ensure_future(self.loop())
        loop.add_signal_handler(signal.SIGINT, task.cancel)
        loop.add_signal_handler(signal.SIGTERM, task.cancel)
        loop.run_until_complete(task)

    # LEGACY
    @staticmethod
    def tramp(corout, *args): 
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        task = asyncio.ensure_future(corout(*args))
        loop.add_signal_handler(signal.SIGINT, task.cancel)
        loop.add_signal_handler(signal.SIGTERM, task.cancel)
        loop.run_until_complete(task)
        #loop.run_forever()
 
    async def _jumpad(self, pool):
        loop = asyncio.get_event_loop()
        # loop = asyncio.get_running_loop()
        print('starting loop')
        loop.run_in_executor(pool, self.tramp, self.loop)
        print('launched jumpad')
        # await asyncio.sleep(2)
    
    def run_multiprocessed(self):
        print('start multiprocessed')
        loop = asyncio.get_event_loop()
        pool = concurrent.futures.ProcessPoolExecutor()
        loop.run_until_complete(self._jumpad(pool))

    def run_multithreaded(self):
        loop = asyncio.get_event_loop()
        pool = concurrent.futures.ThreadPoolExecutor()
        loop.run_until_complete(self._jumpad(pool))


class RigidBodyFilter:
    def __init__(self):
        self.placeholder = None
        self.v_prev = numpy.zeros((3, 1))
        self.w_prev = numpy.zeros((3, 1))
        # todo generic filter for v and w
