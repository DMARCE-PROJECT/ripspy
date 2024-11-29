## Copyright (C) 2023  Enrique Soriano <enrique.soriano@urjc.es>
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.

import base64
import os
import queue
import socket
from subprocess import Popen
from threading import Thread
from time import sleep
from typing import Dict, List, Tuple
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from ripspy.ripscontext.ripscontext import RipsContext
import sys
from system_modes_msgs.srv import ChangeMode
import subprocess
import _io

class RipsCore(Node):
    """Rips node."""

    __slots__ = [
        '_customcontext',
        '_socket',
        '_teesocket',
        '_ripscoreq',
        '_currentlevel',
        '_currentgrav',
        '_lastalert',
        '_whitelist',
        '_blacklist',
        '_subscribedto',
        '_needmsgs',
        '_needraw',
    ]

    __DEFAULT_POLLING_TIME = 0.5
    __QUEUE_DEPTH = 100
    __LEVEL_PARAM_NAME = "systemmode"

    _pollingtime = __DEFAULT_POLLING_TIME
    _customcontext: RipsContext
    _socket: socket.socket
    _teesocket: socket.socket
    _ripscoreq: queue.Queue
    _currentlevel: str
    _currentgrav: float
    _lastalert: str
    # if the whitelist is empty, all topics are ok except those in the blacklist
    _whitelist: List[str]
    _blacklist: List[str]
    # Not to be confused with Node._subscriptions. This is used for
    # local accounting, used to ensure just one subscription per topic, it's
    # just a paranoid double-check. This list should have the same subcriptions
    # than Node._subscriptions.
    _localsubscriptions: List[rclpy.subscription.Subscription]
    _needmsgs: bool
    _needraw: bool

    def __init__(self, sock: socket.socket, teesock: socket.socket, coreq: queue.Queue, nmsg: bool, nraw: bool):
        assert isinstance(sock, socket.socket)
        assert teesock == None or isinstance(teesock, socket.socket)
        assert isinstance(coreq, queue.Queue)
        assert isinstance(nmsg, bool)
        assert isinstance(nraw, bool)
        super().__init__('rips')
        self._socket = sock
        self._teesocket = teesock
        self._ripscoreq = coreq
        self.create_timer(self._pollingtime, self.timer_callback)
        self._customcontext = RipsContext()
        self.declare_parameter(self.__LEVEL_PARAM_NAME, "__DEFAULT__")
        self._currentlevel = "init"
        self._currentgrav = 0.0
        self._lastalert = ""
        self._whitelist = []
        self._blacklist = ["/rosout"]
        self._localsubscriptions = []
        self._needmsgs = nmsg
        self._needraw = nraw
        bl = os.environ.get('RIPSBLACKLIST', '')
        if bl != '':
            self._blacklist = self._blacklist +  bl.split(":")
        wl = os.environ.get('RIPSWHITELIST', '')
        if wl != '':
            self._whitelist = self._whitelist +  wl.split(":")
        try:
            polling = os.environ.get('RIPSPOLLING', '')
            if polling != '':
                if float(polling) > 0.0 and float(polling) < 10.0:
                    self._pollingtime = float(polling)
                else:
                    self.get_logger().warning(f"RIPSPOLLING must be in (0.0 .. 10.0)")
        except:
            self.get_logger().warning(f"RIPSPOLLING variable is not a number")
        self.get_logger().info(f"Polling: {self._pollingtime}")
        self.get_logger().info(f"Blacklist: {self._blacklist}")
        self.get_logger().info(f"Whitelist: {self._whitelist}")
        self.get_logger().info(f"Become subscriptor: {self._needmsgs}")
        self.get_logger().info(f"Processing messages' payload: {self._needraw}")

    def _send_data(self, m: str):
        assert isinstance(m, str)
        try:
            self._socket.sendall(m.encode(encoding = 'UTF-8'))
        except:
            self.get_logger().warning(f"can't send event to rips")
        if self._teesocket != None:
            try:
                self._teesocket.sendall(m.encode(encoding = 'UTF-8'))
            except:
                self.get_logger().warning(f"can't send event to ripspydash")

    def _send_graph_event(self):
        s = (
                f"---\n"
                f"currentlevel: {self._currentlevel}\n"
                f"currentgrav: {self._currentgrav}\n"
                f"lastalert: '{self._lastalert}'\n"
                f"event: graph\n"
                f"{self._customcontext.to_yaml()}"
                f"...\n\n"
        )
        self._send_data(s)

    def _send_level_only(self):
        s = (
                f"---\n"
                f"currentlevel: {self._currentlevel}\n"
                f"currentgrav: {self._currentgrav}\n"
                f"event: level\n"
                f"...\n\n"
        )
        self._send_data(s)

    def _send_msg_event(self, topic: str, raw: bytes):
        assert isinstance(topic, str)
        assert isinstance(raw, bytes)
        rawmsg = ''
        if self._needraw:
            rawmsg = base64.encodebytes(raw).decode()
            rawmsg = "  ".join(rawmsg.splitlines(True))
        s = (
                f"---\n"
                f"currentlevel: {self._currentlevel}\n"
                f"currentgrav: {self._currentgrav}\n"
                f"lastalert: '{self._lastalert}'\n"
                f"event: message\n"
                f"fromtopic: {topic}\n"
                # deprecated
                f"msg: null\n"
                f"rawmsg: |\n"
                f"  {rawmsg}\n"
                f"{self._customcontext.to_yaml()}"
                f"...\n\n"
        )
        self._send_data(s)

    # double check for paranoids
    def _already_subscribed(self, topic: str) -> bool:
        assert isinstance(topic, str)
        for s in self._localsubscriptions:
            if s.topic_name == topic:
                return True
        return False

    def _subscribe(self, topic: str):
        assert isinstance(topic, str)
        if topic in self._blacklist:
            return
        if len(self._whitelist) > 0 and not topic in self._whitelist:
            return
        if self._already_subscribed(topic):
            self.get_logger().error(f"Already subscribed to {topic}, this should not happen!")
            return
        self.get_logger().info(f"Subscribing to topic {topic}")
        try:
            params = self._customcontext.params_of(topic)
        except:
            self.get_logger().warning(f"Can't subscribe to topic {topic}: no such topic")
            return
        if len(params) != 1:
            self.get_logger().warning(f"Topic {topic} has more than one type, skipping")
            return
        try:
            msgtype = get_message(params[0])
        except:
            self.get_logger().warning(f"Unknown message type {params[0]}, {topic} is blacklisted")
            self._blacklist.append(topic)
            return
        def f(binmsg):
            ## We need to save time to receive from high frequency topics such
            ## as cameras, etc. Now, the context is only updated by polling.
            ## Note that, now, we can receive a message from a node that is
            ## not in the context (because it showed up after the last update).
            ## Anyway, we observed that in some ros2 configurations (tiago),
            ## some nodes are not visible (but the topics are). So, in practice, you
            ## could receive msgs from nodes that are not in the context. Therefore,
            ## we can save time here skipping the update.
            ## self._update_context()
            self._send_msg_event(topic, binmsg)
            ## self.get_logger().info(f"processed message, type: {msgtype}")
        subscription = self.create_subscription(
            msgtype,
            topic,
            f,
            self.__QUEUE_DEPTH,
            raw = True
        )
        self._localsubscriptions.append(subscription)

    def _invoke_service(self, level: str):
        assert isinstance(level, str)
        self.cli = self.create_client(ChangeMode, '/safety/change_mode')
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('service /safety/change_mode not available')
            return None
        self.req = ChangeMode.Request()
        self.req.mode_name = level
        self.get_logger().warning('invoking service /safety/change_mode')
        self.cli.call_async(self.req)
        self.get_logger().warning('service invoked')

    ## two methods to notify system modes: parameter and service
    def _set_level(self, level: str, grav: float):
        assert isinstance(level, str)
        assert isinstance(grav, float)
        self._currentlevel = level
        self._currentgrav = grav
        p = self.get_parameter(self.__LEVEL_PARAM_NAME)
        if p.get_parameter_value != level:
            type = rclpy.Parameter.Type.STRING
            param = rclpy.Parameter(self.__LEVEL_PARAM_NAME, type, level)
            self.set_parameters([param])
        self._invoke_service(level)
        self._send_level_only()


    def _set_lastalert(self, alert: str):
        assert isinstance(alert, str)
        self._lastalert = alert

    def timer_callback(self):
        self._update_context()
        while not self._ripscoreq.empty():
            d = self._ripscoreq.get()
            assert isinstance(d, dict)
            if "level" in d:
                level = d.get("level")
                self.get_logger().info(f"New level: {level}")
                grav = d.get("gravity")
                assert isinstance(grav, float)
                self._set_level(level, grav)
            elif "alert" in d:
                alert = d.get("alert")
                self.get_logger().info(f"Rips alert: {alert}")
                self._set_lastalert(alert)
            else:
                self.get_logger().error("Bad dict from queue")

    def _update_context(self):
        nodes = self.get_node_names()
        self._customcontext.update_nodes(nodes)
        for elem in nodes:
            try:
                services = self.get_service_names_and_types_by_node(elem, "/")
                self._customcontext.update_services(elem, services)
            except:
                self.get_logger().warning(f"error getting info for node: " + elem)
        topics = self.get_topic_names_and_types()
        for elem in topics:
            pubs = self.get_publishers_info_by_topic(elem[0])
            subs = self.get_subscriptions_info_by_topic(elem[0])
            self._customcontext.update_topic(elem[0], elem[1], pubs, subs)
            self._customcontext.update_gids(pubs)
            self._customcontext.update_gids(subs)
            if self._needmsgs and not self._customcontext.is_subscribed("rips", elem[0]):
                self._subscribe(elem[0])
        if self._customcontext.check_and_clear():
            self._send_graph_event()
            self.get_logger().info("Context changed")

def read_yaml(f: _io.TextIOWrapper):
    assert isinstance(f, _io.TextIOWrapper)
    start = False
    s = ""
    for line in f:
        if start:
            s = s + f"{line}"
        if line == "---\n":
            start = True
            s = "---\n"
        if line == "...\n":
            s = s + f"{line}"
            break
    try:
        d = yaml.safe_load(s)
    except:
        print(f"Read_yaml error: bad yaml\n{s}\n", file=sys.stderr)
        return None
    return d

def from_engine_thread(f: _io.TextIOWrapper, q: queue.Queue):
    assert isinstance(f, _io.TextIOWrapper)
    assert isinstance(q, queue.Queue)
    while True:
        d = read_yaml(f)
        if d == None:
            print(f"Engine thread: can't read yaml from socket\n", file=sys.stderr)
            os._exit(1)
        q.put(d)

def connect_to_engine(logger: rclpy.impl.rcutils_logger.RcutilsLogger) -> tuple[subprocess.Popen, socket.socket]:
    assert isinstance(logger, rclpy.impl.rcutils_logger.RcutilsLogger)
    rulespath = os.environ.get('RIPSRULES', '')
    scriptspath = os.environ.get('RIPSSCRIPTS', '')
    if rulespath == '' or scriptspath == '':
        logger.error("RIPSRULES or RIPSCRIPTS environment variables are not defined")
        os._exit(1)
    sockpath = os.environ.get('RIPSSOCKET', "/tmp/rips.socket")
    try:
        os.remove(sockpath)
    except:
        pass
    sharepath = get_package_share_directory('ripspy')
    ## for debugging: -D, -DD...
    proc = Popen([sharepath+'/bin/rips', "-s", sockpath, scriptspath, rulespath])
    if proc.poll() != None:
        logger.error("Go process not ready, aborting")
        os._exit(1)
    attempts = 0
    ready = False
    # wait for the socket
    while not ready:
        ready = os.access(sockpath, os.F_OK)
        if not ready:
            attempts += 1
            if attempts == 4:
                proc.kill()
                logger.error("Socket path is not ready, aborting")
                os._exit(1)
            sleep(0.5)
    logger.info(f"Connecting to unix socket {sockpath}")
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(sockpath)
    except:
        logger.error("Can't connect to socket, aborting")
        proc.kill()
        os._exit(1)
    return proc, sock

def handshake(f: _io.TextIOWrapper, logger: rclpy.impl.rcutils_logger.RcutilsLogger) -> tuple[bool, bool]:
    assert isinstance(f, _io.TextIOWrapper)
    assert isinstance(logger, rclpy.impl.rcutils_logger.RcutilsLogger)
    d = read_yaml(f)
    if d == None or (not "msg" in d or not "raw" in d):
        logger.error(f"Handshake failed: {d}")
        os._exit(1)
    return  d.get("msg"), d.get("raw")

def dash(logger: rclpy.impl.rcutils_logger.RcutilsLogger) -> socket.socket:
    assert isinstance(logger, rclpy.impl.rcutils_logger.RcutilsLogger)
    teesockpath = os.environ.get('RIPSTEESOCKET', "")
    dashsock = None
    if teesockpath != "":
        try:
            dashsock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            dashsock.connect(teesockpath)
        except:
                logger.error("Can't connect to dash socket: " + teesockpath + ", aborting")
                proc.kill()
                os._exit(1)
    return dashsock

def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger("rips")
    proc, sock = connect_to_engine(logger)
    try:
        fsock = os.fdopen(sock.fileno())
    except:
        logger.error(f": Can't get the socket's fd\n")
        proc.kill()
        os._exit(1)
    needmsgs, needraw = handshake(fsock, logger)
    logger.info("Connected, creating ripscore thread")
    ripsq = queue.Queue()
    sockthread = Thread(target=from_engine_thread, args=[fsock, ripsq])
    sockthread.start()
    dashsock = dash(logger)
    rips_core = RipsCore(sock, dashsock, ripsq, needmsgs, needraw)
    logger.info("RIPS core is ready")
    rclpy.spin(rips_core)
    proc.kill()
    sockthread.join()
    rips_core.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
