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

# All missing asserts for isinstance are deleted because of
# this error: Subscripted generics cannot be used with class
# and instance checks. For example, this one raises the error:
# assert isinstance(pubs, List[rclpy.node.TopicEndpointInfo])

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
    ]

    __POLLING_TIME = 0.5  # secs
    __QUEUE_DEPTH = 100
    __LEVEL_PARAM_NAME = "systemmode"

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
    # Not to be confused with Node._subscriptions. This is for
    # local accounting, used to ensure just one subscription per topic, it's
    # just a paranoid double-check. This list should have the same subcriptions
    # than Node._subscriptions.
    _localsubscriptions: List[rclpy.subscription.Subscription]

    def __init__(self, sock: socket.socket, teesock: socket.socket, coreq: queue.Queue):
        super().__init__('rips')
        self._socket = sock
        self._teesocket = teesock
        self._ripscoreq = coreq
        self.create_timer(self.__POLLING_TIME, self.timer_callback)
        self._customcontext = RipsContext()
        self.declare_parameter(self.__LEVEL_PARAM_NAME, "__DEFAULT__")
        self._currentlevel = "init"
        self._currentgrav = 0.0
        self._lastalert = ""
        self._whitelist = []
        self._blacklist = ["/rosout"]
        self._localsubscriptions = []
        bl = os.environ.get('RIPSBLACKLIST', '')
        if bl != '':
            self._blacklist = self._blacklist +  bl.split(":")
        wl = os.environ.get('RIPSWHITELIST', '')
        if wl != '':
            self._whitelist= self._whitelist +  wl.split(":")

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

    def _send_msg_event(self, topic: str, msg: str, raw: bytes):
        assert isinstance(topic, str)
        assert isinstance(msg, str)
        assert isinstance(raw, bytes)
        # possible bug: with message_to_yaml, long String messages
        # may generate lines too long for YAML (max. line is 80)
        msg = "  ".join(msg.splitlines(True))
        rawmsg = base64.encodebytes(raw).decode()
        rawmsg = "  ".join(rawmsg.splitlines(True))
        s = (
                f"---\n"
                f"currentlevel: {self._currentlevel}\n"
                f"currentgrav: {self._currentgrav}\n"
                f"lastalert: '{self._lastalert}'\n"
                f"event: message\n"
                f"fromtopic: {topic}\n"
                f"msg:\n"
                f"  {msg}\n"
                f"rawmsg: |\n"
                f"  {rawmsg}\n"
                f"{self._customcontext.to_yaml()}"
                f"...\n\n"
        )
        self._send_data(s)

    # double check for paranoids
    def _already_subscribed(self, topic: str) -> bool:
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
            ## we want both the serialized (binary) msg and the python message
            try:
                pymsg = deserialize_message(binmsg, msgtype)
            except:
                self.get_logger().warning(f"Handle: can't deserilialize message, type: {msgtype}")
                return
            self._update_context()
            self._send_msg_event(topic, message_to_yaml(pymsg), binmsg)
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
        self.cli.call_async(self.req)

    ## two methods to notify system modes: parameter and service
    def _set_level(self, level: str, grav: float):
        self._currentlevel = level
        self._currentgrav = grav
        p = self.get_parameter(self.__LEVEL_PARAM_NAME)
        if p.get_parameter_value != level:
            type = rclpy.Parameter.Type.STRING
            param = rclpy.Parameter(self.__LEVEL_PARAM_NAME, type, level)
            self.set_parameters([param])
        self._invoke_service(level)

    def _set_lastalert(self, alert: str):
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
                self.get_logger().error("BAD DICT IN QUEUE")

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
            if not self._customcontext.is_subscribed("rips", elem[0]):
                self._subscribe(elem[0])
        if self._customcontext.check_and_clear():
            self._send_graph_event()
            self.get_logger().info("Context changed")

def from_engine_thread(sock: socket.socket, q: queue.Queue):
    f = os.fdopen(sock.fileno())
    start = False
    s = ""
    for line in f:
        if start:
            s = s + f"{line}"
        if line == "---\n":
            start = True
            s = "---\n"
        if line == "...\n":
            start = False
            try:
                d = yaml.safe_load(s)
                q.put(d)
            except:
                print(f"ERROR: sockthread: bad YAML\n{s}\n", file=sys.stderr)

def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger("rips")
    sockpath = os.environ.get('RIPSSOCKET', "/tmp/rips.socket")
    rulespath = os.environ.get('RIPSRULES', '')
    scriptspath = os.environ.get('RIPSSCRIPTS', '')
    if rulespath == '' or scriptspath == '':
        logger.error("RIPSRULES or RIPSCRIPTS environment variables are not defined")
        os._exit(1)
    sharepath = get_package_share_directory('ripspy')
    proc = Popen([sharepath+'/bin/rips', "-s", sockpath, scriptspath, rulespath])
    # Give some time to the engine to start and create the socket. Note that this
    # is not a trivial issue: (i) the engine needs some time to create the socket;
    # (ii) the socket may exist from a previous (incorrect) execution; (iii) the engine
    # can crash while starting (before creating the socket).
    # The race is worse without this delay (it's not ideal, but it works).
    # Posible solution: use inotify to watch the path, overkill??
    # Another one: change the protocol, add a hello message and set an alarm. overkill??
    # Polling is not necessary, just wait 2 sec and go.
    sleep(2)
    if proc.poll() != None:
        logger.error("go process not ready, aborting")
        os._exit(1)
    logger.info(f"connecting to unix socket {sockpath}")
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(sockpath)
    except:
        logger.error("can't connect to socket, aborting")
        proc.kill()
        os._exit(1)

    logger.info("connected, creating ripscore thread")
    ripsq = queue.Queue()
    sockthread = Thread(target=from_engine_thread, args=[sock, ripsq])
    sockthread.start()

    teesockpath = os.environ.get('RIPSTEESOCKET', "")
    dashsock = None
    if teesockpath != "":
        try:
            dashsock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            dashsock.connect(teesockpath)
        except:
                logger.error("can't connect to dash socket: " + teesockpath + ", aborting")
                proc.kill()
                os._exit(1)

    rips_core = RipsCore(sock, dashsock, ripsq)
    logger.info("RIPS core is ready")

    rclpy.spin(rips_core)
    proc.kill()
    sockthread.join()
    rips_core.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
