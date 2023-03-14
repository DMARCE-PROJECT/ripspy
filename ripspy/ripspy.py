from ament_index_python.packages import get_package_share_directory
import base64
import os
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

from ripspy.ripscontext.ripscontext import RipsContext

from subprocess import Popen

from time import sleep
from threading import Thread
import queue

import yaml
import socket
import sys

# All missing asserts for isinstance are deleted because of
# this error: Subscripted generics cannot be used with class 
# and instance checks. For example, this one raises the error:
# assert isinstance(pubs, List[rclpy.node.TopicEndpointInfo])

class RipsCore(Node):
    """Rips node."""

    __slots__ = [
        '_context',
        '_socket',
    ]

    __IGNORED_TOPICS = ["/rosout"]
    __POLLING_TIME = 0.5  # secs
    __QUEUE_DEPTH = 100 

    _context: RipsContext
    _socket: socket.socket
    _queue: queue.Queue
 
    def __init__(self, sock: socket.socket, q: queue.Queue):
        super().__init__('rips')
        self._socket = sock
        self._queue = q
        self.create_timer(self.__POLLING_TIME, self.timer_callback)
        self._context = RipsContext()

    def _send_to_engine(self, m: str):
        assert isinstance(m, str)
        try:
            self._socket.sendall(m.encode(encoding = 'UTF-8'))
        except:
            self.get_logger().warning(f"can't send event to the fifo")

    def _send_graph_event(self):
        s = (
                f"---\n"
                f"event: graph\n"
                f"{self._context.to_yaml()}"
                f"...\n\n"
        )
        self._send_to_engine(s)

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
                f"event: message\n"
                f"fromtopic: {topic}\n"
                f"msg:\n"
                f"  {msg}\n"
                f"rawmsg: |\n"
                f"  {rawmsg}\n"
                f"{self._context.to_yaml()}"
                f"...\n\n"
        )
        self._send_to_engine(s)

    def _subscribe(self, topic: str): 
        assert isinstance(topic, str)
        if topic in self.__IGNORED_TOPICS:
            return 
        self.get_logger().info(f"subscribing to topic {topic}")
        try:
            params = self._context.params_of(topic)
        except:
            self.get_logger().warning(f"can't subscribe to topic {topic}: no such topic")
            return
        if len(params) != 1:
            self.get_logger().warning(f"Topic {topic} has more than one type")
            return 
        msgtype = get_message(params[0]) 
        def f(binmsg):
            ## we want both the serialized (binary) msg and the python message
            pymsg = deserialize_message(binmsg, msgtype)
            self._send_msg_event(topic, message_to_yaml(pymsg), binmsg);
            #self.get_logger().info(f"Received from topic {topic}")
        subscription = self.create_subscription(
            msgtype,
            topic,
            f,
            self.__QUEUE_DEPTH,
            raw=True
        ) 

    def timer_callback(self):
        nodes = self.get_node_names()
        self._context.update_nodes(nodes)
        for elem in nodes:
            services = self.get_service_names_and_types_by_node(elem, "/")
            self._context.update_services(elem, services)
        topics = self.get_topic_names_and_types()
        for elem in topics:
            pubs = self.get_publishers_info_by_topic(elem[0])
            subs = self.get_subscriptions_info_by_topic(elem[0])
            self._context.update_topic(elem[0], elem[1], pubs, subs)
            self._context.update_gids(pubs)
            self._context.update_gids(subs)
            if not self._context.is_subscribed("rips", elem[0]):
                self._subscribe(elem[0])
        if self._context.check_and_clear():
            self._send_graph_event()
            self.get_logger().info("Context changed")

        while not self._queue.empty():
            d = self._queue.get()
            assert isinstance(d, dict)
            if "level" in d:
                level = d.get("level")
                self.get_logger.info(f"NEW LEVEL: {level}")
            elif "alert" in d:
                alert = d.get("alert")
                self.get_logger.info(f"RIPS ALERT: {alert}")
            else:
                self.get_logger.err("BAD DICT IN QUEUE")


AQUI


def from_engine_thread(sock: socket.socket, q: queue.Queue):
    f = os.fdopen(socket.fileno())
start=False
yamfile = ""
for line in f:
    print(line)
    if "---" == line:
        start = True
    if start:
        yamfile = f"{yamfile}\n{line}\n"
    if "..." == line:
        start = False
        print(yamfile)
        yamfile = ""
       
        d = yaml.safe_load(yamfile)
        print()
        q.put(d)

def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger("rips")
    sockpath = os.environ.get('RIPSSOCKET', "/tmp/rips.socket")
    rulespath = os.environ.get('RIPSRULES', '')
    scriptspath = os.environ.get('RIPSSCRIPTS', '')
    if rulespath == '':
        logger.err("RIPSRULES environment variable not defined")
        os._exit(1)  
    sharepath = get_package_share_directory('ripspy')
    proc = Popen([sharepath+'/bin/rips', rulespath, sockpath, scriptspath])
    sleep(2) 
    if proc.poll() != None:
        logger.err("go process not ready, aborting")
        os._exit(1)
    logger.info(f"connecting to unix socket {sockpath}")   
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(sockpath)
    except:
        logger.err("can't connect to socket, aborting")  
        proc.kill()
        os.exit(1)
    logger.info("connected, creating thread")
    q = queue.Queue()
    sockthread = Thread(target=from_engine_thread, args=[sock, q])
    sockthread.start()
    logger.info("RIPS core is ready")

    rips_core = RipsCore(sock, q)
    rclpy.spin(rips_core)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rips_core.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()