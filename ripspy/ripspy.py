import os

import typing
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos_event import SubscriptionEventCallbacks
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

from rosidl_runtime_py import message_to_yaml

# all missing asserts for isinstance are deleted because of
# this error: Subscripted generics cannot be used with class 
# and instance checks. For exameple, this one raises the error:
# assert isinstance(pubs, List[rclpy.node.TopicEndpointInfo])

class RipsTopic:
    """Data of interest for a topic"""

    __slots__ = [
        '_name',
        '_parameters',
        '_subscribers',
        '_publishers'
    ]

    def __init__(self, name: str, params: List[str]):
        assert isinstance(name, str)
        self._name = name
        self._subscribers = []
        self._publishers = []
        self._parameters = params.copy()
    
    @property
    def name(self) -> str:
        return self._name

    @property
    def subscribers(self) -> List[str]:
        return self._subscribers

    @property
    def publishers(self) -> List[str]:
        return self._publishers

    @property
    def parameters(self) -> List[str]:
        return self._parameters
 
    def update_parameters(self, params: List[str]):
        new = params.copy()
        new.sort()
        if new != self._parameters:
            self._parameters = new
            return True
        return False

    def update_pubs(self, pubs: List[rclpy.node.TopicEndpointInfo]) -> bool:
        new = []
        for elem in pubs:
            new.append(elem.node_name)
        new.sort()
        if new != self._publishers:
            self._publishers = new
        return False

    def update_subs(self, subs: List[rclpy.node.TopicEndpointInfo]) -> bool:
        new = []
        for elem in subs:
            new.append(elem.node_name)
        new.sort()
        if new != self._subscribers:
            self._subscribers = new
        return False

    def is_subscribed(self, node: str) -> bool:
        assert isinstance(node, str)
        return node in self._subscribers

    def to_yaml(self) -> str:
        s = (
            f"    - topic: {self._name}\n"
        )
        s = f"{s}      parameters:\n"
        for elem in self._parameters:
            s = f"{s}        - {elem}\n"
        s = f"{s}      publishers:\n"
        if len(self._publishers) == 0:
            s = f"{s}        - ~\n"
        else:
            for elem in self._publishers:
                s = f"{s}        - {elem}\n"
        s = f"{s}      subscribers:\n"
        if len(self._subscribers) == 0:
            s = f"{s}        - ~\n"
        else:
            for elem in self._subscribers:
                s = f"{s}        - {elem}\n"    
        return s   

class RipsNode:
    """Rips node abstraction"""

    __slots__ = [
        '_name',
        '_services',
        '_gids'
    ]

    def __init__(self, name: str):
        assert isinstance(name, str)
        self._name = name
        self._gids = []
        self._services = {} 

    def add_gid(self, gid: str) -> bool :
        assert isinstance(gid, str)
        if gid in self._gids:
            return False
        self._gids.append(gid)
        return True

    def _update_service(self, name: str, l: List[str]) -> bool :
        new = l.copy()
        new.sort()
        if not name in self._services.keys():
            self._services[name] = new;
            return True
        if new != self._services[name]:
            self._services[name] = new
            return True
        return False

    def update_services(self, l: List[Tuple[str, List[str]]]) -> bool :
        ret = False
        for elem in l:
            if self._update_service(elem[0], elem[1]):
                ret = True
        return ret

    @property
    def gids(self) -> List[str] :
        return self._gids

    @property
    def name(self) -> str:
        return self._name;

    def __eq__(self, other) -> bool :
        return self._name == other._name

    def __ne__(self, other) -> bool :
        return self._name != other._name
    
    def __gt__(self, other) -> bool :
        return self._name > other._name

    def __lt__(self, other) -> bool :
        return self._name < other._name

    def __ge__(self, other) -> bool :
        return self._name >= other._name

    def __le__(self, other) -> bool :
        return self._name <= other._name

    def to_yaml(self) -> str:
        s = f"    - node: {self._name}\n"
        s = f"{s}      gids:\n"
        for elem in self._gids:
            s = f"{s}        - {elem}\n"
        s = f"{s}      services:\n"
        for k, v in self._services.items():
            s = f"{s}        - service: {k}\n"
            s = f"{s}          params:\n"
            for param in v:
                s = f"{s}            - {param}\n"
        return s


class RipsContext:
    """Rips context abstraction"""

    __slots__ = [
        '_topics',
        '_nodes',
        '_changed'
    ]

    def __init__(self):
        self._topics = []
        self._nodes = []
        self._changed = False
    
    @property
    def topics(self) -> List[RipsTopic]:
        return self._topics;

    @property
    def nodes(self) -> List[RipsNode]:
        return self._nodes;

    def get_node(self, name: str) -> RipsNode:
        assert isinstance(name, str)
        for elem in self._nodes:
           if elem.name == name:
                return elem
        return None
          
    def update_nodes(self, nodes: List[str]):
        new = nodes.copy()
        new.sort() 
        current = []
        for elem in self._nodes:
            current.append(elem.name)
        current.sort()
        if new != current:
            self._nodes = []
            for n in new:
                self._nodes.append(RipsNode(n))
            changed = True

    def update_gids(self, l: List[rclpy.node.TopicEndpointInfo]):
        for elem in l:
            n = self.get_node(elem.node_name)
            if n:
                g = '.'.join(format(x, '02x') for x in elem.endpoint_gid)
                if n.add_gid(g):
                    self._changed = True
  
    def update_services(self, node: str, l: List[Tuple[str, List[str]]]):
        n = self.get_node(node)
        if not n:
            return
        if n.update_services(l):
            self._changed = True

    ## topics are never deleted, because RIPS will be subscribed 
    ## forever.
    def update_topic(self, name: str, 
                    params: List[str],
                    pubs: List[rclpy.node.TopicEndpointInfo],
                    subs: List[rclpy.node.TopicEndpointInfo]):
        assert isinstance(name, str)
        t = None
        for elem in self._topics:
            if elem.name == name:
                t = elem
        if not t:
            t = RipsTopic(name, params)
            self._topics.append(t)
            self._changed = True
        elif t.update_parameters(params):
            self._changed = True
        if t.update_pubs(pubs):
            self._changed = True
        if t.update_subs(subs):
            self._changed = True

    def is_subscribed(self, node: str, topic: str) -> bool:
        assert isinstance(node, str)
        assert isinstance(topic, str)
        for elem in self._topics:
            if elem.name == topic:
                return elem.is_subscribed(node)
        return False

    def check_and_clear(self) -> bool:
        ret = self._changed
        self._changed = False
        return ret

    def params_of(self, topic: str) -> List[str]:
        assert isinstance(topic, str)
        for elem in self._topics:
            if topic == elem.name:
                return elem.parameters
        return None

    def to_yaml(self) -> str:
        s = (
            f"context:\n"
            f"  nodes:\n"
        )
        for elem in self._nodes:
            s = f"{s}{elem.to_yaml()}"
        s = f"{s}  topics:\n"
        for elem in self._topics:
            s = f"{s}{elem.to_yaml()}" 
        return s


class RipsCore(Node):
    """Rips node."""

    __slots__ = [
        '_context',
        '_fifo'
    ]

    __IGNORED_TOPICS = ["/rosout"]
    __POLLING_TIME = 0.5  # secs
    __QUEUE_DEPTH = 100 
    __DEFAULT_FIFO_PATH = "/tmp/rips.fifo"

    def __init__(self):
        super().__init__('rips')
        self.create_timer(self.__POLLING_TIME, self.timer_callback)
        self._context = RipsContext()
        path = os.environ.get('RIPSFIFOPATH', self.__DEFAULT_FIFO_PATH)
        try:
            self._fifo = os.open(path, os.O_WRONLY)
        except FileNotFoundError:
            os.mkfifo(path, 0o600)
        self._fifo = os.open(path, os.O_WRONLY)
        self.get_logger().info(f"fifo {path} ready, fd: {self._fifo}")

    def _send_to_engine(self, m: str):
        assert isinstance(m, str)
        try:
            os.write(self._fifo, m.encode(encoding = 'UTF-8'))
        except:
            self.get_logger().warning(f"can't write event to the fifo")

    def _send_graph_event(self):
        s = (
                f"---\n"
                f"event: graph\n"
                f"{self._context.to_yaml()}"
                f"...\n"
        )
        self._send_to_engine(s)

    def _send_msg_event(self, topic: str, msg: str):
        assert isinstance(topic, str)
        assert isinstance(msg, str)
        s = (
                f"---\n"
                f"event: message\n"
                f"fromtopic: {topic}\n"
                f"{msg}"
                f"{self._context.to_yaml()}"
                f"...\n"
        )
        self._send_to_engine(s)

    def _subscribe(self, topic: str): 
        assert isinstance(topic, str)
        if topic in self.__IGNORED_TOPICS:
            return 
        self.get_logger().info(f"subscribing to topic {topic}")
        params = self._context.params_of(topic)
        if len(params) != 1:
            self.get_logger().warning(f"Topic {topic} has more than one type\n")
            return 
        msgtype = get_message(params[0]) 
        def f(msg): 
            self._send_msg_event(topic, message_to_yaml(msg));
            self.get_logger().info(f"Received from topic {topic}")
        subscription = self.create_subscription(
            msgtype,
            topic,
            f,
            self.__QUEUE_DEPTH,
            raw=False
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
            self.get_logger().info("Context changed}")
        
def main(args=None):
    rclpy.init(args=args)
    rips_core = RipsCore()
    rclpy.spin(rips_core)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rips_core.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()