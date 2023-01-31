import inspect

import typing
from typing import Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rosidl_runtime_py.utilities import get_message

import sys

from rclpy.qos_event import SubscriptionEventCallbacks

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

    def __str__(self) -> str:
        s = self._name + "\n"
        for elem in self._parameters:
            s = f"{s}   parameter: {elem}\n"
        for elem in self._publishers:
            s = f"{s}   publisher: {elem}\n"
        for elem in self._subscribers:
            s = f"{s}   subscriber: {elem}\n"    
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
        self._nodes = {} 
        self._changed = False
    
    @property
    def topics(self) -> List[RipsTopic]:
        return self._topics;

    @property
    def nodes(self) -> Dict[str, List[str]]:
        return self._nodes;

    def update_nodes(self, nodes: List[str]):
        new = nodes.copy()
        new.sort() 
        current = list(self._nodes.keys())
        current.sort()
        if new != current:
            self._nodes = {}
            for n in new:
                self._nodes[n] = []
            changed = True

    def update_gids(self, l: List[rclpy.node.TopicEndpointInfo]):
        for elem in l:
            n = elem.node_name
            g = '.'.join(format(x, '02x') for x in elem.endpoint_gid)
            if n in self._nodes:
                if not g in self._nodes[n]:
                    self._nodes[n].append(g)
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

    def __str__(self) -> str:
        s = "NODES:\n"
        for k, v in self._nodes.items():
            s = f"{s}{k}\n" 
            for g in v:
                    s = f"{s}   gid: {g}\n"
        s = f"{s}TOPICS:\n"
        for elem in self._topics:
            s = f"{s}{str(elem)}" 
        return s

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

class RipsCore(Node):
    """Rips node."""

    __slots__ = [
        '_context',
    ]

    __IGNORED_TOPICS = ["/rosout"]
    __POLLING_TIME = 0.5  # secs

    def __init__(self):
        super().__init__('rips')
        self.create_timer(self.__POLLING_TIME, self.timer_callback)
        self._context = RipsContext()

    @property
    def context(self):
        return self._context;

    def topic_callback(self, msg):
        datatype = msg.get_fields_and_field_types()['data']
        dataclass = msg.__class__
        s = (
            f"RIPS: RECEIVED MESSAGE:\n"
            f"  type:{datatype}\n"
            f"  class:{dataclass}\n"
        )
        self.get_logger().info(s)
        # l = inspect.getmembers(msg)
        # for elem in l:
        #     print(elem)
        #     print("\n")
##        sys.exit(0)

    def _subscribe(self, topic: str): 
        assert isinstance(topic, str)
        if topic in self.__IGNORED_TOPICS:
            return 
        self.get_logger().info(f"subscribing to topic {topic}")
        msgtype = get_message(self._context.params_of(topic)[0]) ## what if there are more than 1 parameter
        eventcb=SubscriptionEventCallbacks(liveliness=lambda event: print(f'MGS EVENT: {event}'))
        subscription = self.create_subscription(
            msgtype,
            topic,
            self.topic_callback,
            100, ## history depth (queue)
            event_callbacks=eventcb) 

    def event_callback(self, x):
        print('event callback')

    def timer_callback(self):
        self._context.update_nodes(self.get_node_names())
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
            m = (
                f"\n-------------------\n{self._context}"
                f"-------------------\n"
            )
            self.get_logger().info(m)

        
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