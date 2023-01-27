import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import typing
from typing import List
from typing import Dict

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
        # Subscripted generics cannot be used with class and instance checks
        # assert isinstance(param, List[str])
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
        # Subscripted generics cannot be used with class and instance checks
        # assert isinstance(params, List[str])
        new = params.copy()
        new.sort()
        if new != self._parameters:
            self._parameters = new
            return True
        return False

    def update_pubs(self, pubs: List[rclpy.node.TopicEndpointInfo]) -> bool:
        # Subscripted generics cannot be used with class and instance checks
        # assert isinstance(pubs, List[rclpy.node.TopicEndpointInfo])
        new = []
        for info in pubs:
            new.append(info.node_name)
        new.sort()
        if new != self._publishers:
            self._publishers = new
        return False

    def update_subs(self, subs: List[rclpy.node.TopicEndpointInfo]) -> bool:
        # Subscripted generics cannot be used with class and instance checks
        # assert isinstance(subs, List[rclpy.node.TopicEndpointInfo])
        new = []
        for info in subs:
            new.append(info.node_name)
        new.sort()
        if new != self._subscribers:
            self._subscribers = new
        return False

    def __str__(self) -> str:
        s = self._name + "\n"
        for p in self._parameters:
            s = s + "   parameter: " + p + "\n"
        for p in self._publishers:
            s = s + "   publisher: " + p + "\n"
        for s in self._subscribers:
            s = s + "   subscriber: " + s + "\n"    
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
        # Subscripted generics cannot be used with class and instance checks
        # assert isinstance(nodes, List[str])
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
        for info in l:
            n = info.node_name
            g = '.'.join(format(x, '02x') for x in info.endpoint_gid)
            if n in self._nodes:
                if not g in self._nodes[n]:
                    self._nodes[n].append(g)
                    self._changed = True
             
    def update_topic(self, name: str, 
                    params: List[str],
                    pubs: List[rclpy.node.TopicEndpointInfo],
                    subs: List[rclpy.node.TopicEndpointInfo]):
        assert isinstance(name, str)
        # Subscripted generics cannot be used with class and instance checks
        # assert isinstance(params, Any) # it does not work with List[str])
        # assert isinstance(pubs, List[rclpy.node.TopicEndpointInfo])
        # assert isinstance(subs, List[rclpy.node.TopicEndpointInfo])
        t = None
        print(" >>>>>> name " + name)
        for item in self._topics:
            if item.name == name:
                t = item
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
        s = ""
        for k, v in self._nodes.items():
            s = s + "node: " + k + "\n" 
            for g in v:
                    s = s + "   gid: " + g + "\n"
        for t in self._topics:
            s = s + "topic: " + str(t) 
        return s

    def check_and_clear(self) -> bool:
        ret = self._changed
        self._changed = False
        return ret


class RipsCore(Node):
    """Rips node."""

    __slots__ = [
        '_context'
    ]

    def __init__(self):
        super().__init__('rips')
        self.create_timer(0.5, self.timer_callback)
        self.subscription = self.create_subscription(
            String,
            'ripsfake',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self._context = RipsContext()

    @property
    def context(self):
        return self._context;

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def timer_callback(self):
        self._context.update_nodes(self.get_node_names())
        topics = self.get_topic_names_and_types()
        for topic in topics:
            pubs = self.get_publishers_info_by_topic(topic[0])
            subs = self.get_subscriptions_info_by_topic(topic[0])
            self._context.update_topic(topic[0], topic[1], pubs, subs)
            self._context.update_gids(pubs)
            self._context.update_gids(subs)
        if self._context.check_and_clear():
            self.get_logger().info("changes:\n%s\n" % self._context)
                    
        
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