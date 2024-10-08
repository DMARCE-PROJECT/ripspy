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

from typing import Dict, List, Tuple

import rclpy

# All missing asserts for isinstance are deleted because of
# this error: Subscripted generics cannot be used with class
# and instance checks. For example, this one raises the error:
# assert isinstance(pubs, List[rclpy.node.TopicEndpointInfo])

class RipsTopic:
    """Data of interest for a topic"""

    __slots__ = [
        '_name',
        '_parameters',
        '_subscribers',
        '_publishers'
    ]

    _name: str
    _subscribers: List[str]
    _publishers: List[str]
    _parameters: List[str]

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

    _gids: List[str]
    _services: Dict[str, List[str]]

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

    def to_yaml(self) -> str:
        s = f"    - node: {self._name}\n"
        s = f"{s}      gids:\n"
        if len(self._gids) == 0:
            s = f"{s}        - ~\n"
        else:
            for elem in self._gids:
                s = f"{s}        - {elem}\n"
        s = f"{s}      services:\n"
        if len(self._services) == 0:
            s = f"{s}        - ~\n"
        else:
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

    _topics: List[RipsTopic]
    _nodes: List[RipsNode]
    _changed: bool

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
        raise Exception("no such node")

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
            try:
                n = self.get_node(elem.node_name)
                g = '.'.join(format(x, '02x') for x in elem.endpoint_gid)
                if n.add_gid(g):
                    self._changed = True
            except:
                pass

    def update_services(self, node: str, l: List[Tuple[str, List[str]]]):
        try:
            n = self.get_node(node)
            if n.update_services(l):
                self._changed = True
        except:
            return

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
        raise Exception("No such topic")

    def to_yaml(self) -> str:
        s = (
            f"context:\n"
            f"  nodes:\n"
        )
        for n in self._nodes:
            s = f"{s}{n.to_yaml()}"
        s = f"{s}  topics:\n"
        for t in self._topics:
            s = f"{s}{t.to_yaml()}"
        return s
