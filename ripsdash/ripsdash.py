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

import socket
import sys
import os
import yaml
from time import sleep
from datetime import datetime

from rich import box
from rich.align import Align
from rich.console import Console, Group
from rich.layout import Layout
from rich.panel import Panel
from rich.progress import BarColumn, Progress, SpinnerColumn, TextColumn
from rich.syntax import Syntax
from rich.table import Table
from rich.tree import Tree
from rich import print
from rich.live import Live
from rich.console import Console
from rich.text import Text

MaxMsgs = 20
MaxAlerts = 5
TheConsole = None

class Header:
    """Ripspy Dash Header"""
    
    __slots__ = [
        '_currentlevel',
        '_currentgrav',
    ]

    _currentlevel: str
    _currentgrav: float
    _lastalert: str

    def __init__(self,  level: str, grav: float):
        self._currentlevel = level
        self._currentgrav = grav

    def __rich__(self) -> Panel:
        grid = Table.grid(expand = True)
        grid.add_column(justify="left", ratio = 3)
        grid.add_column(justify="right")
        grid.add_row(
            "[b]RIPSpy term dashboard[/b]",
            datetime.now().ctime().replace(":", "[blink]:[/]"),
        )
        grid.add_row(
            f"[b]Current Level:[/b] {self._currentlevel}",
            "",
        )
        style = "black on bright_red"
        if self._currentgrav < 0.25:
            style = "black on light_green"
        elif self._currentgrav < 0.50:
            style = "black on yellow3"
        elif self._currentgrav < 0.75:
            style = "black on orange3"
        return Panel(grid, style = style)

def make_layout() -> Layout:
    """Define the layout."""
    layout = Layout(name="root")
    layout.split(
        Layout(name="header", size=4),
        Layout(name="main", ratio=1),
        Layout(name="footer", size=MaxMsgs+2),
        Layout(name="footeralerts", size=MaxAlerts+2),
    )
    layout["main"].split_row(Layout(name="box_topics"), Layout(name="box_nodes"))
    layout["header"].update(Header("init", 0.0))
    layout["footer"].update(Panel("", title="msgs", border_style="green"))
    layout["footeralerts"].update(Panel("", title="alerts", border_style="green"))
    return layout

def update_nodes(context, layout):
    added = []
    naux = []
    nodes = context["nodes"]
    for node in nodes:
        naux.append(node["node"])
    tree = Tree("Nodes:")
    for node in nodes:
        t = None
        if naux.count(node["node"]) > 1:
            if not node["node"] in added:
                    t = tree.add(":warning: [bold red]" + node["node"] + " (dup)")
                    added.append(node["node"])
            else:
                continue
        else: 
            t = tree.add(":black_circle_for_record: " + node["node"])
        #tserv = t.add("Services")
        #for serv in node["services"]:
        #    tserv.add(serv["service"])
    layout["box_nodes"].update(Panel(tree, border_style="green"))

def add_nodes_to_topic(nodes, tree):
    if nodes == None:
        return
    added = []
    for n in nodes:
        if n != None:
            if nodes.count(n) > 1: 
                if not n in added:
                    tree.add(":warning: [bold red]" + n + " (dup)")
                    added.append(n)
            else:
                tree.add(n)
                            
def update_topics(context, layout):
    tree = Tree("Topics")
    for topic in context["topics"]:
        t = tree.add(":black_circle_for_record: " + topic["topic"])
        tpubs = t.add("Publishers")
        add_nodes_to_topic(topic["publishers"], tpubs)
        tsubs = t.add("Subscribers") 
        add_nodes_to_topic(topic["subscribers"], tsubs)
    layout["box_topics"].update(Panel(tree, border_style="green"))

last_msgs = ""

def update_msgs(d, layout):
    global last_msgs
    t = d["fromtopic"]
    m = d["msg"]
    msgdata = ""
    if "data" in m.keys():
        msgdata = f"String content: {m['data']}"
    else:
        msgdata = f"Base64 content: {d['rawmsg']}".strip()
    msgdata = "".join(msgdata.splitlines())
    now = datetime.now()
    curtime = now.strftime("%H:%M:%S")
    line = f"✉ {curtime} msg to topic:{t} {msgdata}"
    last_msgs = f"{last_msgs}{line}\n"
    l = last_msgs.splitlines()
    if len(l) > MaxMsgs:
        l.pop(0)
        last_msgs = "\n".join(l) + "\n"
    t = Text(last_msgs, overflow="ellipsis", no_wrap=True)
    layout["footer"].update(Panel(t, title="msgs", border_style="green"))

last_alerts = ""

def update_alerts(d, layout):
    global last_alerts
    new = d["lastalert"]
    if new == "":
        return
    new = "".join(new.splitlines())
    l = last_alerts.splitlines()
    if len(l) != 0 and l[len(l)-1] == new:
        return
    l.append(new)
    if len(l) > MaxAlerts:
        l.pop(0)
    last_alerts = "\n".join(l) + "\n"
    t = Text(last_alerts, overflow="ellipsis", no_wrap=True)
    layout["footeralerts"].update(Panel(t, title="alerts", border_style="green"))

def update_header(d, layout):
    level = d["currentlevel"]
    grav = d["currentgrav"]
    layout["header"].update(Header(level, grav))

def update(d, layout):
    if d["event"] == "message":
        update_msgs(d, layout)
        None
    elif d["event"] == "graph":
        context = d["context"]
        update_nodes(context, layout)
        update_topics(context, layout)
    update_alerts(d, layout)
    update_header(d, layout)

def mainloop(sock: socket.socket):
    f = os.fdopen(sock.fileno())
    start = False
    s = "" 
    layout = make_layout()
    with Live(layout, refresh_per_second=10, screen=True) as live:
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
                except:
                    print(f"ERROR: sockthread: bad YAML\n{s}\n", file=sys.stderr)
                    continue
                update(d, layout)
                
def main(args=None):
    global TheConsole
    TheConsole = Console()
    teesockpath = os.environ.get('RIPSTEESOCKET', "")
    try:
        os.unlink(teesockpath)
    except OSError:
        if os.path.exists(teesockpath):
            raise
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(teesockpath)
    TheConsole.clear()
    with TheConsole.status("[bold green]Waiting for RIPSpy....") as status:
        sock.listen(1)
        connection, client_address = sock.accept()
    mainloop(connection)

if __name__ == '__main__':
    main()