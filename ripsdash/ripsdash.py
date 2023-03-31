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

MaxMsgs = 20
TheConsole = None

class Header:
    """Display header with clock."""

    def __rich__(self) -> Panel:
        grid = Table.grid(expand=True)
        grid.add_column(justify="center", ratio=1)
        grid.add_column(justify="right")
        grid.add_row(
            "[b]RIPSpy[/b] term dashboard",
            datetime.now().ctime().replace(":", "[blink]:[/]"),
        )
        return Panel(grid, style="white on blue")

def make_layout() -> Layout:
    """Define the layout."""
    layout = Layout(name="root")

    layout.split(
        Layout(name="header", size=3),
        Layout(name="main", ratio=1),
        Layout(name="footer", size=MaxMsgs+2),
    )
    layout["main"].split_row(Layout(name="box_topics"), Layout(name="box_nodes"))
    layout["header"].update(Header())
    layout["footer"].update(Panel("", title="msgs", border_style="green"))
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
        tserv = t.add("Services:")
        for serv in node["services"]:
            tserv.add(serv["service"])
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
    tree = Tree("Topics:")
    for topic in context["topics"]:
        t = tree.add(":black_circle_for_record: " + topic["topic"])
        tpubs = t.add("Publishers:")
        add_nodes_to_topic(topic["publishers"], tpubs)
        tsubs = t.add("Subscribers:") 
        add_nodes_to_topic(topic["subscribers"], tsubs)
    layout["box_topics"].update(Panel(tree, border_style="green"))

last_msgs = ""

def update_msgs(d, layout):
    global last_msgs
    t = d["fromtopic"]
    m = d["msg"]
    msgdata = ""
    if "data" in m.keys():
        msgdata = f"String content: [i]{m['data']}[/i]"
    else:
        msgdata = f"Base64 content: [i]{d['rawmsg']}[/i]".strip()
    now = datetime.now()
    curtime = now.strftime("%H:%M:%S")
    line = f"[r]:envelope: {curtime}[/r] msg to topic:[b]{t}[/b] {msgdata}"
    line = line[:TheConsole.width-4] ## adjust the width to the console
    last_msgs = f"{last_msgs}{line}\n"
    # remove the first N lines to fit the footer size
    l = last_msgs.splitlines()
    if len(l) > MaxMsgs:
        to_remove = len(l)-MaxMsgs
        last_msgs = "\n".join(last_msgs.split("\n")[to_remove:])
    layout["footer"].update(Panel(last_msgs, title="msgs", border_style="green"))

def update(d, layout):
    if d["event"] == "message":
        update_msgs(d, layout)
    elif d["event"] == "graph":
        context = d["context"]
        update_nodes(context, layout)
        update_topics(context, layout)

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