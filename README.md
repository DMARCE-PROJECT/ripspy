# ripspy

Python RIPS monitor prototype.

# Build

If you want to change the engine, build the Golang program
rips and copy the binary to bin.

```
cd $myworkspace
colcon build --packages-select ripspy --symlink-install
```

# Run

You have to set some environment variables before running it. *RIPRULES* is
the path for a Rips rules file. You have some examples in the directory
_scenarios_. *RIPSCRIPTS* is the path to the directory with the scripts
required for the selected rule file.

For example:

```
cd $myworkspace
. install/setup.bash
export RIPSRULES=r.rul
export RIPSSCRIPTS=myscripts
ros2 run ripspy ripspy
```

The ROS2 packages for message types to be inspected by Rips must
be properly installed.

# Status

## Subexpressions for message events

### Covered

• topicin(topics:set of string): This function returns true if the topic name
of the message is included in the specified set of topic names.

• topicmatches(regex:string): The function’s parameter is a regular expres-
sion. The function returns true if the topic name (the whole name, including
the namespace) of a message matches the regular expression.

• publishercount(min:int, max:int): The function returns true if the number
of participants publishing to the topic of this message are in the range defined
by the interval [min..max].

• subscribercount(min:int, max:int): The function returns true if the num-
ber of participants subscribed to the topic of this message are in the range defined
by the interval [min..max].

• publishersinclude(pubs:set of string): The parameter is a set of strings
with the names of participants (nodes). The function returns true if all the
participants of the set are publishers for the topic of the message.

• subscribersinclude(subs:set of string): The parameter is a set of strings
with the names of participants (nodes). The function returns true if all the
participants of the set are subscribers of the topic of the message.

• publishers(pubs: set of string): The parameter is a set of strings with the
names of participants (nodes). The function returns true if the participants of
the set are exactly the publishers for the topic of the message.

• subscribers(subs: set of string): The parameter is a set of strings with
the names of participants (nodes). The function returns true if the participants
of the array are exactly the subscribers of the topic of the message.

• msgtypein(msgs: set of string): The parameter is a set of strings
with the names of ROS2 message types ("std msgs", "sensor msgs",
"diagnostic msgs", "geometry msgs", "nav msgs", "shape msgs",
"stereo msgs", "trajectory msgs", and "visualization msgs"). The
function returns true if the message type of the message is included in the set.

• msgsubtype(msg:string, submsg:string): The first parameter is the type of
the message, the second one is the subtype of the message. For example, if the
type is "std msgs", the subtype can be "ColorRGBA", "Empty", or "Header".
The function returns true if the message has the specified type and subtype.

• plugin(id:string): The plugin specified by the parameter is invoked. A plugin
is a C++ or Python class which is able to use the ROS2 libraries to inspect and
analyze the ROS2 message. A method of this class is invoked, passing the ROS2
message as an argument. Plugins permit the user to create extensions for custom
analysis. For example, the user can create a plugin that analyzes the frames sent
by a camera component in order to detect a camera blinding attack.

• payload(path:string): This function invokes YARA to find patterns in the
payload of the message. YARA is the de facto standard for textual and bi-
nary pattern matching in malware analysis. It is widely used to detect malware
signatures. The RIPS must be able to inspect the payload of the robotic mes-
sages, because they can be used to inject malicious code in the components. This
function executes the YARA engine over the payload of the message to find the
patterns specified by the YARA rules stored in the file passed as argument. If
any of these YARA rules matches, the function returns true.

• eval(var:string, operator:string, value:string) This function is used
to check the values of the variables. The second parameter is the textual rep-
resentation of an operator. The following operators are defined for all the basic
types: ==, ! =, <, >, <=, >=. The third parameter is the textual representation
of the value for the corresponding basic type.

## Subexpressions for graph events

### Covered

• nodes(n: set of string): This function returns true if the current nodes of
the graph are just the ones defined by the set passed as argument.

• nodesinclude(n: set of string): This function returns true if the current
nodes of the graph are included in the set passed as argument.

• nodecount(min:int, max:int): If the current number of nodes is in the interval
[min..max], the function returns true.

• eval(varname:string, operator:string, value:string): Its the same
function explained in Section 4.4.1, used to check the values of variables.

• service(node:string, serv:string): This function returns true if the node
specified by the first parameter is providing the service specified by the second
parameter.

• services(s:set of string)

• servicesinclude(s:set of string)

• servicecount(min:int, max:int)

The semantics of the following functions are similar to the semantics of the previous
ones (i.e., nodes, nodeinclude, and nodecount), but they are oriented to topics,
services, topic’s subscribers, and topic’s publishers:

• topics(n:set of string)

• topicsinclude(n:set of string)

• topiccount(min:int, max:int)

• subscribers(topic:string, nodes:set of string)

• subscribersinclude(topic:string, nodes:set of string)

• subscribercount(topic:string, min:int, max:int)

• publishers(topic:string, nodes:set of string)

• publishersinclude(topic:string, nodes:set of string)

• publishercount(topic:string, min:int, max:int)


## Subexpressions for external events

Those are not based on info provided by the monitor:

• idsalert(alert:string): The RIPS can use a conventional IDS/NIPS/HIPS
to detect low-level network threats (e.g., to detect suspect messages at network
or transport levels). A special expression can be used to react to a low level
detection: This function returns true when the alert specified in the parameter is
triggered by the underlying detection system. In our prototype, we use Snort [70]
as a low-level IDS.

• signal(sig:string): It can be used to react to user-defined Unix signals. It
may be useful to trigger emergency actions from a system shell if necessary. This
function returns true when the RIPS process receives a SIGUSR1 or SIGUSR2
signal.
