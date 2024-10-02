# ripspy

Python RIPS monitor prototype.

# Build

If you want to change the engine, build the Golang program
(https://github.com/DMARCE-PROJECT/rips) and copy the binary to _./bin/rips_.

```
cd $myworkspace
colcon build --packages-select ripspy --symlink-install
```

# Run

You have to set some environment variables before running it:

* *RIPRULES*: the path for a Rips rules file.

* *RIPSCRIPTS*: is the path for the directory that contains the scripts
required.

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

Optional environment variables:

* *RIPSBLACKLIST*: A list of topics seppared by ':'. The monitor will ignore
those topics. /rosout is allways in the black list in order to avoid loops.

* *RIPSWHITELIST*: A list of topics sepparated by ':'. If the variable exists
  and the list is not empty, the monitor will be subscribed only to those
  topics (if they are not in the black list).

* *RIPSSOCKET*: The path for the UNIX domain socket for the engine. If this
variable is not set, */tmp/rips.socket* is used by default.

* *RIPSTEESOCKET*: The path for the UNIX domain socket for the dashboard.
It must be set if the dashboard is executing.

# Docs

See:

https://github.com/DMARCE-PROJECT/Documentation/tree/main/WP3/D3.03.RIPS-Technical-Report

# Examples

You can find several examples in the _scenarios_ directory.
