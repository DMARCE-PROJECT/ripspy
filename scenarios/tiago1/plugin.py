#!/usr/bin/python3

import sys
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

def main():
  data = sys.stdin.buffer.read()
  msgtype = get_message("std_msgs/msg/String")
  try:
    msg = deserialize_message(data, msgtype)
  except Exception as e:
    sys.exit(1)
  else:      
      if msg.data == "detected":
        sys.exit(0)
      elif msg.data == "clear":
        sys.exit(1)
  sys.exit(2)
if __name__ == '__main__':
    main()
21
