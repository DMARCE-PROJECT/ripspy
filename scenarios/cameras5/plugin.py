#!/usr/bin/python3

import sys
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

def main():   
  data = sys.stdin.buffer.read()
  msgtype = get_message("rcl_interfaces/msg/Log")
  try:
    msg = deserialize_message(data, msgtype)
  except Exception as e: 
    sys.exit(1)
  else:
    if msg.level == 10 and msg.name == "patata" and msg.msg == "pocha111" and msg.file == "nofile" and msg.function == "nofunc":  
      sys.exit(0)
  sys.exit(1)

if __name__ == '__main__':
    main()