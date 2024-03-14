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
