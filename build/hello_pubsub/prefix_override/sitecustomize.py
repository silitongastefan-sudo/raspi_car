import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/stefan/Desktop/ros2_ws/install/hello_pubsub'
