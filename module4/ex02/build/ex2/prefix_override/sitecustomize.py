import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ilya-trushkin/ros2_module4/src/ex2/install/ex2'
