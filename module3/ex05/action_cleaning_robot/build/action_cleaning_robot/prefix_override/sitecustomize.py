import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ilya-trushkin/ros2_module3/src/action_cleaning_robot/install/action_cleaning_robot'
