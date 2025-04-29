import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bara/ros2_ws/src/amr_webots_sim/install/amr_webots_sim'
