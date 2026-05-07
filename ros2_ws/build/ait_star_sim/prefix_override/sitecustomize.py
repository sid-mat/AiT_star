import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jigar/AiT_star/ros2_ws/install/ait_star_sim'
