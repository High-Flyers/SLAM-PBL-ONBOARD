import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hf2137/ros2_salamanca_ws/install/rviz_lidar_view'
