import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/widha893/ros2_hil_quadrotor/install/sensor_data_logger'
