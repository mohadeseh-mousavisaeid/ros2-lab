import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/projectlab3_ss25/test_project/install/obstacle_avoidance_controller'
