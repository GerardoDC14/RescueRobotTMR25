import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gerardo/Projects/Robotics/RescueRobot2025/jaguar_odom/install/jaguar_odom'
