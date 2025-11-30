import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/karan_k/interbotix_ws/install/interbotix_xs_modules'
