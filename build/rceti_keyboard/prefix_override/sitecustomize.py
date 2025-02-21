import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user/Documents/Capstone/RCETI/install/rceti_keyboard'
