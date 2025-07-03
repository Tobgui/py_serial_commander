import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tobgui/titi/src/py_serial_commander/install/py_serial_commander'
