import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rhobtor/PHD/ARGOS_J8/ARGOSJ8_IA_/install/car'
