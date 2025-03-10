from psu_control import *

# ch 1 top
psu_operation(volt=0.9, curr=5, channel=1, operation='on')
# ch 2 primary
psu_operation(volt=0.9, curr=1, channel=2, operation='on')
# ch 3 secondary
psu_operation(volt=0.9, curr=1, channel=3, operation='on')

time.sleep(1)