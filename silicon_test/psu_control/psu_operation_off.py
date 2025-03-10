from psu_control import *

# ch 1 top
psu_operation(volt=0, curr=5, channel=1, operation='off')
# ch 2 primary
psu_operation(volt=0, curr=1, channel=2, operation='off')
# ch 3 secondary
psu_operation(volt=0, curr=1, channel=3, operation='off')

time.sleep(1)