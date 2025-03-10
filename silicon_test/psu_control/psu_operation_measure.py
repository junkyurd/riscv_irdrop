from psu_control import *

# ch 1 top
psu_operation(volt=1, curr=5, channel=1, operation=1)
# ch 2 primary
psu_operation(volt=1, curr=1, channel=2, operation=1)
# ch 3 secondary
psu_operation(volt=0.9, curr=1, channel=3, operation=1)

time.sleep(1)