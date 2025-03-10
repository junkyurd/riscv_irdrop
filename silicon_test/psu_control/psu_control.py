################################################################################################################################
# Power Supply Operation
################################################################################################################################
import pyvisa
import time

psu_address = 'USB0::10893::36609::CN61050090::0::INSTR'
rm = pyvisa.ResourceManager()
psu = rm.open_resource(psu_address)

def psu_operation(volt, curr, channel, operation):
    # set the selected channel into provided voltage and current level
    # operation '1' to turn on, '0' to turn off
    psu.write('INST:NSEL '+str(channel)) # selects channel
    if (operation):
        psu.write('SOUR:VOLT:LEV:IMM:AMPL ' + str(volt))
        psu.write('SOUR:CURR:LEV:IMM:AMPL ' + str(curr))
        psu.write('OUTP:STAT 1') # enables channel
    else:
        psu.write('OUTP:STAT 0') # disables channel

def psu_reading(channel):
    psu.write('INST:NSEL '+str(channel)) # selects channel
    curr = psu.query('MEAS:SCAL:CURR:DC?')
    volt = psu.query('MEAS:SCAL:VOLT:DC?')
    return float(volt), float(curr)

#psu_operation(1.8, 5.0, 1, 0)
#psu_operation(1.0, 1.0, 2, 0)
#psu_operation(0.9, 1.0, 3, 0)
#time.sleep(1)
# psu_operation(1.8, 5.0, 1, 1)
# psu_operation(1.0, 1.0, 2, 1)
# psu_operation(0.9, 1.0, 3, 1)